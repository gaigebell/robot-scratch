"""Optional LAN WebRTC input adapter. No workflow, IK or device commands here.

One reliable ordered channel carries the existing webxr-input/1 snapshots.
HTTPS exchanges the offer/answer; aiortc owns DTLS/SCTP/ICE. All application
calls are sequential on one worker, outside the networking event loop.
"""

import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeout


class WebRTCInputServer:
    """Own the peer/event loop; serialize application calls on one worker.

    The bounded channel queue retains ordered button snapshots. Closing the
    adapter pauses the current owner before canceling tasks and joining threads.
    """

    def __init__(self, app):
        # Optional imports: HTTP/WSS installations do not need aiortc.
        from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

        self.Configuration, self.Peer, self.Description = (
            RTCConfiguration,
            RTCPeerConnection,
            RTCSessionDescription,
        )
        self.app = app
        self.loop = asyncio.new_event_loop()
        self.peer = None
        self.tasks = set()
        self.closed = False
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="xr-input")
        self.active_pause = None
        self.thread = threading.Thread(target=self._run, name="xr-webrtc", daemon=True)
        self.thread.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        self.loop.close()

    def offer(self, body):
        if self.closed:
            raise ValueError("WebRTC listener closed")
        if (
            not isinstance(body, dict)
            or body.get("type") != "offer"
            or not isinstance(body.get("sdp"), str)
        ):
            raise ValueError("SDP offer required")
        if len(body["sdp"]) > 60000:
            raise ValueError("SDP too large")
        future = asyncio.run_coroutine_threadsafe(self._offer(body), self.loop)
        try:
            return future.result(timeout=12)
        except FutureTimeout:
            future.cancel()
            raise ValueError("WebRTC negotiation timed out") from None

    async def _offer(self, body):
        if self.peer is not None:
            raise ValueError("WebRTC input already connected; close the previous page first")
        pc = self.Peer(self.Configuration(iceServers=[]))
        self.peer = pc
        queue = asyncio.Queue(maxsize=32)
        sid = None
        channel = None
        paused = False

        async def pause(reason):
            nonlocal paused
            if not paused and sid and self.app.session and self.app.session.viewer == sid:
                paused = True
                await self.loop.run_in_executor(self.worker, self.app.pause, reason)

        self.active_pause = pause

        async def release(reason):
            await pause(reason)
            await pc.close()
            if self.peer is pc:
                self.peer = None

        async def consume():
            nonlocal sid
            try:
                while True:
                    raw = await queue.get()
                    began = time.monotonic()
                    try:
                        packet = json.loads(raw)
                        if not isinstance(packet, dict):
                            raise ValueError("input object required")
                        sid = packet["session_id"]
                        result = await self.loop.run_in_executor(
                            self.worker, self.app.input, packet
                        )
                        response = {
                            "type": "ack",
                            "seq": packet["seq"],
                            "result": result,
                            "server_s": time.monotonic(),
                            "server_timing": {"input_ms": (time.monotonic() - began) * 1000},
                        }
                    except (ValueError, KeyError, PermissionError) as exc:
                        response = {
                            "type": "error",
                            "error": str(exc),
                            "server_s": time.monotonic(),
                        }
                    if channel.readyState != "open":
                        return
                    if channel.bufferedAmount > 65536:
                        raise ValueError("WebRTC reply backlog")
                    channel.send(json.dumps(response, allow_nan=False))
            except asyncio.CancelledError:
                raise
            except Exception:
                await release("webrtc_input_failure")

        def spawn(coro):
            task = self.loop.create_task(coro)
            self.tasks.add(task)
            task.add_done_callback(self.tasks.discard)
            return task

        @pc.on("datachannel")
        def on_channel(dc):
            nonlocal channel
            if (
                channel is not None
                or dc.label != "robot-input"
                or not dc.ordered
                or dc.maxRetransmits is not None
                or dc.maxPacketLifeTime is not None
            ):
                spawn(release("webrtc_invalid_channel"))
                return
            channel = dc
            consumer = spawn(consume())

            def ready():
                dc.send(
                    json.dumps(
                        {
                            "type": "ready",
                            "schema_version": "webxr-control/1",
                            "server_s": time.monotonic(),
                        }
                    )
                )

            @dc.on("message")
            def message(raw):
                if not isinstance(raw, str) or len(raw.encode("utf8")) > 65536 or queue.full():
                    spawn(release("webrtc_input_backlog"))
                    return
                queue.put_nowait(raw)

            @dc.on("close")
            def closed():
                consumer.cancel()
                spawn(release("webrtc_channel_closed"))

            if dc.readyState == "open":
                ready()
            else:
                dc.on("open", ready)

        @pc.on("connectionstatechange")
        async def state_change():
            if pc.connectionState in ("failed", "closed"):
                await pause("webrtc_connection_closed")
                if self.peer is pc:
                    self.peer = None
                if pc.connectionState == "failed":
                    spawn(release("webrtc_connection_failed"))

        async def connection_deadline():
            await asyncio.sleep(15)
            if channel is None or channel.readyState != "open":
                await release("webrtc_connection_timeout")

        spawn(connection_deadline())

        try:
            await pc.setRemoteDescription(self.Description(sdp=body["sdp"], type=body["type"]))
            if pc.sctp is None or pc.getTransceivers():
                raise ValueError("one data-only WebRTC connection required")
            await pc.setLocalDescription(await pc.createAnswer())
            return {"type": pc.localDescription.type, "sdp": pc.localDescription.sdp}
        except BaseException:
            await release("webrtc_negotiation_failed")
            raise

    async def _shutdown(self):
        if self.active_pause:
            await self.active_pause("webrtc_server_shutdown")
        if self.peer:
            await self.peer.close()
        # Channel-close callbacks may schedule a final pause/close operation.
        await asyncio.sleep(0)
        tasks = list(self.tasks)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        await self.loop.shutdown_default_executor()
        self.worker.shutdown(wait=True)

    def close(self):
        if self.closed:
            return
        self.closed = True
        asyncio.run_coroutine_threadsafe(self._shutdown(), self.loop).result(timeout=5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("WebRTC thread did not stop")
