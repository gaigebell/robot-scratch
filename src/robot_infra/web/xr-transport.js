// Existing HTTP/WSS: one in-flight request. Optional WebRTC: bounded continuous
// sending on ONE reliable ordered channel, using the same packets/workflow.
export class InputTransport {
  constructor(onError) {
    this.onError = onError;
    this.edges = [];
    this.latest = null;
    this.sending = false;
    this.seq = 0;
    this.serverTime = 0;
    this.skipped = 0;
    this.errors = 0;
    this.lastRttMs = 0;
    this.failed = false;
    this.stopSent = false;
  }
  fail(error) {
    if (this.failed) return;
    this.failureDetails = this.debugState();
    this.failed = true;
    this.clear();
    clearTimeout(this.rtcOpenTimer);
    for (const item of this.inflight?.values() || []) clearTimeout(item.timer);
    this.inflight?.clear();
    this.peer?.close();
    this.onError(error);
  }
  debugState() {
    const times = Array.from(this.inflight?.values() || [], (item) => item.started);
    return {
      unacked: this.inflight?.size || 0,
      buffered_bytes: this.channel?.bufferedAmount || 0,
      oldest_reply_age_ms: times.length ? performance.now() - Math.min(...times) : 0,
      last_ack_seq: this.lastAckSeq ?? null,
      next_seq: this.seq,
      connection_state: this.peer?.connectionState || null,
    };
  }
  start() {
    if (this.failed || this.active) return;
    this.active = true;
    this.configure(this.config);
  }
  suspend() {
    this.active = false;
    this.clear();
    if (this.config?.input !== "webrtc") return;
    clearTimeout(this.rtcOpenTimer);
    for (const item of this.inflight?.values() || []) clearTimeout(item.timer);
    this.inflight?.clear();
    this.rtcReady = false;
    if (this.channel) {
      this.channel.onmessage = null;
      this.channel.onclose = null;
      this.channel.onerror = null;
    }
    if (this.peer) {
      this.peer.onconnectionstatechange = null;
      this.peer.close();
    }
    this.peer = null;
    this.channel = null;
    this.route = "WebRTC 待进入 VR";
  }
  configure(config) {
    if (this.failed) return;
    this.config = config || {};
    if (this.config.input === "webrtc" && this.active && !this.peer) {
      const opening = this.openRTC(),
        peer = this.peer;
      opening.catch((e) => {
        if (this.peer === peer) this.abortRTC(e);
      });
      return;
    }
    if (this.config.input === "websocket" && !this.socket) {
      const url = new URL("/input", location.href);
      url.protocol = "wss:";
      url.port = this.config.port;
      this.socket = new WebSocket(url);
      this.pending = null;
      this.ready = new Promise((resolve, reject) => {
        this.resolveReady = resolve;
        this.rejectReady = reject;
      });
      // The page reports connection failures; avoid an unhandled early rejection.
      this.ready.catch((e) => this.fail(e));
      this.socket.onmessage = (event) => {
        const value = JSON.parse(event.data);
        this.updateTime(value.server_s);
        this.lastServerInputMs = value.server_timing?.input_ms ?? null;
        if (value.type === "ready") {
          this.resolveReady();
          return;
        }
        if (this.pending) {
          const item = this.pending;
          this.pending = null;
          clearTimeout(item.timer);
          value.error ? item.reject(Error(value.error)) : item.resolve(value);
        }
      };
      const closed = () => {
        const e = Error("控制连接中断，请退出 VR 并刷新；若证书未信任，先打开控制端口地址");
        this.rejectReady(e);
        if (this.pending) {
          clearTimeout(this.pending.timer);
          this.pending.reject(e);
          this.pending = null;
        }
        this.fail(e);
      };
      this.socket.onclose = closed;
      this.socket.onerror = closed;
    }
  }
  async openRTC() {
    this.inflight = new Map();
    this.rtcReady = false;
    this.route = "WebRTC 连接中";
    const pc = (this.peer = new RTCPeerConnection({ iceServers: [] }));
    const dc = (this.channel = pc.createDataChannel("robot-input", { ordered: true }));
    this.rtcOpenTimer = setTimeout(
      () => this.abortRTC(Error("WebRTC 连接超时，请退出 VR 并刷新")),
      10000,
    );
    dc.onmessage = (event) => {
      try {
        const value = JSON.parse(event.data);
        this.updateTime(value.server_s);
        if (value.type === "ready") {
          clearTimeout(this.rtcOpenTimer);
          this.rtcReady = true;
          this.drain();
          return;
        }
        if (value.type === "error") throw Error(value.error);
        const item = this.inflight.get(value.seq);
        if (!item) throw Error("WebRTC 回复序号无效");
        clearTimeout(item.timer);
        this.inflight.delete(value.seq);
        this.lastAckSeq = value.seq;
        this.lastRttMs = performance.now() - item.started;
        this.lastServerInputMs = value.server_timing?.input_ms ?? null;
      } catch (e) {
        this.abortRTC(e);
      }
    };
    dc.onclose = () => this.abortRTC(Error("WebRTC 控制连接中断，请退出 VR 并刷新"));
    dc.onerror = () => this.abortRTC(Error("WebRTC 数据通道错误"));
    pc.onconnectionstatechange = () => {
      if (pc.connectionState === "failed") this.abortRTC(Error("WebRTC 无法建立直连"));
      if (pc.connectionState === "connected")
        this.inspectRoute().catch(() => {
          this.route = "WebRTC（路径未知）";
        });
    };
    await pc.setLocalDescription(await pc.createOffer());
    if (pc.iceGatheringState !== "complete")
      await new Promise((resolve) => {
        pc.onicegatheringstatechange = () => {
          if (pc.iceGatheringState === "complete") resolve();
        };
      });
    if (this.failed || this.peer !== pc) return;
    const response = await fetch("/api/webrtc/offer", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(pc.localDescription),
      signal: AbortSignal.timeout(10000),
    });
    const answer = await response.json();
    if (!response.ok) throw Error(answer.error || response.status);
    if (!this.failed && this.peer === pc) await pc.setRemoteDescription(answer);
  }
  async inspectRoute() {
    const stats = await this.peer.getStats();
    let pair;
    for (const value of stats.values()) {
      if (value.type === "transport" && value.selectedCandidatePairId)
        pair = stats.get(value.selectedCandidatePairId);
      if (
        !pair &&
        value.type === "candidate-pair" &&
        value.state === "succeeded" &&
        value.nominated
      )
        pair = value;
    }
    const local = pair && stats.get(pair.localCandidateId),
      remote = pair && stats.get(pair.remoteCandidateId);
    const protocol = local?.protocol || remote?.protocol;
    this.route = protocol ? "WebRTC / " + protocol.toUpperCase() : "WebRTC（路径未知）";
    fetch("/api/client-event", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        stage: "webrtc_route",
        session_id: this.sessionId,
        protocol: protocol || null,
        local_type: local?.candidateType,
        remote_type: remote?.candidateType,
      }),
      signal: AbortSignal.timeout(1500),
    }).catch(() => {});
  }
  abortRTC(error) {
    if (this.failed) return;
    this.errors++;
    this.fail(error);
    if (!this.stopSent) {
      this.stopSent = true;
      this.post("/api/stop", { reason: "webrtc_transport_error" }).catch(() => {});
    }
  }
  drainRTC() {
    if (this.failed || !this.rtcReady) return;
    try {
      while (this.edges.length || this.latest) {
        if (this.inflight.size >= 32) throw Error("WebRTC 未回复包数量达到上限，请暂停后检查网络");
        if (this.channel.bufferedAmount > 65536)
          throw Error("WebRTC 发送缓冲积压，请暂停后检查网络");
        const p = this.edges.length ? this.edges.shift() : this.latest;
        if (p === this.latest) this.latest = null;
        p.seq = this.seq++;
        p.last_server_s = this.serverTime;
        p.transport = {
          skipped: this.skipped,
          errors: this.errors,
          previous_rtt_ms: this.lastRttMs,
          previous_server_input_ms: this.lastServerInputMs ?? null,
          input: "webrtc",
          ...this.debugState(),
        };
        const started = performance.now(),
          timer = setTimeout(() => this.abortRTC(Error("WebRTC 控制回复超时")), 1000);
        this.inflight.set(p.seq, { started, timer });
        this.channel.send(JSON.stringify(p));
      }
    } catch (e) {
      this.abortRTC(e);
    }
  }
  async socketPost(body) {
    if (this.socket.readyState !== WebSocket.OPEN) await this.ready;
    if (this.socket.readyState !== WebSocket.OPEN) throw Error("控制连接未就绪");
    if (this.pending) throw Error("控制请求重叠");
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending = null;
        this.socket.close();
        reject(Error("控制响应超时"));
      }, 1000);
      this.pending = { resolve, reject, timer };
      this.socket.send(JSON.stringify(body));
    });
  }
  async post(path, body) {
    const started = performance.now();
    if (path === "/api/input" && this.config?.input === "websocket") {
      const value = await this.socketPost(body);
      this.lastRttMs = performance.now() - started;
      return value;
    }
    const r = await fetch(path, {
      method: "POST",
      priority: "high",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(1000),
    });
    const value = await r.json();
    this.updateTime(value.server_s);
    if (path === "/api/input") this.lastRttMs = performance.now() - started;
    if (!r.ok) throw Error(value.error || r.status);
    return value;
  }
  updateTime(value) {
    if (Number.isFinite(value)) this.serverTime = Math.max(this.serverTime, value);
  }
  submit(packet, edge = false) {
    if (this.failed) return;
    this.sessionId = packet.session_id;
    if (edge) {
      if (this.edges.length >= 32) {
        this.onError(Error("按键队列已满，请暂停后重试"));
        return;
      }
      this.edges.push(packet);
      this.latest = null;
    } else {
      if (this.latest) this.skipped++;
      this.latest = packet;
    }
    this.drain();
  }
  async drain() {
    if (this.config?.input === "webrtc") {
      this.drainRTC();
      return;
    }
    if (this.sending) return;
    this.sending = true;
    try {
      while (this.edges.length || this.latest) {
        const p = this.edges.length ? this.edges.shift() : this.latest;
        if (p === this.latest) this.latest = null;
        p.seq = this.seq++;
        p.last_server_s = this.serverTime;
        p.transport = {
          skipped: this.skipped,
          errors: this.errors,
          previous_rtt_ms: this.lastRttMs,
          previous_server_input_ms: this.lastServerInputMs ?? null,
        };
        await this.post("/api/input", p);
      }
    } catch (e) {
      this.errors++;
      this.fail(e);
      if (this.socket) this.socket.close();
      if (!this.stopSent) {
        this.stopSent = true;
        try {
          await this.post("/api/stop", { reason: "transport_error" });
        } catch (_) {}
      }
    } finally {
      this.sending = false;
    }
  }
  clear() {
    this.edges = [];
    this.latest = null;
  }
}
