import { InputTransport } from "./xr-transport.js";
import { TeleopScene, instruction } from "./xr-scene.js";

const $ = (id) => document.getElementById(id),
  roles = ["left_wrist", "scene_fixed", "right_wrist"];
$("preview").checked = new URLSearchParams(location.search).get("preview") !== "off";
const sid = crypto.randomUUID(),
  sourceIds = new WeakMap(),
  images = {};
let nextSource = 0,
  xr = null,
  ref = null,
  scene = null,
  status = {},
  epoch = 0,
  clientFrame = 0;
let lastSent = -Infinity,
  lastSignature = null,
  stopped = false,
  videoSerial = 0;
let visibleSince = null,
  everVisible = false;
let trackingPauseSent = false;
let lastFrameWorkMs = 0;
const mediaStats = {};
const transport = new InputTransport((e) => {
  $("error").textContent = String(e) + "；控制已停止，请退出 VR 后刷新。";
  fetch("/api/client-event", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      stage: "control_transport",
      session_id: sid,
      error: String(e),
      previous_rtt_ms: transport.lastRttMs,
      transport: transport.failureDetails,
    }),
    signal: AbortSignal.timeout(1500),
  }).catch(() => {});
});

async function pause(reason) {
  transport.suspend();
  try {
    await transport.post("/api/stop", { reason });
  } catch (e) {
    $("error").textContent = String(e);
  }
}
function pose(p) {
  if (!p) return null;
  const t = p.transform;
  return {
    position_m: [t.position.x, t.position.y, t.position.z],
    orientation_xyzw: [t.orientation.x, t.orientation.y, t.orientation.z, t.orientation.w],
    emulated_position: !!p.emulatedPosition,
  };
}
function sample(frame, time) {
  return {
    schema_version: "webxr-input/1",
    session_id: sid,
    seq: 0,
    event: "frame",
    epoch,
    reference_space: "local-floor",
    client_time_ms: performance.now(),
    client_unix_ms: Date.now(),
    xr_frame_time_ms: time,
    client_frame_seq: clientFrame++,
    previous_frame_work_ms: lastFrameWorkMs,
    client_media: { enabled: $("preview").checked, streams: structuredClone(mediaStats) },
    visibility: xr.visibilityState,
    head: pose(frame.getViewerPose(ref)),
    inputs: Array.from(xr.inputSources, (s) => {
      if (!sourceIds.has(s)) sourceIds.set(s, ++nextSource);
      return {
        source_id: sourceIds.get(s),
        handedness: s.handedness,
        profiles: Array.from(s.profiles),
        grip: s.gripSpace ? pose(frame.getPose(s.gripSpace, ref)) : null,
        target_ray: pose(frame.getPose(s.targetRaySpace, ref)),
        gamepad: s.gamepad
          ? {
              buttons: Array.from(s.gamepad.buttons, (b) => ({
                value: b.value,
                pressed: b.pressed,
                touched: b.touched,
              })),
              axes: Array.from(s.gamepad.axes),
            }
          : null,
      };
    }),
  };
}
function signature(packet) {
  const cfg = status.profile?.buttons || {
    primary: { hand: "right", index: 4 },
    discard: { hand: "left", index: 4 },
    clutch: 1,
  };
  return packet.inputs
    .map((s) => {
      const buttons = s.gamepad?.buttons || [];
      return (
        s.handedness +
        ":" +
        [
          cfg.clutch,
          ...["primary", "discard"]
            .filter((k) => cfg[k].hand === s.handedness)
            .map((k) => cfg[k].index),
        ]
          .map((i) => ((buttons[i]?.value || 0) >= 0.7 ? "1" : "0"))
          .join("")
      );
    })
    .join("|");
}
function frame(time, frame) {
  if (!xr || !frame) return;
  const began = performance.now();
  try {
    const viewer = frame.getViewerPose(ref);
    if (viewer && !scene.anchored) scene.anchor(viewer);
    if (xr.visibilityState === "visible" && viewer) {
      trackingPauseSent = false;
      if (visibleSince === null) visibleSince = time;
      if (time - visibleSince >= 250) everVisible = true;
    } else {
      visibleSince = null;
      if (everVisible && !trackingPauseSent) {
        trackingPauseSent = true;
        pause("tracking_or_visibility_lost");
      }
      return;
    }
    if (everVisible) {
      transport.start();
      const p = sample(frame, time),
        sig = signature(p),
        edge = lastSignature !== null && sig !== lastSignature;
      lastSignature = sig;
      if (edge || time - lastSent >= 33) {
        lastSent = time;
        transport.submit(p, edge);
      }
      if (edge)
        $("input").textContent = JSON.stringify(
          p.inputs.map((s) => ({
            hand: s.handedness,
            buttons: s.gamepad?.buttons.map((b) => b.value),
          })),
          null,
          2,
        );
    }
    // Submit control before texture uploads / rendering; media must not age
    // this frame's input while it is already available to the application.
    scene.update(status, time);
    scene.render();
  } catch (e) {
    $("error").textContent = String(e);
    pause("webxr_frame_error");
  } finally {
    lastFrameWorkMs = performance.now() - began;
  }
}

for (const role of roles) {
  mediaStats[role] = { fetch_ms: null, decode_ms: null, bytes: 0, errors: 0 };
  images[role] = { bitmap: null, updated: 0, serial: 0 };
  (async () => {
    while (!stopped) {
      images[role].disabled = !$("preview").checked;
      if (images[role].disabled) {
        await new Promise((resolve) => setTimeout(resolve, 250));
        continue;
      }
      const began = performance.now();
      try {
        const r = await fetch("/camera/" + role + ".jpg?t=" + Date.now(), {
          priority: "low",
          signal: AbortSignal.timeout(1200),
        });
        if (!r.ok) throw Error("相机不可用");
        const blob = await r.blob();
        const received = performance.now();
        mediaStats[role].fetch_ms = received - began;
        mediaStats[role].bytes = blob.size;
        mediaStats[role].server_frame_age_ms = Number(r.headers.get("X-Frame-Age-Ms"));
        mediaStats[role].preview_seq = r.headers.get("X-Frame-Sequence");
        mediaStats[role].capture_seq = r.headers.get("X-Capture-Sequence");
        const bitmap = await createImageBitmap(blob, { imageOrientation: "flipY" }),
          old = images[role].bitmap;
        mediaStats[role].decode_ms = performance.now() - received;
        images[role] = { bitmap, updated: performance.now(), serial: ++videoSerial };
        // Textures are updated on the next XR frame before old bitmaps are retired.
        if (old) setTimeout(() => old.close(), 1000);
        const url = URL.createObjectURL(blob),
          previous = $(role).dataset.url;
        $(role).src = url;
        $(role).dataset.url = url;
        if (previous) URL.revokeObjectURL(previous);
      } catch (_) {
        mediaStats[role].errors++;
      }
      const media = status.profile?.webxr_transport || {};
      const target = status.cameras?.[role]?.pipeline?.preview?.fps || 8;
      const fps =
        transport.lastRttMs > (media.preview_congestion_rtt_ms || 100)
          ? Math.min(target, media.preview_congestion_fps || 2)
          : target;
      await new Promise((resolve) =>
        setTimeout(resolve, Math.max(0, 1000 / fps - (performance.now() - began))),
      );
    }
  })();
}

$("start").onclick = async () => {
  $("start").disabled = true;
  $("error").textContent = "";
  try {
    if (!scene) scene = new TeleopScene($("xr"), images);
    scene.configure(status.profile.vr_layout);
    scene.anchored = false;
    visibleSince = null;
    everVisible = false;
    trackingPauseSent = false;
    xr = await navigator.xr.requestSession("immersive-vr", { requiredFeatures: ["local-floor"] });
    await scene.renderer.xr.setSession(xr);
    ref = scene.renderer.xr.getReferenceSpace();
    ref.addEventListener("reset", () => {
      epoch++;
      pause("reference_reset");
    });
    xr.addEventListener("visibilitychange", () => {
      if (everVisible && xr.visibilityState !== "visible" && !trackingPauseSent) {
        trackingPauseSent = true;
        pause("vr_visibility_" + xr.visibilityState);
      }
    });
    xr.addEventListener("end", () => {
      pause("vr_session_end");
      epoch++;
      xr = null;
      ref = null;
      scene.renderer.setAnimationLoop(null);
      $("start").disabled = false;
    });
    lastSignature = null;
    lastSent = -Infinity;
    scene.renderer.setAnimationLoop(frame);
  } catch (e) {
    $("error").textContent = String(e);
    if (xr) await xr.end();
    xr = null;
    $("start").disabled = false;
  }
};
$("pause").onclick = () => pause("operator_pause");
$("end").onclick = async () => {
  await pause("operator_exit");
  if (xr) await xr.end();
};

async function refreshStatus() {
  try {
    const r = await fetch("/api/status", { priority: "low", signal: AbortSignal.timeout(1200) });
    status = await r.json();
    transport.updateTime(status.server_s);
    transport.configure(status.profile?.webxr_transport);
    if (status.profile?.webxr_transport?.input === "websocket") {
      const check = new URL("/", location.href);
      check.port = status.profile.webxr_transport.port;
      $("control-link").href = check.href;
      $("control-check").hidden = false;
    }
    $("state").textContent = instruction(status.session?.state, status.session?.resume_state);
    $("instruction").textContent = instruction(status.session?.state, status.session?.resume_state);
    $("trigger-help").textContent = ["left", "right"]
      .map(
        (s) =>
          (s === "left" ? "左" : "右") +
          "扳机：" +
          (status.profile?.arms?.[s]?.gripper?.trigger_pressed === "open"
            ? "按下打开、松开关闭。"
            : "按下关闭、松开打开。"),
      )
      .join("");
    status.client = {
      rtt_ms: transport.lastRttMs,
      failed: transport.failed,
      preview_enabled: $("preview").checked,
      route:
        transport.route ||
        (status.profile?.webxr_transport?.input === "websocket" ? "WSS / TCP" : "HTTPS / TCP"),
    };
    $("status").textContent = JSON.stringify(
      {
        session: status.session,
        arms: status.arms,
        errors: status.errors,
        client: status.client,
        media: mediaStats,
        transport: { skipped: transport.skipped, errors: transport.errors },
      },
      null,
      2,
    );
    for (const role of roles)
      $(role + "_fps").textContent = (status.cameras?.[role]?.fps || 0).toFixed(1) + " FPS";
  } catch (e) {
    $("error").textContent = String(e);
  }
  if (!stopped) setTimeout(refreshStatus, 300);
}
window.addEventListener("pagehide", () => {
  stopped = true;
  transport.suspend();
  navigator.sendBeacon(
    "/api/stop",
    new Blob(['{"reason":"pagehide"}'], { type: "application/json" }),
  );
  if (scene) scene.dispose();
  for (const x of Object.values(images)) x.bitmap?.close();
});
(async () => {
  await refreshStatus();
  try {
    $("start").disabled = !(
      isSecureContext &&
      navigator.xr &&
      (await navigator.xr.isSessionSupported("immersive-vr"))
    );
  } catch (e) {
    $("error").textContent = String(e);
  }
})();
