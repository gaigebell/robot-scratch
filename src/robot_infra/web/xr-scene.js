import * as THREE from "./vendor/three.module.js";

const stateText = {
  SELF_CHECK: "设备自检中",
  STANDBY: "待机 · A 回到 Home",
  HOMING: "正在回位",
  READY: "准备就绪 · A 开始",
  PREPARING: "正在准备录制",
  TELEOP: "遥操中 · 侧握跟随 / A 结束",
  FINALIZING: "正在写盘与检查数据",
  REVIEW: "待审阅 · A 保存 / X 丢弃",
  COMMITTING: "正在提交采集结果",
  PAUSED: "已暂停 · A 恢复",
  FAULT: "设备或会话异常",
  CLOSED: "会话已结束",
};
export function instruction(s, resume) {
  if (s === "DIAGNOSTIC") return "相机与连接诊断 · 不驱动机械臂";
  if (s === "PAUSED")
    return resume === "STANDBY"
      ? "已暂停 · A 回到 Home"
      : resume === "READY"
        ? "已暂停 · A 开始遥操"
        : "已暂停 · A 恢复遥操";
  return stateText[s] || "等待设备状态";
}

export class TeleopScene {
  constructor(canvas, images) {
    this.images = images;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.xr.enabled = true;
    this.renderer.xr.setReferenceSpaceType("local-floor");
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#07121e");
    this.camera = new THREE.PerspectiveCamera();
    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.textures = {};
    this.panels = {};
    this.lastUploaded = {};
    this.anchored = false;
    this.lastHUD = 0;
    for (const role of ["left_wrist", "scene_fixed", "right_wrist"]) {
      const texture = new THREE.Texture();
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.flipY = false;
      this.textures[role] = texture;
      const mesh = new THREE.Mesh(
        new THREE.PlaneGeometry(1, 1),
        new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide }),
      );
      this.root.add(mesh);
      this.panels[role] = mesh;
    }
    this.hudCanvases = {};
    for (const label of ["top", "bottom"]) {
      const canvas = document.createElement("canvas");
      canvas.width = 2048;
      canvas.height = 448;
      this.hudCanvases[label] = canvas;
      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      this.textures[label] = texture;
      const mesh = new THREE.Mesh(
        new THREE.PlaneGeometry(1, 1),
        new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide }),
      );
      this.root.add(mesh);
      this.panels[label] = mesh;
    }
  }
  configure(layout) {
    const cfg = {
      distance_m: 2.21,
      main_width_m: 5.4,
      side_width_m: 1.4,
      side_x_m: 3.8,
      hud_y_m: 3.05,
      ...layout,
    };
    const main = this.panels.scene_fixed;
    main.position.set(0, 0, -cfg.distance_m);
    main.scale.set(cfg.main_width_m, cfg.main_width_m * 0.75, 1);
    for (const [side, sign] of [
      ["left_wrist", -1],
      ["right_wrist", 1],
    ]) {
      const panel = this.panels[side];
      panel.position.set(sign * cfg.side_x_m, 0, -cfg.distance_m);
      panel.rotation.y = -sign * Math.atan2(cfg.side_x_m, cfg.distance_m);
      panel.scale.set(cfg.side_width_m, cfg.side_width_m * 0.75, 1);
    }
    for (const [name, sign] of [
      ["top", 1],
      ["bottom", -1],
    ]) {
      this.panels[name].position.set(0, sign * cfg.hud_y_m, -cfg.distance_m);
      this.panels[name].scale.set(4.8, 1.05, 1);
      this.panels[name].rotation.x =
        sign *
        (cfg.hud_face_viewer
          ? Math.atan2(Math.abs(cfg.hud_y_m), cfg.distance_m)
          : THREE.MathUtils.degToRad(cfg.hud_tilt_deg || 0));
    }
  }
  anchor(viewer) {
    const p = viewer.transform.position,
      q = viewer.transform.orientation;
    const rotation = new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(q.x, q.y, q.z, q.w),
      "YXZ",
    );
    this.root.position.set(p.x, p.y, p.z);
    this.root.rotation.set(0, rotation.y, 0);
    this.anchored = true;
  }
  update(status, time) {
    for (const role of ["left_wrist", "scene_fixed", "right_wrist"]) {
      const item = this.images[role];
      if (item?.bitmap && this.lastUploaded[role] !== item.serial) {
        this.textures[role].image = item.bitmap;
        this.textures[role].needsUpdate = true;
        this.lastUploaded[role] = item.serial;
      }
      this.panels[role].material.color.setHex(
        item?.disabled
          ? 0x555555
          : item && performance.now() - item.updated < 1500
            ? 0xffffff
            : 0x63342e,
      );
    }
    if (time - this.lastHUD < 250) return;
    this.lastHUD = time;
    const s = status.session || {};
    const draw = (name, lines) => {
      const canvas = this.hudCanvases[name],
        c = canvas.getContext("2d");
      c.fillStyle = "#122b3c";
      c.fillRect(0, 0, canvas.width, canvas.height);
      c.fillStyle = "#a5edcf";
      c.font = "bold 60px sans-serif";
      c.fillText(lines[0], 64, 88);
      c.fillStyle = "#dfedf1";
      c.font = "42px sans-serif";
      lines.slice(1).forEach((line, i) => c.fillText(line, 64, 168 + i * 76));
      this.textures[name].needsUpdate = true;
    };
    const jawAction =
      status.profile?.arms?.right?.gripper?.trigger_pressed === "open"
        ? "扳机按下打开 / 松开关闭"
        : "扳机按下关闭 / 松开打开";
    const ep = s.episode;
    const motion = ["left", "right"]
      .map((side) => {
        const m = s.last_motion?.[side],
          l = status.arms?.[side]?.nearest_limit;
        return `${side === "left" ? "左" : "右"} ${m ? `${(m.position_error_m * 1000).toFixed(0)}mm/${THREE.MathUtils.radToDeg(m.orientation_error_rad).toFixed(0)}°` : "待跟随"} ${l ? `${l.joint}余${THREE.MathUtils.radToDeg(l.remaining_rad).toFixed(0)}°` : ""}`;
      })
      .join("  |  ");
    draw("top", [
      instruction(s.state, s.resume_state),
      s.state === "DIAGNOSTIC"
        ? "仅显示画面与连接统计；按键不控制机械臂"
        : `侧握：同侧跟随    ${jawAction}`,
      ep
        ? `任务 ${ep.task}  |  episode ${ep.episode}  |  已保存 ${ep.saved_episodes}  丢弃 ${ep.discarded_episodes}`
        : `已完成遥操 ${s.completed_trials || 0} 次   |   episode 采集未启用`,
      s.home
        ? `回位 ${s.home.elapsed_s.toFixed(1)}s · ` +
          Object.entries(s.home.error_rad)
            .map(
              ([side, error]) =>
                `${side === "left" ? "左" : "右"}误差 ${THREE.MathUtils.radToDeg(error).toFixed(1)}°`,
            )
            .join(" / ")
        : s.state === "FAULT"
          ? s.reason
          : ep?.qc
            ? `QC ${ep.qc.passed ? "通过" : "需检查"} · ${ep.qc.issues.join("；") || "A 保存 / X 丢弃"}`
            : "退出 VR 自动暂停；录制中断后保留数据供审阅",
      motion,
    ]);
    const link = status.client || {};
    const storage = ep?.image_storage === "mp4" ? "MP4" : ep ? "原始 MCAP" : "未录制";
    const lines = [
      `${link.route || "控制连接"} · ${storage} · RTT ${(link.rtt_ms || 0).toFixed(0)}ms${link.failed ? " · 控制断开，请退出刷新" : link.preview_enabled === false ? " · 预览已关闭" : ""}`,
    ];
    for (const [role, title] of [
      ["scene_fixed", "主视角"],
      ["left_wrist", "左腕"],
      ["right_wrist", "右腕"],
    ]) {
      const camera = status.cameras?.[role],
        p = camera?.profile || {};
      const v = camera?.pipeline?.preview,
        rec = camera?.pipeline?.record?.rgb;
      lines.push(
        `${title} ${p.width || "—"}×${p.height || "—"} / ${p.fps || "—"}  实收 ${(camera?.fps || 0).toFixed(1)} FPS · VR ${v?.width || "—"}×${v?.height || "—"} · 录制 ${rec?.width || "—"}×${rec?.height || "—"}`,
      );
    }
    draw("bottom", lines);
  }
  render() {
    this.renderer.render(this.scene, this.camera);
  }
  dispose() {
    this.renderer.setAnimationLoop(null);
    for (const mesh of Object.values(this.panels)) {
      mesh.geometry.dispose();
      mesh.material.dispose();
    }
    for (const texture of Object.values(this.textures)) texture.dispose();
    this.renderer.dispose();
  }
}
