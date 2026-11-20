// The records the headset web page sends: the same QuestXarmPoseV1 records the headset app writes
// (scripts/quest_pose_v1.py decodes them; tests/test_web_source.py runs this file with node and checks every record).
// No browser objects here: the page passes plain values, so this file also runs under node.
(function (root) {
  "use strict";
  const BUILD = "webxr-1";
  const STATE = { visible: "FOCUSED", "visible-blurred": "VISIBLE", hidden: "SYNCHRONIZED" };
  const IN_L = ["trigger", "squeeze", "thumbstick", "thumbstick_click", "thumbstick_touch", "trigger_touch", "thumbrest_touch", "x", "y", "menu"];
  const IN_R = ["trigger", "squeeze", "thumbstick", "thumbstick_click", "thumbstick_touch", "trigger_touch", "thumbrest_touch", "a", "b"];

  const fin = (v) => typeof v === "number" && isFinite(v);
  const rd = (v, n) => Number(v.toFixed(n));
  const ns = (ms) => Math.max(0, Math.round(ms * 1e6));

  // pose: null (not located) or {p: [x, y, z], q: [x, y, z, w], emulated: bool}; active: undefined for the head
  function poseObj(pose, active) {
    const o = { queried: true, result: 0 };
    if (active !== undefined) o.active = !!active;
    let p = null, q = null;
    if (pose && active !== false && pose.p.every(fin) && pose.q.every(fin)) {
      const n = Math.hypot(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
      if (n > 1e-6) {
        p = pose.p.map((v) => rd(v, 6));
        q = pose.q.map((v) => rd(v / n, 7));
      }
    }
    const ok = p !== null;
    const posTracked = ok && !pose.emulated; // an emulated position is a guess, not a tracked one
    o.flags = ok ? 1 | 2 | 4 | (posTracked ? 8 : 0) : 0;
    o.pos_valid = ok; o.ori_valid = ok; o.pos_tracked = posTracked; o.ori_tracked = ok;
    o.p = p; o.q = q;
    return o;
  }

  // gamepad: null (no controller) or {buttons: [{pressed, touched, value}], axes: [...]} in WebXR's xr-standard order:
  // buttons 0 trigger, 1 squeeze, 3 thumbstick, 4 A or X, 5 B or Y, 6 thumb rest; axes 2, 3 the thumbstick (up is -1).
  function inputObj(gamepad, left) {
    const names = left ? IN_L : IN_R, o = {};
    if (!gamepad || !gamepad.buttons || gamepad.buttons.length < 2) {
      for (const k of names) o[k] = null;
      return o;
    }
    const b = (i) => gamepad.buttons[i] || { pressed: false, touched: false, value: 0 };
    const clamp = (v) => (fin(v) ? Math.min(1, Math.max(-1, v)) : 0);
    const ax = gamepad.axes || [];
    o.trigger = rd(clamp(b(0).value), 4);
    o.squeeze = rd(clamp(b(1).value), 4);
    o.thumbstick = [rd(clamp(ax[2]), 4), rd(-clamp(ax[3]), 4) + 0]; // the app reports up as +1
    o.thumbstick_click = !!b(3).pressed;
    o.thumbstick_touch = !!b(3).touched;
    o.trigger_touch = !!b(0).touched;
    o.thumbrest_touch = !!b(6).touched;
    if (left) { o.x = !!b(4).pressed; o.y = !!b(5).pressed; o.menu = false; } // WebXR does not report the menu button
    else { o.a = !!b(4).pressed; o.b = !!b(5).pressed; }
    return o;
  }

  // One stream per page load (sid). Pose and event records share one counter, as in the app.
  function makeStream(sid) {
    let seq = 0, epoch = 0, changeNs = 0;
    const base = (type) => ({ v: 1, type: type, build: BUILD, sid: sid, seq: ++seq });
    return {
      get seq() { return seq; },
      event(name, tMs, visibility) {
        const o = base("event");
        o.event = name; o.space = "LOCAL"; o.local_epoch = epoch;
        if (visibility !== undefined) o.session_state = STATE[visibility] || "UNKNOWN";
        o.time_ns = ns(tMs || 0);
        return JSON.stringify(o);
      },
      // the reference space moved (a recentre): poses after it are not continuous with those before
      spaceChanged(tMs) {
        epoch += 1; changeNs = ns(tMs);
        return this.event("local_space_change_pending", tMs);
      },
      // f: {tMs, visibility, head: pose|null, left: null | {pose, gamepad}, right: null | {pose, gamepad}}
      pose(f) {
        const o = base("pose");
        o.space = "LOCAL"; o.t_query_ns = ns(f.tMs); o.t_query_kind = "predicted_display_time";
        o.session_state = STATE[f.visibility] || "UNKNOWN"; o.focused = f.visibility === "visible";
        o.local_epoch = epoch; o.local_change_time_ns = changeNs;
        o.head = poseObj(f.head, undefined);
        o.grip_l = poseObj(f.left && f.left.pose, !!f.left);
        o.grip_r = poseObj(f.right && f.right.pose, !!f.right);
        o.in_l = inputObj(f.left && f.left.gamepad, true);
        o.in_r = inputObj(f.right && f.right.gamepad, false);
        return JSON.stringify(o);
      },
    };
  }

  const api = { BUILD: BUILD, makeStream: makeStream, poseObj: poseObj, inputObj: inputObj };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.QuestRecords = api;
})(typeof self !== "undefined" ? self : this);
