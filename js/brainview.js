// Brain view: draws a robot's neural network live while it runs.
//
// Left: the inputs (clock, joint angles, accelerometer, camera) with their
// current values. Middle: the hidden units. Right: one output per servo, with
// the target it is asking for and where the joint really is.
// Each connection's colour is the sign of the signal flowing along it right
// now (orange = pushing up, blue = pushing down) and its brightness is how
// much signal it carries; its thickness is the size of the weight.
(function (RW) {
  'use strict';
  const POS = [255, 176, 32], NEG = [77, 171, 247];
  const GROUP_COLOR = { bias: '#94a0b8', clock: '#8ecbff', joint: '#ffb020', accel: '#3ad29f', cam: '#b46cff' };

  function rgba(c, a) { return `rgba(${c[0]},${c[1]},${c[2]},${a})`; }
  function signed(v, a) { return rgba(v >= 0 ? POS : NEG, a); }
  const clamp1 = v => Math.max(0, Math.min(1, v));

  // What each input is, in the order the brain sees them (see gatherInputs in sim.js).
  RW.inputLabels = function (robot) {
    const L = [
      { t: 'bias (always 1)', g: 'bias' },
      { t: 'clock sin', g: 'clock' }, { t: 'clock cos', g: 'clock' },
      { t: 'clock 2× sin', g: 'clock' }, { t: 'clock 2× cos', g: 'clock' },
    ];
    robot.servos.forEach((s, i) => L.push({ t: `S${i + 1} angle`, g: 'joint' }));
    robot.accels.forEach((a, i) => {
      const n = robot.accels.length > 1 ? ` ${i + 1}` : '';
      L.push({ t: `accel${n} x`, g: 'accel' }, { t: `accel${n} y`, g: 'accel' });
    });
    if (robot.camera) for (let i = 0; i < RW.CAM_RAYS; i++) {
      L.push({ t: `camera ${Math.round(RW.camRayAngle(i) * 180 / Math.PI)}°`, g: 'cam' });
    }
    return L;
  };

  // Lay the network out in a w x h box; returns node positions (also used for hover).
  function layout(brain, w, h) {
    const nIn = brain.nIn, nH = RW.HIDDEN, nOut = brain.nOut;
    const top = 30, bottom = 10, span = h - top - bottom;
    const xIn = Math.min(150, w * 0.3), xOut = Math.max(xIn + 120, w - Math.min(250, w * 0.4));
    const xHid = (xIn + xOut) / 2;
    const col = (n, i) => top + span * (i + 0.5) / n;
    return {
      xIn, xHid, xOut,
      inY: Array.from({ length: nIn }, (_, i) => col(nIn, i)),
      hidY: Array.from({ length: nH }, (_, i) => col(nH, i)),
      outY: Array.from({ length: nOut }, (_, i) => col(nOut, i)),
      inR: Math.max(2.5, Math.min(6, span / nIn * 0.35)),
    };
  }

  // Which servo output is under (x, y), for hover-to-trace.
  RW.brainViewHit = function (brain, w, h, x, y) {
    if (!brain) return null;
    const L = layout(brain, w, h);
    let best = null, bd = 16;
    L.outY.forEach((oy, o) => {
      const d = x >= L.xOut - 12 ? Math.abs(y - oy) : Infinity;
      if (d < bd) { bd = d; best = o; }
    });
    return best;
  };

  RW.drawBrainView = function (ctx, w, h, run, focus, locked) {
    ctx.fillStyle = '#0e1219'; ctx.fillRect(0, 0, w, h);
    if (!run) {
      ctx.fillStyle = '#94a0b8'; ctx.font = '13px system-ui'; ctx.textAlign = 'center';
      ctx.fillText('Train your robot first, then its brain shows up here', w / 2, h / 2);
      return;
    }
    const b = run.brain, robot = run.robot, g = b.g, x = b.inBuf, nIn = b.nIn, nH = RW.HIDDEN, nOut = b.nOut;
    const L = layout(b, w, h), labels = RW.inputLabels(robot);
    const wIH = (j, i) => g[1 + j * nIn + i];                    // input i -> hidden j
    const oBase = o => 1 + nH * nIn + o * (nH + 1);              // output o: bias, then hidden weights
    const wHO = (o, j) => g[oBase(o) + 1 + j];

    // How much each hidden unit matters to the focused servo (for tracing).
    const hidImportance = Array.from({ length: nH }, (_, j) => focus == null ? 1 : clamp1(Math.abs(wHO(focus, j)) / 1.5));

    // input -> hidden connections
    for (let j = 0; j < nH; j++) {
      for (let i = 0; i < nIn; i++) {
        const wgt = wIH(j, i), flow = wgt * x[i];
        let a = 0.03 + 0.75 * clamp1(Math.abs(flow) / 1.5);
        a *= focus == null ? 1 : 0.06 + 0.94 * hidImportance[j];
        ctx.strokeStyle = signed(flow || wgt, a);
        ctx.lineWidth = 0.4 + 1.6 * clamp1(Math.abs(wgt) / 2);
        ctx.beginPath(); ctx.moveTo(L.xIn, L.inY[i]); ctx.lineTo(L.xHid, L.hidY[j]); ctx.stroke();
      }
    }
    // hidden -> output connections
    for (let o = 0; o < nOut; o++) {
      for (let j = 0; j < nH; j++) {
        const wgt = wHO(o, j), flow = wgt * b.h[j];
        let a = 0.05 + 0.85 * clamp1(Math.abs(flow) / 1.2);
        if (focus != null && focus !== o) a *= 0.08;
        ctx.strokeStyle = signed(flow || wgt, a);
        ctx.lineWidth = 0.6 + 2.4 * clamp1(Math.abs(wgt) / 2);
        ctx.beginPath(); ctx.moveTo(L.xHid, L.hidY[j]); ctx.lineTo(L.xOut, L.outY[o]); ctx.stroke();
      }
    }

    // input nodes and labels
    ctx.font = `${Math.max(8, Math.min(11, L.inR * 2.2))}px system-ui`;
    ctx.textBaseline = 'middle';
    for (let i = 0; i < nIn; i++) {
      const v = x[i], y = L.inY[i], lab = labels[i] || { t: `input ${i}`, g: 'bias' };
      ctx.fillStyle = signed(v, 0.25 + 0.75 * clamp1(Math.abs(v)));
      ctx.beginPath(); ctx.arc(L.xIn, y, L.inR, 0, 7); ctx.fill();
      ctx.textAlign = 'right';
      ctx.fillStyle = GROUP_COLOR[lab.g];
      ctx.fillText(lab.t, L.xIn - L.inR - 42, y);
      ctx.fillStyle = '#c9d1e0';
      ctx.fillText((v >= 0 ? '+' : '') + v.toFixed(2), L.xIn - L.inR - 3, y);
    }
    // hidden nodes
    for (let j = 0; j < nH; j++) {
      const v = b.h[j], y = L.hidY[j];
      ctx.globalAlpha = focus == null ? 1 : 0.25 + 0.75 * hidImportance[j];
      ctx.fillStyle = '#1b2130'; ctx.beginPath(); ctx.arc(L.xHid, y, 10, 0, 7); ctx.fill();
      ctx.fillStyle = signed(v, 0.2 + 0.8 * Math.abs(v)); ctx.beginPath(); ctx.arc(L.xHid, y, 10, 0, 7); ctx.fill();
      ctx.strokeStyle = '#33405a'; ctx.lineWidth = 1; ctx.stroke();
      ctx.globalAlpha = 1;
    }
    // outputs: what each servo is asked for, and where it really is
    ctx.font = '11px system-ui';
    for (let o = 0; o < nOut; o++) {
      const s = robot.servos[o], y = L.outY[o], v = b.out[o], dim = focus != null && focus !== o;
      ctx.globalAlpha = dim ? 0.35 : 1;
      ctx.fillStyle = signed(v, 0.3 + 0.7 * Math.abs(v)); ctx.beginPath(); ctx.arc(L.xOut, y, 9, 0, 7); ctx.fill();
      ctx.strokeStyle = focus === o ? '#ffffff' : '#7a4d00'; ctx.lineWidth = focus === o ? 2.5 : 1.5; ctx.stroke();
      ctx.textAlign = 'left'; ctx.fillStyle = '#e8ecf4';
      const ang = s.joint.getJointAngle();
      const gx = L.xOut + 16, gw = Math.max(40, w - gx - 10);
      if (s.continuous) {
        ctx.fillText(`S${o + 1} ↻ spin ${(v * RW.SERVO_MAX_SPEED).toFixed(1)} rad/s · ${(ang / (2 * Math.PI)).toFixed(1)} turns`, gx, y - 6);
      } else {
        const deg = r => Math.round(r * 180 / Math.PI);
        ctx.fillText(`S${o + 1} wants ${deg(s.target ?? 0)}° · at ${deg(ang)}° · stiff ${Math.round(b.gain[o])}`, gx, y - 6);
        // gauge across the servo's range: orange = target, white = actual
        const toX = r => gx + gw * (0.5 + 0.5 * Math.max(-1, Math.min(1, r / s.range)));
        ctx.fillStyle = '#232b3d'; ctx.fillRect(gx, y + 3, gw, 5);
        ctx.fillStyle = '#33405a'; ctx.fillRect(gx + gw / 2 - 0.5, y + 1, 1, 9);
        ctx.fillStyle = '#ffb020'; ctx.fillRect(toX(s.target ?? 0) - 1.5, y + 1, 3, 9);
        ctx.fillStyle = '#ffffff'; ctx.fillRect(toX(ang) - 1, y + 2, 2, 7);
      }
      ctx.globalAlpha = 1;
    }
    // title
    ctx.textBaseline = 'alphabetic'; ctx.textAlign = 'left'; ctx.font = '12px system-ui'; ctx.fillStyle = '#94a0b8';
    ctx.fillText(`Brain · rhythm ${b.freq.toFixed(2)} Hz · ${nIn} inputs → ${nH} hidden → ${nOut} servos · ` +
      (focus == null ? 'hover a servo (or click one on the robot) to trace it' : `tracing S${focus + 1}` + (locked ? ': click it again to stop' : ': click to keep it traced')), 10, 18);
    ctx.textAlign = 'right';
    const down = '■ pushing down', downW = ctx.measureText(down).width;
    ctx.fillStyle = rgba(NEG, 1); ctx.fillText(down, w - 8, 18);
    ctx.fillStyle = rgba(POS, 1); ctx.fillText('■ pushing up', w - 20 - downW, 18);
  };
})(window.RW = window.RW || {});
