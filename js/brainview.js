// Brain view: draws a robot's neural network live while it runs, and explains
// every part of it in plain words.
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
  const sgn = v => (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(2);
  const deg = r => Math.round(r * 180 / Math.PI);

  // What each input is, in the order the brain sees them (see gatherInputs in sim.js).
  RW.inputLabels = function (robot) {
    const L = [
      { t: 'bias (always 1)', g: 'bias', k: 'bias' },
      { t: 'clock sin', g: 'clock', k: 'clock', speed: 1 }, { t: 'clock cos', g: 'clock', k: 'clock', speed: 1 },
      { t: 'clock 2× sin', g: 'clock', k: 'clock', speed: 2 }, { t: 'clock 2× cos', g: 'clock', k: 'clock', speed: 2 },
    ];
    robot.servos.forEach((s, i) => L.push({ t: `S${i + 1} angle`, g: 'joint', k: 'joint', servo: i }));
    robot.accels.forEach((a, i) => {
      const n = robot.accels.length > 1 ? ` ${i + 1}` : '';
      L.push({ t: `accel${n} x`, g: 'accel', k: 'accel', axis: 'x' }, { t: `accel${n} y`, g: 'accel', k: 'accel', axis: 'y' });
    });
    if (robot.camera) for (let i = 0; i < RW.CAM_RAYS; i++) {
      L.push({ t: `camera ${deg(RW.camRayAngle(i))}°`, g: 'cam', k: 'cam', ray: i });
    }
    return L;
  };

  // Lay the network out in a w x h box.
  function layout(brain, w, h) {
    const nIn = brain.nIn, nH = RW.HIDDEN, nOut = brain.nOut;
    const top = 30, bottom = 10, span = h - top - bottom;
    const xIn = Math.min(150, w * 0.3), xOut = Math.max(xIn + 120, w - Math.min(250, w * 0.4));
    const xHid = (xIn + xOut) / 2;
    const col = (n, i) => top + span * (i + 0.5) / n;
    return {
      xIn, xHid, xOut, rowIn: span / nIn,
      inY: Array.from({ length: nIn }, (_, i) => col(nIn, i)),
      hidY: Array.from({ length: nH }, (_, i) => col(nH, i)),
      outY: Array.from({ length: nOut }, (_, i) => col(nOut, i)),
      inR: Math.max(2.5, Math.min(6, span / nIn * 0.35)),
    };
  }

  // Weight lookups (genome layout: [freq, hidden weights, output bias + weights, stiffness]).
  function weights(b) {
    const g = b.g, nIn = b.nIn, nH = RW.HIDDEN;
    const oBase = o => 1 + nH * nIn + o * (nH + 1);
    return {
      wIH: (j, i) => g[1 + j * nIn + i],      // input i -> hidden j
      wHO: (o, j) => g[oBase(o) + 1 + j],      // hidden j -> output o
      bO: o => g[oBase(o)],                    // output o's own bias
    };
  }

  // Which node is under (x, y): {kind: 'in'|'hid'|'out', i} or null.
  RW.brainViewNodeAt = function (run, w, h, x, y) {
    if (!run) return null;
    const b = run.brain, L = layout(b, w, h);
    if (x >= L.xOut - 14) {
      let best = null, bd = 16;
      L.outY.forEach((oy, o) => { const d = Math.abs(y - oy); if (d < bd) { bd = d; best = o; } });
      return best == null ? null : { kind: 'out', i: best };
    }
    let hb = null, hd = 16;
    L.hidY.forEach((hy, j) => { const d = Math.hypot(x - L.xHid, y - hy); if (d < hd) { hd = d; hb = j; } });
    if (hb != null) return { kind: 'hid', i: hb };
    if (x <= L.xIn + L.inR + 6) {
      const i = Math.floor((y - L.inY[0] + L.rowIn / 2) / L.rowIn);
      if (i >= 0 && i < b.nIn) return { kind: 'in', i };
    }
    return null;
  };

  // Where a node is drawn, for placing its explanation next to it.
  RW.brainViewNodePos = function (run, w, h, node) {
    const L = layout(run.brain, w, h);
    if (node.kind === 'in') return { x: L.xIn, y: L.inY[node.i] };
    if (node.kind === 'hid') return { x: L.xHid, y: L.hidY[node.i] };
    return { x: L.xOut, y: L.outY[node.i] };
  };

  // How strongly each connection and node should show, given what is focused.
  function emphasis(W, focus) {
    if (!focus) return { ih: () => 1, ho: () => 1, inNode: () => 1, hidNode: () => 1, outNode: () => 1 };
    if (focus.kind === 'out') {
      const imp = j => 0.06 + 0.94 * clamp1(Math.abs(W.wHO(focus.i, j)) / 1.5);
      return { ih: j => imp(j), ho: o => (o === focus.i ? 1 : 0.08), inNode: () => 1, hidNode: imp, outNode: o => (o === focus.i ? 1 : 0.35) };
    }
    if (focus.kind === 'hid') {
      return { ih: j => (j === focus.i ? 1 : 0.06), ho: (o, j) => (j === focus.i ? 1 : 0.08), inNode: () => 1, hidNode: j => (j === focus.i ? 1 : 0.3), outNode: () => 1 };
    }
    const imp = j => 0.06 + 0.94 * clamp1(Math.abs(W.wIH(j, focus.i)) / 2);
    return { ih: (j, i) => (i === focus.i ? 1 : 0.05), ho: (o, j) => imp(j), inNode: i => (i === focus.i ? 1 : 0.35), hidNode: imp, outNode: () => 1 };
  }

  // opts: {focus, locked, guide}. guide is which part the step-by-step guide is
  // pointing at: 'in' | 'edges' | 'hid' | 'out' | null.
  RW.drawBrainView = function (ctx, w, h, run, opts) {
    opts = opts || {};
    ctx.fillStyle = '#0e1219'; ctx.fillRect(0, 0, w, h);
    if (!run) {
      ctx.fillStyle = '#94a0b8'; ctx.font = '13px system-ui'; ctx.textAlign = 'center';
      ctx.fillText('Train your robot first, then its brain shows up here', w / 2, h / 2);
      return;
    }
    const b = run.brain, robot = run.robot, x = b.inBuf, nIn = b.nIn, nH = RW.HIDDEN, nOut = b.nOut;
    const L = layout(b, w, h), labels = RW.inputLabels(robot), W = weights(b);
    const f0 = opts.focus;
    const focus = f0 && ((f0.kind === 'in' && f0.i < nIn) || (f0.kind === 'hid' && f0.i < nH) || (f0.kind === 'out' && f0.i < nOut)) ? f0 : null;
    const E = emphasis(W, focus);
    const gd = opts.guide;
    const part = p => (!gd || gd === p ? 1 : 0.18); // dim the parts the guide isn't talking about

    // input -> hidden connections
    for (let j = 0; j < nH; j++) {
      for (let i = 0; i < nIn; i++) {
        const wgt = W.wIH(j, i), flow = wgt * x[i];
        const a = (0.03 + 0.75 * clamp1(Math.abs(flow) / 1.5)) * E.ih(j, i) * part('edges');
        ctx.strokeStyle = signed(flow || wgt, a);
        ctx.lineWidth = 0.4 + 1.6 * clamp1(Math.abs(wgt) / 2);
        ctx.beginPath(); ctx.moveTo(L.xIn, L.inY[i]); ctx.lineTo(L.xHid, L.hidY[j]); ctx.stroke();
      }
    }
    // hidden -> output connections
    for (let o = 0; o < nOut; o++) {
      for (let j = 0; j < nH; j++) {
        const wgt = W.wHO(o, j), flow = wgt * b.h[j];
        const a = (0.05 + 0.85 * clamp1(Math.abs(flow) / 1.2)) * E.ho(o, j) * part('edges');
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
      ctx.globalAlpha = E.inNode(i) * part('in');
      ctx.fillStyle = signed(v, 0.25 + 0.75 * clamp1(Math.abs(v)));
      ctx.beginPath(); ctx.arc(L.xIn, y, L.inR, 0, 7); ctx.fill();
      if (focus && focus.kind === 'in' && focus.i === i) { ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke(); }
      ctx.textAlign = 'right';
      ctx.fillStyle = GROUP_COLOR[lab.g];
      ctx.fillText(lab.t, L.xIn - L.inR - 42, y);
      ctx.fillStyle = '#c9d1e0';
      ctx.fillText((v >= 0 ? '+' : '') + v.toFixed(2), L.xIn - L.inR - 3, y);
    }
    ctx.globalAlpha = 1;
    // hidden nodes, numbered H1..H8
    ctx.font = 'bold 9px system-ui'; ctx.textAlign = 'center';
    for (let j = 0; j < nH; j++) {
      const v = b.h[j], y = L.hidY[j];
      ctx.globalAlpha = E.hidNode(j) * part('hid');
      ctx.fillStyle = '#1b2130'; ctx.beginPath(); ctx.arc(L.xHid, y, 11, 0, 7); ctx.fill();
      ctx.fillStyle = signed(v, 0.2 + 0.8 * Math.abs(v)); ctx.beginPath(); ctx.arc(L.xHid, y, 11, 0, 7); ctx.fill();
      const f = focus && focus.kind === 'hid' && focus.i === j;
      ctx.strokeStyle = f ? '#ffffff' : '#33405a'; ctx.lineWidth = f ? 2.5 : 1; ctx.stroke();
      ctx.fillStyle = Math.abs(v) > 0.5 ? '#10131a' : '#e8ecf4'; ctx.fillText(`H${j + 1}`, L.xHid, y + 0.5);
    }
    ctx.globalAlpha = 1;
    // outputs: what each servo is asked for, and where it really is
    ctx.font = '11px system-ui';
    for (let o = 0; o < nOut; o++) {
      const s = robot.servos[o], y = L.outY[o], v = b.out[o];
      ctx.globalAlpha = E.outNode(o) * part('out');
      ctx.fillStyle = signed(v, 0.3 + 0.7 * Math.abs(v)); ctx.beginPath(); ctx.arc(L.xOut, y, 9, 0, 7); ctx.fill();
      const f = focus && focus.kind === 'out' && focus.i === o;
      ctx.strokeStyle = f ? '#ffffff' : '#7a4d00'; ctx.lineWidth = f ? 2.5 : 1.5; ctx.stroke();
      ctx.textAlign = 'left'; ctx.fillStyle = '#e8ecf4';
      const ang = s.joint.getJointAngle();
      const gx = L.xOut + 16, gw = Math.max(40, w - gx - 10);
      if (s.continuous) {
        ctx.fillText(`S${o + 1} ↻ spin ${(v * RW.SERVO_MAX_SPEED).toFixed(1)} rad/s · ${(ang / (2 * Math.PI)).toFixed(1)} turns`, gx, y - 6);
      } else {
        ctx.fillText(`S${o + 1} wants ${deg(s.target ?? 0)}° · at ${deg(ang)}° · stiff ${Math.round(b.gain[o])}`, gx, y - 6);
        // gauge across the servo's range: orange = target, white = actual
        const toX = r => gx + gw * (0.5 + 0.5 * Math.max(-1, Math.min(1, r / s.range)));
        ctx.fillStyle = '#232b3d'; ctx.fillRect(gx, y + 3, gw, 5);
        ctx.fillStyle = '#33405a'; ctx.fillRect(gx + gw / 2 - 0.5, y + 1, 1, 9);
        ctx.fillStyle = '#ffb020'; ctx.fillRect(toX(s.target ?? 0) - 1.5, y + 1, 3, 9);
        ctx.fillStyle = '#ffffff'; ctx.fillRect(toX(ang) - 1, y + 2, 2, 7);
      }
    }
    ctx.globalAlpha = 1;
    // title
    ctx.textBaseline = 'alphabetic'; ctx.textAlign = 'left'; ctx.font = '12px system-ui'; ctx.fillStyle = '#94a0b8';
    const name = n => (n.kind === 'in' ? (labels[n.i] || {}).t : n.kind === 'hid' ? `H${n.i + 1}` : `S${n.i + 1}`);
    ctx.fillText(`Brain · rhythm ${b.freq.toFixed(2)} Hz · ${nIn} inputs → ${nH} hidden → ${nOut} servos · ` +
      (focus == null ? 'hover anything to see what it means; click to keep it highlighted'
        : `showing ${name(focus)}` + (opts.locked ? ': click it again to let go' : ': click to keep it highlighted')), 10, 18);
    ctx.textAlign = 'right';
    const down = '■ pushing down', downW = ctx.measureText(down).width;
    ctx.fillStyle = rgba(NEG, 1); ctx.fillText(down, w - 8, 18);
    ctx.fillStyle = rgba(POS, 1); ctx.fillText('■ pushing up', w - 20 - downW, 18);
  };

  // ---------- explanations ----------

  // The few biggest pushes on a unit right now: [{name, v}] sorted by size.
  function topPushes(list, n) {
    return list.filter(p => Math.abs(p.v) > 0.02).sort((a, b) => Math.abs(b.v) - Math.abs(a.v)).slice(0, n);
  }
  const pushList = ps => ps.length ? ps.map(p => `<b>${p.name}</b> ${sgn(p.v)}`).join(', ') : 'nothing much';

  // Plain-words explanation of one node, with its live value: {title, html}.
  RW.explainBrainNode = function (run, node) {
    const b = run.brain, robot = run.robot, x = b.inBuf, W = weights(b), nH = RW.HIDDEN;
    const labels = RW.inputLabels(robot);
    if (node.kind === 'in') {
      const lab = labels[node.i], v = x[node.i];
      const listeners = topPushes(Array.from({ length: nH }, (_, j) => ({ name: `H${j + 1}`, v: W.wIH(j, node.i) })), 3);
      const listen = `<p class="tip-dim">Listened to most by: ${listeners.map(p => `<b>${p.name}</b> (weight ${sgn(p.v)})`).join(', ') || 'nobody much'}.</p>`;
      if (lab.k === 'bias') return { title: 'Bias: always 1', html:
        `<p>A constant. It gives every hidden unit a starting lean, so it can be "on" or "off" by default even when all the sensors read zero, a bit like the resting position of a see-saw.</p>${listen}` };
      if (lab.k === 'clock') {
        const beat = lab.speed === 1 ? `${b.freq.toFixed(2)} times a second` : `${(2 * b.freq).toFixed(2)} times a second (double speed)`;
        return { title: `${lab.t}: the robot's own rhythm`, html:
          `<p>A steady wave the brain makes for itself, like a metronome. It goes round ${beat}; evolution chose the speed. Sine and cosine are the same wave a quarter of a beat apart, so together they say exactly where in the beat the robot is.</p>` +
          `<p>Now: <b>${sgn(v)}</b>. Watch it swing smoothly between −1 and +1.</p>${listen}` };
      }
      if (lab.k === 'joint') {
        const s = robot.servos[lab.servo];
        const what = s.continuous ? 'the sine of how far it has turned (it can spin all the way round)' : `a fraction of its range: ${sgn(v)} means ${deg(s.joint.getJointAngle())}° of its ±${deg(s.range)}°`;
        return { title: `${lab.t}: where servo S${lab.servo + 1} is`, html:
          `<p>The brain feels where its own joint is, like you knowing where your leg is with your eyes shut. This is ${what}.</p>` +
          `<p>It is how the brain can tell a leg has finished its swing, or got stuck against the ground.</p>${listen}` };
      }
      if (lab.k === 'accel') {
        const along = lab.axis === 'x' ? 'along the stick it is fixed to' : 'across that stick';
        const meaning = lab.axis === 'y'
          ? (v > 0.8 ? 'mostly gravity: the stick is fairly level' : v > 0.3 ? 'gravity, but the stick is tilted a fair way' : v > -0.3 ? 'very little: the stick is nearly on its end, or falling' : 'upside down, or being thrown about')
          : (Math.abs(v) < 0.2 ? 'almost nothing: level and steady' : `${v > 0 ? 'towards the front' : 'towards the back'}: tilted ${v > 0 ? 'nose up' : 'nose down'}, or being jolted`);
        return { title: `${lab.t}: what the accelerometer feels`, html:
          `<p>The pull felt ${along}, measured in g (1 g = the pull of gravity). Standing still and level it reads x 0, y +1: just gravity pulling down. Tilting shifts gravity from y into x; bumps and lurches add sudden spikes.</p>` +
          `<p>Now: <b>${sgn(v)} g</b>, which is ${meaning}.</p>${listen}` };
      }
      if (lab.k === 'cam') {
        const a = deg(RW.camRayAngle(lab.ray));
        const dir = a > 0 ? `${a}° above straight ahead` : a < 0 ? `${-a}° below straight ahead` : 'straight ahead';
        const dist = v > 0.001 ? `something ${((1 - v) * RW.CAM_RANGE).toFixed(1)} m away` : `nothing within ${RW.CAM_RANGE} m`;
        return { title: `${lab.t}: one ray of the camera`, html:
          `<p>The time-of-flight camera measures how far it is to the nearest thing along this ray, pointing ${dir} from the head. 0 means nothing within ${RW.CAM_RANGE} m; closer things give bigger numbers, up to 1 right at the lens.</p>` +
          `<p>Now: <b>${sgn(v)}</b>, so it sees ${dist}. The lower rays see the ground; a hurdle shows up as the upper rays suddenly getting bigger.</p>${listen}` };
      }
    }
    if (node.kind === 'hid') {
      const j = node.i, v = b.h[j];
      const pushes = topPushes(Array.from({ length: b.nIn }, (_, i) => ({ name: labels[i].t, v: W.wIH(j, i) * x[i] })), 3);
      const drives = topPushes(Array.from({ length: b.nOut }, (_, o) => ({ name: `S${o + 1}`, v: W.wHO(o, j) })), 3);
      return { title: `H${j + 1}: a hidden unit`, html:
        `<p>It takes every input, multiplies each by the weight on its line, adds them all up, then squashes the total to between −1 and +1. Think of it as a little detector that evolution has tuned to respond to some mix of the inputs.</p>` +
        `<p>Now: <b>${sgn(v)}</b>${Math.abs(v) > 0.9 ? ' (fully on)' : Math.abs(v) < 0.2 ? ' (quiet)' : ''}.</p>` +
        `<p>Biggest pushes right now (input × weight): ${pushList(pushes)}.</p>` +
        `<p class="tip-dim">It mostly drives: ${drives.map(p => `<b>${p.name}</b> (weight ${sgn(p.v)})`).join(', ') || 'nothing much'}.</p>` };
    }
    const o = node.i, s = robot.servos[o], v = b.out[o];
    const pushes = topPushes([{ name: 'its own bias', v: W.bO(o) }].concat(Array.from({ length: nH }, (_, j) => ({ name: `H${j + 1}`, v: W.wHO(o, j) * b.h[j] }))), 3);
    const what = s.continuous
      ? `<p>S${o + 1} spins all the way round, so the output is a speed: ${sgn(v)} × ${RW.SERVO_MAX_SPEED} = ${(v * RW.SERVO_MAX_SPEED).toFixed(1)} radians a second${v < 0 ? ' backwards' : ''}.</p>`
      : `<p>The output (−1 to +1) becomes a target angle: ${sgn(v)} × ${deg(s.range)}° = <b>${deg(s.target ?? 0)}°</b>. The servo pushes towards it with its learned stiffness (${Math.round(b.gain[o])}): high is snappy, low is springy. It is at ${deg(s.joint.getJointAngle())}° now; any gap is the servo still catching up, or the ground pushing back.</p>`;
    return { title: `S${o + 1}: what the brain tells this servo`, html:
      `<p>It adds up the hidden units, each times the weight on its line, plus a bias of its own, then squashes the total to between −1 and +1.</p>${what}` +
      `<p>Biggest pushes right now: ${pushList(pushes)}.</p>` };
  };

  // The step-by-step guide.
  RW.BRAIN_GUIDE = [
    { part: null, title: 'This is the robot’s whole program', html:
      '<p>Sixty times a second, numbers flow from left to right: what the robot senses goes in, passes through 8 hidden units, and comes out as instructions for each servo.</p><p>There is no list of moves stored anywhere. The walk comes from these numbers reacting to what is happening right now.</p>' },
    { part: 'in', title: '1. Inputs: what the robot knows', html:
      '<p>On the left. A <b style="color:#8ecbff">clock</b> it makes for itself (its sense of rhythm), where each <b style="color:#ffb020">joint</b> is, what the <b style="color:#3ad29f">accelerometer</b> feels (which way is down, and bumps), and what the <b style="color:#b46cff">camera</b> sees ahead.</p><p>Each number is the reading right now. Hover one to see what it means.</p>' },
    { part: 'edges', title: '2. Weights: the lines', html:
      '<p>Every line is a <b>weight</b>: one number that evolution picked. A signal travelling along a line gets multiplied by it. Thick lines are big weights.</p><p>The colour shows what the signal is doing right now: <b style="color:#ffb020">orange pushes the next unit up</b>, <b style="color:#4dabf7">blue pushes it down</b>, and brighter means stronger. Watch them flicker as the robot moves.</p>' },
    { part: 'hid', title: '3. Hidden units: little detectors', html:
      '<p>Each of H1 to H8 adds up everything coming into it and squashes the total to between −1 and +1. Some end up responding to the beat of the clock, some to tilting, some to a leg being in a certain place.</p><p>Nobody decided what each one does; evolution shaped them. Hover one to see what is driving it right now.</p>' },
    { part: 'out', title: '4. Outputs: orders for the servos', html:
      '<p>On the right, one per servo. Each adds up the hidden units the same way, and its value becomes a target angle. The gauge shows the target (orange) and where the joint really is (white).</p><p>Click a servo on the robot above to see only the lines that feed it.</p>' },
    { part: null, title: '5. So where did the numbers come from?', html:
      '<p>Nobody wrote them. Training started with completely random weights. Each generation, the robots that got furthest (keeping their heads up) were copied with small random changes, and the rest were dropped.</p><p>That’s evolution: no one tells the brain how to walk, it just keeps whatever works. <b>Try it:</b> press pause, then hover the clock inputs and find which servos they drive most.</p>' },
  ];
})(window.RW = window.RW || {});
