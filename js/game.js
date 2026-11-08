// UI: the robot editor, the training loop, the replay, and all the drawing.
(function () {
  'use strict';
  const RW = window.RW;
  const $ = s => document.querySelector(s);
  const $$ = s => [...document.querySelectorAll(s)];
  const canvas = $('#view'), ctx = canvas.getContext('2d');
  const chart = $('#chart'), cctx = chart.getContext('2d');
  const SAVE_KEY = 'robot-walker.v2';

  const COLORS = {
    sky1: '#bfe3ff', sky2: '#eef7ff', ground: '#6b8f3a', ground2: '#557530', dirt: '#8a6a43',
    grid: 'rgba(30,60,100,.08)', grid2: 'rgba(30,60,100,.16)',
    stick: '#2b3954', servo: '#ffb020', hinge: '#ffffff', weld: '#5b6b88', accel: '#3ad29f',
    hover: '#ff4f9a', ghost: '#2b3954', lead: '#d9480f', head: '#e8eef8', fell: '#d6336c',
    cam: '#b46cff', rock: '#9aa5b8', rockEdge: '#5d6880', zone: 'rgba(58,210,159,.12)',
  };

  const S = {
    mode: 'build', tool: 'stick', ctool: 'block',
    course: { obstacles: [] },
    design: null, nextId: 1,
    opts: { popSize: 40, timeLimit: 15, strength: 1, rules: { fall: true, level: 2 } },
    trainer: null, running: false, speed: 4, ghosts: true,
    bestEver: null, history: [],
    watch: null, wspeed: 1, wlen: 'train',
    cam: { x: 1, y: 0.9, s: 120 },
    drag: null, hover: null, mouse: null,
  };

  // ---------- persistence ----------
  function save() {
    try {
      localStorage.setItem(SAVE_KEY, JSON.stringify({
        design: S.design, opts: S.opts, course: S.course,
        bestEver: S.bestEver && { ...S.bestEver, genome: Array.from(S.bestEver.genome) },
        history: S.history,
      }));
    } catch (e) { /* storage unavailable: fine */ }
  }
  function load() {
    try {
      const d = JSON.parse(localStorage.getItem(SAVE_KEY) || 'null');
      if (!d || !d.design) return false;
      S.design = d.design; S.opts = { ...S.opts, ...d.opts, rules: { ...S.opts.rules, ...(d.opts && d.opts.rules) } };
      if (d.bestEver) S.bestEver = { ...d.bestEver, genome: Float32Array.from(d.bestEver.genome) };
      S.history = d.history || [];
      // a brain saved by an older version may not fit the robot's brain any more
      if (S.bestEver && S.bestEver.genome.length !== RW.genomeLength(RW.analyse(S.design))) { S.bestEver = null; S.history = []; }
      if (d.course && Array.isArray(d.course.obstacles)) S.course = d.course;
      return true;
    } catch (e) { return false; }
  }

  function setDesign(design) {
    S.design = design;
    S.nextId = 1 + Math.max(0, ...design.points.map(p => p.id), ...design.sticks.map(s => s.id));
    fitBuildCamera();
    forget();
  }
  // Servos that the shape of the robot holds rigid (see RW.findLockedServos).
  let lockedKey = null;
  function updateLocked() {
    const key = JSON.stringify(S.design);
    if (key === lockedKey) return;
    lockedKey = key;
    const before = S.locked ? S.locked.length : 0;
    try { S.locked = RW.findLockedServos(S.design); } catch (e) { S.locked = []; }
    S.newLock = S.locked.length > before;
    if (S.newLock) toast('A servo can’t move: the sticks around it are braced into a rigid triangle. It’s marked in red.');
  }
  function forget() {
    S.trainer = null; S.running = false; S.bestEver = null; S.history = []; S.watch = null;
    save(); updateLocked(); updateUI();
  }
  // Any change to the robot means the old brain no longer fits it.
  function designChanged() {
    const had = S.bestEver || (S.trainer && S.trainer.generation > 0);
    forget();
    if (had && !S.newLock) toast('Robot changed, so its brain starts again from scratch');
  }

  // ---------- canvas / camera ----------
  let W = 0, H = 0, DPR = 1;
  function resize() {
    const r = canvas.getBoundingClientRect();
    DPR = window.devicePixelRatio || 1;
    W = r.width; H = r.height;
    canvas.width = Math.round(W * DPR); canvas.height = Math.round(H * DPR);
  }
  window.addEventListener('resize', () => { resize(); fitBuildCamera(); });
  const toScreen = (x, y) => [(x - S.cam.x) * S.cam.s + W / 2, H / 2 - (y - S.cam.y) * S.cam.s];
  const toWorld = (sx, sy) => [(sx - W / 2) / S.cam.s + S.cam.x, (H / 2 - sy) / S.cam.s + S.cam.y];

  // The build view stays put while you edit; it only refits when a robot is
  // loaded, the window resizes, or a part ends up off screen.
  function fitBuildCamera() { S.buildCam = buildCamera(); }
  function refitIfOffscreen() {
    for (const p of S.design.points) {
      const [x, y] = toScreen(p.x, p.y);
      if (x < 20 || x > W - 20 || y < 20) { fitBuildCamera(); return; }
    }
  }
  function buildCamera() {
    // Fit the robot (or a default area) with some room around it.
    const pts = S.design.points;
    let minX = 0, maxX = 2, maxY = 1.4;
    for (const p of pts) { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y); }
    const wspan = Math.max(3, maxX - minX + 2), hspan = Math.max(2, maxY + 0.8);
    const s = Math.min(W / wspan, (H - 40) / hspan, 220);
    return { x: (minX + maxX) / 2, y: hspan / 2 - 0.25, s };
  }

  // ---------- editor ----------
  const SNAP = 0.05;
  const snap = v => Math.round(v / SNAP) * SNAP;
  const pointById = id => S.design.points.find(p => p.id === id);
  function stickCount(id) { return S.design.sticks.filter(s => s.a === id || s.b === id).length; }

  function hitPoint(sx, sy) {
    let best = null, bd = 14;
    for (const p of S.design.points) {
      const [px, py] = toScreen(p.x, p.y);
      const d = Math.hypot(px - sx, py - sy);
      if (d < bd) { bd = d; best = p; }
    }
    return best;
  }
  function hitStick(sx, sy) {
    let best = null, bd = 9;
    for (const s of S.design.sticks) {
      const a = pointById(s.a), b = pointById(s.b);
      const [ax, ay] = toScreen(a.x, a.y), [bx, by] = toScreen(b.x, b.y);
      const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy || 1;
      const t = Math.max(0, Math.min(1, ((sx - ax) * dx + (sy - ay) * dy) / L2));
      const d = Math.hypot(ax + t * dx - sx, ay + t * dy - sy);
      if (d < bd) { bd = d; best = s; }
    }
    return best;
  }
  function hoverTarget(sx, sy) {
    const t = S.tool;
    if (t === 'stick' || t === 'move' || t === 'servo' || t === 'weld' || t === 'head') { const p = hitPoint(sx, sy); return p && { point: p }; }
    if (t === 'accel') { const s = hitStick(sx, sy); return s && { stick: s }; }
    if (t === 'erase') { const p = hitPoint(sx, sy); if (p) return { point: p }; const s = hitStick(sx, sy); return s && { stick: s }; }
    return null;
  }

  function evPos(e) { const r = canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }

  canvas.addEventListener('pointerdown', e => {
    if (S.mode === 'course') return courseDown(e);
    if (S.mode !== 'build') return;
    const [sx, sy] = evPos(e);
    const [wx, wy] = toWorld(sx, sy);
    const t = S.tool;
    canvas.setPointerCapture(e.pointerId);
    if (t === 'stick') {
      const p = hitPoint(sx, sy);
      S.drag = { kind: 'stick', from: p ? { id: p.id, x: p.x, y: p.y } : { id: null, x: snap(wx), y: Math.max(0, snap(wy)) } };
    } else if (t === 'move') {
      const p = hitPoint(sx, sy);
      if (p) S.drag = { kind: 'move', p, moved: false };
    } else if (t === 'servo' || t === 'weld') {
      const p = hitPoint(sx, sy);
      if (!p) return;
      if (stickCount(p.id) < 2) { toast(`A ${t === 'servo' ? 'servo' : 'weld'} goes where two or more sticks meet`); return; }
      if (t === 'servo' && p.kind === 'servo') {
        // clicking a servo again steps through its ranges, then removes it
        const i = RW.SERVO_RANGES.indexOf(RW.servoRangeDeg(p));
        if (i >= 0 && i < RW.SERVO_RANGES.length - 1) p.range = RW.SERVO_RANGES[i + 1];
        else { p.kind = 'free'; delete p.range; }
        toast(p.kind === 'servo' ? `Servo: ${rangeLabel(p)}` : 'Servo removed');
      } else {
        p.kind = p.kind === t ? 'free' : t;
        delete p.range;
      }
      designChanged();
    } else if (t === 'camera') {
      if (!pointById(S.design.head)) { toast('Give the robot a head first: the camera goes on the head'); return; }
      S.design.camera = !S.design.camera;
      designChanged();
    } else if (t === 'head') {
      const p = hitPoint(sx, sy);
      if (!p) return;
      S.design.head = S.design.head === p.id ? null : p.id;
      if (!S.design.head) S.design.camera = false;
      designChanged();
    } else if (t === 'accel') {
      const s = hitStick(sx, sy);
      if (!s) return;
      s.accel = !s.accel;
      designChanged();
    } else if (t === 'erase') {
      const p = hitPoint(sx, sy);
      if (p) {
        S.design.sticks = S.design.sticks.filter(s => s.a !== p.id && s.b !== p.id);
        S.design.points = S.design.points.filter(q => q.id !== p.id);
      } else {
        const s = hitStick(sx, sy);
        if (!s) return;
        S.design.sticks = S.design.sticks.filter(q => q !== s);
      }
      // drop points no stick uses; a joint with <2 sticks can't be a servo/weld
      S.design.points = S.design.points.filter(q => stickCount(q.id) > 0);
      for (const q of S.design.points) if (stickCount(q.id) < 2) q.kind = 'free';
      if (!pointById(S.design.head)) { S.design.head = null; S.design.camera = false; }
      designChanged();
    }
  });

  canvas.addEventListener('pointermove', e => {
    const [sx, sy] = evPos(e);
    S.mouse = [sx, sy];
    if (S.mode === 'course') return courseMove(e);
    if (S.mode !== 'build') return;
    const [wx, wy] = toWorld(sx, sy);
    if (S.drag && S.drag.kind === 'move') {
      S.drag.p.x = snap(wx); S.drag.p.y = Math.max(0, snap(wy)); S.drag.moved = true;
    }
    S.hover = hoverTarget(sx, sy);
    canvas.style.cursor = S.hover || S.drag ? 'pointer' : (S.tool === 'stick' ? 'crosshair' : 'default');
  });

  canvas.addEventListener('pointercancel', () => { S.drag = null; });
  canvas.addEventListener('pointerup', e => {
    if (S.mode === 'course') return courseUp(e);
    if (S.mode !== 'build' || !S.drag) return;
    const d = S.drag; S.drag = null;
    setTimeout(refitIfOffscreen, 0);
    if (d.kind === 'move') {
      if (d.moved) { mergeAt(d.p); designChanged(); }
      return;
    }
    if (d.kind === 'stick') {
      const [sx, sy] = evPos(e);
      const [wx, wy] = toWorld(sx, sy);
      const hp = hitPoint(sx, sy);
      const to = hp ? { id: hp.id, x: hp.x, y: hp.y } : { id: null, x: snap(wx), y: Math.max(0, snap(wy)) };
      if (Math.hypot(to.x - d.from.x, to.y - d.from.y) < 0.1) return;
      if (d.from.id && d.from.id === to.id) return;
      const ensure = q => {
        if (q.id) return q.id;
        const p = { id: S.nextId++, x: q.x, y: q.y, kind: 'free' };
        S.design.points.push(p);
        return p.id;
      };
      const a = ensure(d.from), b = ensure(to);
      if (S.design.sticks.some(s => (s.a === a && s.b === b) || (s.a === b && s.b === a))) return;
      if (S.design.sticks.length >= 40) { toast('That is a lot of sticks! 40 is the limit.'); return; }
      S.design.sticks.push({ id: S.nextId++, a, b, accel: false });
      designChanged();
    }
  });

  const rangeLabel = p => RW.isContinuous(p) ? '↻ spins all the way round' : `±${RW.servoRangeDeg(p)}°`;

  // ---------- course editor ----------
  function clampCourseX(x) {
    const half = W / 2 / S.cam.s, lo = -1 + half, hi = RW.COURSE_END + 1 - half;
    return lo > hi ? (lo + hi) / 2 : Math.max(lo, Math.min(hi, x == null ? lo : x));
  }
  function scrollCourse(x) {
    S.courseX = clampCourseX(x);
    const r = $('#cscroll');
    r.value = String(Math.round(1000 * (S.courseX + 1) / (RW.COURSE_END + 2)));
  }
  canvas.addEventListener('wheel', e => {
    if (S.mode !== 'course') return;
    e.preventDefault();
    scrollCourse(clampCourseX(S.courseX) + (e.deltaX || e.deltaY) / S.cam.s);
  }, { passive: false });
  const clampH = h => Math.max(0.05, Math.min(2, h));
  function hitObstacle(wx, wy) {
    for (let i = S.course.obstacles.length - 1; i >= 0; i--) {
      const o = S.course.obstacles[i];
      if (wx >= o.x - 0.05 && wx <= o.x + o.w + 0.05 && wy >= -0.05 && wy <= o.h + 0.1) return o;
    }
    return null;
  }
  // The obstacle a drag from (x0,y0) to (x1,y1) would make.
  function dragObstacle(d, x1, y1) {
    const type = S.ctool;
    const a = Math.min(d.x0, x1), b = Math.max(d.x0, x1);
    const o = { type, x: a, w: b - a, h: clampH(Math.max(y1, type === 'ramp' ? 0 : d.y0)), dir: x1 >= d.x0 ? 1 : -1 };
    return o;
  }
  function courseDown(e) {
    const [sx, sy] = evPos(e), [wx, wy] = toWorld(sx, sy);
    canvas.setPointerCapture(e.pointerId);
    const t = S.ctool;
    if (t === 'erase') {
      const o = hitObstacle(wx, wy);
      if (o) { S.course.obstacles = S.course.obstacles.filter(q => q !== o); courseChanged(); }
    } else if (t === 'cmove') {
      const o = hitObstacle(wx, wy);
      if (o) S.drag = { kind: 'cmove', o, dx: wx - o.x, moved: false };
    } else {
      S.drag = { kind: 'new', x0: snap(wx), y0: Math.max(0, snap(wy)) };
    }
  }
  function courseMove(e) {
    const [sx, sy] = evPos(e), [wx, wy] = toWorld(sx, sy);
    const d = S.drag;
    if (d && d.kind === 'cmove') {
      d.o.x = Math.max(RW.START_ZONE, Math.min(RW.COURSE_END - d.o.w, snap(wx - d.dx)));
      d.moved = true;
    }
    if (d && d.kind === 'new') d.preview = dragObstacle(d, snap(wx), Math.max(0, snap(wy)));
    S.hover = (S.ctool === 'erase' || S.ctool === 'cmove') && !d ? (o => o && { obstacle: o })(hitObstacle(wx, wy)) : null;
    canvas.style.cursor = S.hover ? 'pointer' : 'crosshair';
  }
  function courseUp(e) {
    const d = S.drag; S.drag = null;
    if (!d) return;
    if (d.kind === 'cmove') { if (d.moved) courseChanged(); return; }
    const [sx, sy] = evPos(e), [wx, wy] = toWorld(sx, sy);
    const o = dragObstacle(d, snap(wx), Math.max(0, snap(wy)));
    if (o.w < 0.1) { toast('Drag sideways to set how wide it is, and up for how tall'); return; }
    if (o.x < RW.START_ZONE) { toast('Keep the green start area clear'); return; }
    if (o.x + o.w > RW.COURSE_END) o.w = RW.COURSE_END - o.x;
    if (S.course.obstacles.length >= 40) { toast('40 obstacles is the limit'); return; }
    o.id = 1 + Math.max(0, ...S.course.obstacles.map(q => q.id));
    S.course.obstacles.push(o);
    courseChanged();
  }
  function drawCourseEditor() {
    // the robot at the start, for scale
    if (S.design.points.length) {
      let minX = Infinity, minY = Infinity;
      for (const p of S.design.points) { minX = Math.min(minX, p.x); minY = Math.min(minY, p.y); }
      const dx = -minX, dy = RW.FOOT_R + 0.01 - minY, w = Math.max(2, stickWidth());
      ctx.globalAlpha = 0.6;
      for (const s of S.design.sticks) {
        const a = pointById(s.a), b = pointById(s.b);
        const [ax, ay] = toScreen(a.x + dx, a.y + dy), [bx, by] = toScreen(b.x + dx, b.y + dy);
        drawSegment(ax, ay, bx, by, COLORS.stick, w);
      }
      const h = pointById(S.design.head);
      if (h) { const [hx, hy] = toScreen(h.x + dx, h.y + dy); drawHead(hx, hy, RW.HEAD_R * S.cam.s, 0, false, S.design.camera); }
      ctx.globalAlpha = 1;
    }
    if (S.drag && S.drag.kind === 'new' && S.drag.preview && S.drag.preview.w > 0.01) {
      const o = S.drag.preview;
      drawObstacle(o, false, true);
      const [lx, ly] = toScreen(o.x + o.w / 2, o.h);
      ctx.fillStyle = '#1d2a3a'; ctx.font = '12px system-ui'; ctx.textAlign = 'center';
      ctx.fillText(`${o.w.toFixed(2)} m wide · ${o.h.toFixed(2)} m tall`, lx, ly - 10);
    }
    if (!S.course.obstacles.length && !S.drag) {
      ctx.fillStyle = 'rgba(29,42,58,.6)'; ctx.font = '16px system-ui'; ctx.textAlign = 'center';
      ctx.fillText('Drag on the track to add an obstacle, or pick a course on the left', W / 2, H * 0.3);
    }
  }
  // A new course doesn't change the brain's shape, so keep the brain and let it
  // carry on learning; only the old best score no longer compares.
  function courseChanged() {
    if (S.bestEver) S.bestEver = { ...S.bestEver, fitness: -Infinity };
    S.trainer = null; S.running = false; S.watch = null;
    save(); updateUI();
  }

  // Dropping a point on top of another joins them.
  function mergeAt(p) {
    const other = S.design.points.find(q => q !== p && Math.hypot(q.x - p.x, q.y - p.y) < 0.02);
    if (!other) return;
    for (const s of S.design.sticks) { if (s.a === p.id) s.a = other.id; if (s.b === p.id) s.b = other.id; }
    S.design.sticks = S.design.sticks.filter(s => s.a !== s.b);
    S.design.points = S.design.points.filter(q => q !== p);
    if (S.design.head === p.id) S.design.head = other.id;
  }

  // ---------- training ----------
  function canTrain() {
    const info = RW.analyse(S.design);
    if (!S.design.sticks.length) return 'Build a robot first';
    if (!info.servos) return 'Your robot needs at least one servo to move';
    const head = pointById(S.design.head);
    if (!head) return 'Your robot needs a head: pick the Head tool and click a point';
    const minY = Math.min(...S.design.points.map(p => p.y));
    if (head.y - minY < RW.HEAD_R + 0.1) return 'The head is too close to the ground: put it higher up';
    return null;
  }
  function ensureTrainer() {
    if (S.trainer) return true;
    const why = canTrain();
    if (why) { toast(why); return false; }
    S.trainer = new RW.Trainer(S.design, { ...S.opts, course: S.course, seed: S.bestEver });
    S.trainer.history = S.history;
    S.trainer.generation = S.history.length;
    return true;
  }
  function onGeneration() {
    const T = S.trainer;
    S.bestEver = T.bestEver; S.history = T.history;
    save(); drawChart(); updateStats();
  }

  // ---------- watching ----------
  function startWatch() {
    if (!S.bestEver) { S.watch = null; return; }
    const brain = new RW.Brain(S.bestEver.genome, RW.analyse(S.design));
    S.watch = new RW.Run(S.design, brain, { strength: S.opts.strength, rules: S.opts.rules, course: S.course, keepGoing: S.wlen !== 'train' });
    S.watch.result = null; S.watch.hold = 0; S.watch.best = 0; S.watch.bestT = 0;
    S.cam.x = 1;
  }

  // ---------- main loop ----------
  let acc = 0, last = performance.now();
  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000); last = now;
    if (S.mode === 'train' && S.running && S.trainer) {
      if (S.speed === 'max') {
        const t0 = performance.now();
        while (performance.now() - t0 < 14) if (S.trainer.step()) onGeneration();
      } else {
        acc += dt * S.speed;
        let n = 0;
        while (acc >= RW.DT && n < 400) { acc -= RW.DT; n++; if (S.trainer.step()) onGeneration(); }
      }
      updateLive();
    } else acc = 0;
    if (S.mode === 'watch' && S.watch) stepWatch(dt);
    render();
    requestAnimationFrame(frame);
  }

  let wacc = 0;
  function stepWatch(dt) {
    const w = S.watch;
    if (w.result) {
      w.hold -= dt;
      if (w.hold <= 0) startWatch();
      return;
    }
    wacc += dt * S.wspeed;
    while (wacc >= RW.DT) {
      wacc -= RW.DT;
      w.step();
      if (w.fallen) { w.result = 'fell'; w.hold = 2.5; break; }
      if (S.wlen === 'train') {
        // same rules as training: stop at the finish or when time runs out
        if (w.finishTime != null) { w.result = 'finish'; w.hold = 3; break; }
        if (w.robot.t >= S.opts.timeLimit) { w.result = 'time'; w.hold = 2; break; }
        continue;
      }
      // long replay: keep going past the finish until time is up, it gets stuck, or the ground runs out
      if (w.dist > w.best + 0.25) { w.best = w.dist; w.bestT = w.robot.t; }
      if (w.robot.t - w.bestT > 10) { w.result = 'stuck'; w.hold = 3; break; }
      if (w.dist >= RW.WORLD_END - 5) { w.result = 'end'; w.hold = 3; break; }
      if (S.wlen !== 'forever' && w.robot.t >= +S.wlen) { w.result = 'time'; w.hold = 3; break; }
    }
    updateAccelReadout();
  }

  // ---------- drawing ----------
  function render() {
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    if (S.mode === 'build') {
      if (!S.buildCam) fitBuildCamera();
      Object.assign(S.cam, S.buildCam);
    } else if (S.mode === 'course') {
      // close enough to edit small obstacles; scroll along with the wheel or slider
      S.cam.s = Math.max(W / (RW.COURSE_END + 2), Math.min(130, H / 5));
      S.cam.x = clampCourseX(S.courseX);
      S.cam.y = (H * 0.25) / S.cam.s;
    } else {
      S.cam.s = Math.min(150, Math.max(50, H / 4));
      S.cam.y = (H / 2 - H * 0.78) / -S.cam.s;
      let target = null;
      if (S.mode === 'train' && S.trainer) { const L = S.trainer.leader(); target = RW.comX(L.robot); }
      if (S.mode === 'watch' && S.watch) target = RW.comX(S.watch.robot);
      if (target != null && isFinite(target)) S.cam.x += (target + 1 - S.cam.x) * 0.08;
    }
    drawWorld();
    if (S.mode === 'build') drawDesign();
    else if (S.mode === 'course') drawCourseEditor();
    else if (S.mode === 'train' && S.trainer) drawTraining();
    else if (S.mode === 'watch' && S.watch) drawRobot(S.watch.robot, { alpha: 1, lead: true, sensors: true, fallen: S.watch.fallen });
    drawHud();
  }

  // The start line is x = 0; distances are measured from it.
  function startX() { return 0; }

  function drawWorld() {
    const g = ctx.createLinearGradient(0, 0, 0, H);
    g.addColorStop(0, COLORS.sky1); g.addColorStop(1, COLORS.sky2);
    ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
    const [x0] = toWorld(0, 0), [x1] = toWorld(W, 0);
    const [, gy] = toScreen(0, 0);

    if (S.mode === 'build') {
      // fine grid for building
      const [, yTop] = toWorld(0, 0);
      for (let x = Math.floor(x0 / 0.1) * 0.1; x < x1; x += 0.1) {
        const [sx] = toScreen(x, 0);
        ctx.strokeStyle = Math.abs(x / 0.5 - Math.round(x / 0.5)) < 1e-3 ? COLORS.grid2 : COLORS.grid;
        ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, gy); ctx.stroke();
      }
      for (let y = 0; y < yTop; y += 0.1) {
        const [, sy] = toScreen(0, y);
        ctx.strokeStyle = Math.abs(y / 0.5 - Math.round(y / 0.5)) < 1e-3 ? COLORS.grid2 : COLORS.grid;
        ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(W, sy); ctx.stroke();
      }
    } else {
      // distant hills for a sense of motion
      ctx.fillStyle = 'rgba(120,160,120,.25)';
      ctx.beginPath(); ctx.moveTo(0, gy);
      for (let sx = 0; sx <= W; sx += 8) {
        const wx = (sx - W / 2) / S.cam.s * 0.4 + S.cam.x * 0.4;
        ctx.lineTo(sx, gy - 40 - 30 * Math.sin(wx * 0.7) - 18 * Math.sin(wx * 1.9 + 1));
      }
      ctx.lineTo(W, gy); ctx.fill();
    }

    if (S.mode === 'course') {
      const [zx0] = toScreen(-5, 0), [zx1] = toScreen(RW.START_ZONE, 0);
      ctx.fillStyle = COLORS.zone; ctx.fillRect(zx0, 0, zx1 - zx0, gy);
      ctx.fillStyle = 'rgba(13,94,68,.7)'; ctx.font = '12px system-ui'; ctx.textAlign = 'center';
      ctx.fillText('start area', (Math.max(0, zx0) + zx1) / 2, 20);
    }
    // ground
    ctx.fillStyle = COLORS.dirt; ctx.fillRect(0, gy, W, H - gy);
    ctx.fillStyle = COLORS.ground; ctx.fillRect(0, gy, W, 8);
    ctx.fillStyle = COLORS.ground2; ctx.fillRect(0, gy + 8, W, 3);

    // distance markers every metre from the start
    const sx0 = startX();
    ctx.font = '12px system-ui'; ctx.textAlign = 'center';
    for (let m = Math.floor(x0 - sx0) - 1; m < x1 - sx0 + 1; m++) {
      if (m < 0 || m > RW.WORLD_END) continue;
      const [mx] = toScreen(sx0 + m, 0);
      ctx.fillStyle = 'rgba(255,255,255,.55)';
      ctx.fillRect(mx - 1, gy + 11, 2, m % 5 === 0 ? 12 : 6);
      if (m % 5 === 0 && S.mode !== 'build') { ctx.fillStyle = 'rgba(255,255,255,.85)'; ctx.fillText(m + ' m', mx, gy + 36); }
    }
    // start line
    const [stx] = toScreen(sx0, 0);
    ctx.fillStyle = 'rgba(255,255,255,.8)'; ctx.fillRect(stx - 2, gy - 2, 4, 12);
    if (S.mode === 'build') { ctx.fillStyle = 'rgba(40,60,90,.5)'; ctx.fillText('start', stx, gy + 36); }
    // finish flag
    const [fx] = toScreen(sx0 + RW.FINISH, 0);
    if (fx > -40 && fx < W + 40) drawFlag(fx, gy);
    drawObstacles();
  }

  function drawObstacles(highlight) {
    for (const o of S.course.obstacles) drawObstacle(o, S.hover && S.hover.obstacle === o);
  }
  function drawObstacle(o, hot, preview) {
    const poly = RW.obstaclePoly(o);
    ctx.beginPath();
    poly.forEach(([x, y], i) => { const [sx, sy] = toScreen(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    ctx.closePath();
    ctx.globalAlpha = preview ? 0.5 : 1;
    ctx.fillStyle = COLORS.rock; ctx.fill();
    ctx.strokeStyle = hot ? COLORS.hover : COLORS.rockEdge; ctx.lineWidth = hot ? 3 : 1.5; ctx.stroke();
    ctx.globalAlpha = 1;
  }

  function drawFlag(fx, gy) {
    const h = Math.max(60, S.cam.s * 1.6);
    // chequered strip on the ground
    for (let i = 0; i < 6; i++) { ctx.fillStyle = i % 2 ? '#111' : '#fff'; ctx.fillRect(fx - 6 + (i % 2) * 6, gy + i * 2, 6, 2); ctx.fillStyle = i % 2 ? '#fff' : '#111'; ctx.fillRect(fx - 6 + ((i + 1) % 2) * 6, gy + i * 2, 6, 2); }
    ctx.fillStyle = '#444'; ctx.fillRect(fx - 2, gy - h, 4, h);
    const cw = 9, rows = 4, cols = 5;
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
      ctx.fillStyle = (r + c) % 2 ? '#111' : '#fff';
      ctx.fillRect(fx + 2 + c * cw, gy - h + r * cw, cw, cw);
    }
    ctx.fillStyle = '#1d2a3a'; ctx.font = 'bold 13px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('FINISH', fx, gy - h - 8);
  }

  function stickWidth() { return Math.max(4, RW.STICK_R * 2.4 * S.cam.s); }

  function drawSegment(ax, ay, bx, by, color, width) {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  }
  function drawServo(x, y, r, target, continuous) {
    ctx.fillStyle = COLORS.servo; ctx.strokeStyle = '#7a4d00'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    const teeth = 8;
    for (let i = 0; i < teeth * 2; i++) {
      const a = (i / (teeth * 2)) * Math.PI * 2 + (target || 0);
      const rr = i % 2 ? r : r * 1.25;
      ctx.lineTo(x + Math.cos(a) * rr, y + Math.sin(a) * rr);
    }
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#7a4d00'; ctx.beginPath(); ctx.arc(x, y, r * 0.35, 0, 7); ctx.fill();
    if (continuous) {
      // a ring with an arrowhead: this one can spin all the way round
      ctx.strokeStyle = '#7a4d00'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, r * 1.6, -2.6 + (target || 0), 1.9 + (target || 0)); ctx.stroke();
      const a = 1.9 + (target || 0), ax = x + Math.cos(a) * r * 1.6, ay = y + Math.sin(a) * r * 1.6;
      ctx.fillStyle = '#7a4d00'; ctx.beginPath();
      ctx.moveTo(ax + Math.cos(a + 1.57) * 5, ay + Math.sin(a + 1.57) * 5);
      ctx.lineTo(ax + Math.cos(a) * 4, ay + Math.sin(a) * 4); ctx.lineTo(ax - Math.cos(a) * 4, ay - Math.sin(a) * 4); ctx.fill();
    }
  }
  function drawHinge(x, y, r) {
    ctx.fillStyle = COLORS.hinge; ctx.strokeStyle = COLORS.stick; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, r * 0.6, 0, 7); ctx.fill(); ctx.stroke();
  }
  function drawWeld(x, y, r) {
    ctx.fillStyle = COLORS.weld; ctx.fillRect(x - r * 0.7, y - r * 0.7, r * 1.4, r * 1.4);
  }
  function drawAccel(x, y, ang, r) {
    ctx.save(); ctx.translate(x, y); ctx.rotate(-ang);
    ctx.fillStyle = COLORS.accel; ctx.strokeStyle = '#0d5e44'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.roundRect(-r * 1.1, -r * 0.8, r * 2.2, r * 1.6, 3); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#0d5e44'; ctx.font = `bold ${Math.round(r * 1.2)}px system-ui`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('A', 0, 1);
    ctx.restore(); ctx.textBaseline = 'alphabetic';
  }
  // The head: a ball with an eye looking forward and an antenna pointing up,
  // so you can see at a glance whether it is level.
  function drawHead(x, y, rad, tilt, fallen, camera) {
    ctx.save(); ctx.translate(x, y); ctx.rotate(-tilt);
    ctx.strokeStyle = COLORS.stick; ctx.lineWidth = Math.max(2, rad * 0.12);
    ctx.beginPath(); ctx.moveTo(0, -rad); ctx.lineTo(0, -rad * 1.55); ctx.stroke();
    ctx.fillStyle = fallen ? COLORS.fell : COLORS.servo;
    ctx.beginPath(); ctx.arc(0, -rad * 1.6, rad * 0.18, 0, 7); ctx.fill();
    ctx.fillStyle = fallen ? '#ffd6e0' : COLORS.head;
    ctx.beginPath(); ctx.arc(0, 0, rad, 0, 7); ctx.fill(); ctx.stroke();
    // visor
    ctx.fillStyle = COLORS.stick;
    ctx.beginPath(); ctx.roundRect(rad * 0.05, -rad * 0.45, rad * 0.85, rad * 0.5, rad * 0.2); ctx.fill();
    if (fallen) {
      ctx.strokeStyle = COLORS.fell; ctx.lineWidth = Math.max(1.5, rad * 0.1);
      const ex = rad * 0.5, ey = -rad * 0.2, e = rad * 0.16;
      ctx.beginPath(); ctx.moveTo(ex - e, ey - e); ctx.lineTo(ex + e, ey + e); ctx.moveTo(ex + e, ey - e); ctx.lineTo(ex - e, ey + e); ctx.stroke();
    } else {
      ctx.fillStyle = '#7df9ff';
      ctx.beginPath(); ctx.arc(rad * 0.55, -rad * 0.2, rad * 0.13, 0, 7); ctx.fill();
    }
    if (camera) {
      // the time-of-flight camera: a lens on the front of the head
      ctx.fillStyle = '#2a1b47'; ctx.strokeStyle = COLORS.cam; ctx.lineWidth = Math.max(1.5, rad * 0.1);
      ctx.beginPath(); ctx.arc(rad * 0.82, rad * 0.32, rad * 0.24, 0, 7); ctx.fill(); ctx.stroke();
      ctx.fillStyle = COLORS.cam; ctx.beginPath(); ctx.arc(rad * 0.82, rad * 0.32, rad * 0.09, 0, 7); ctx.fill();
    }
    ctx.restore();
  }
  // What the camera covers, drawn in the editor as a faint fan.
  function drawFov(x, y, tilt) {
    const [cx, cy] = toScreen(x, y), r = RW.CAM_RANGE * S.cam.s, [, gy] = toScreen(0, 0);
    ctx.save(); ctx.beginPath(); ctx.rect(0, 0, W, gy); ctx.clip(); // the camera can't see through the ground
    ctx.fillStyle = 'rgba(180,108,255,.10)'; ctx.strokeStyle = 'rgba(180,108,255,.35)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, r, -(tilt + RW.CAM_UP), -(tilt + RW.CAM_DOWN));
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();
  }
  // The camera's rays: purple where they hit something, faint where they see nothing.
  function drawRays(cam) {
    ctx.lineWidth = 1.5;
    for (const h of cam.hits) {
      if (!h) continue;
      const [x0, y0] = toScreen(h.x0, h.y0), [x1, y1] = toScreen(h.x1, h.y1);
      ctx.strokeStyle = h.hit ? 'rgba(180,108,255,.75)' : 'rgba(180,108,255,.18)';
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
      if (h.hit) { ctx.fillStyle = COLORS.cam; ctx.beginPath(); ctx.arc(x1, y1, 3, 0, 7); ctx.fill(); }
    }
  }
  const jointR = () => Math.max(6, Math.min(12, S.cam.s * 0.07));

  function drawDesign() {
    const d = S.design, w = stickWidth(), r = jointR();
    const hov = S.hover;
    for (const s of d.sticks) {
      const a = pointById(s.a), b = pointById(s.b);
      const [ax, ay] = toScreen(a.x, a.y), [bx, by] = toScreen(b.x, b.y);
      drawSegment(ax, ay, bx, by, hov && hov.stick === s ? COLORS.hover : COLORS.stick, w);
    }
    for (const s of d.sticks) if (s.accel) {
      const a = pointById(s.a), b = pointById(s.b);
      const [ax, ay] = toScreen(a.x, a.y), [bx, by] = toScreen(b.x, b.y);
      drawAccel((ax + bx) / 2, (ay + by) / 2, Math.atan2(b.y - a.y, b.x - a.x), r);
    }
    for (const p of d.points) {
      const [x, y] = toScreen(p.x, p.y);
      const n = stickCount(p.id);
      if (p.kind === 'servo' && n >= 2) {
        drawServo(x, y, r, 0, RW.isContinuous(p));
        ctx.fillStyle = '#7a4d00'; ctx.font = 'bold 11px system-ui'; ctx.textAlign = 'right';
        ctx.fillText(RW.isContinuous(p) ? '↻' : `±${RW.servoRangeDeg(p)}°`, x - r * 1.4, y + r * 1.9);
        if (S.locked && S.locked.some(l => l.pointId === p.id)) {
          ctx.strokeStyle = COLORS.fell; ctx.lineWidth = 3; ctx.setLineDash([4, 3]);
          ctx.beginPath(); ctx.arc(x, y, r * 1.7, 0, 7); ctx.stroke(); ctx.setLineDash([]);
          ctx.fillStyle = COLORS.fell; ctx.font = 'bold 12px system-ui'; ctx.textAlign = 'left';
          ctx.fillText('can’t move', x + r * 1.9, y - r);
        }
      }
      else if (p.kind === 'weld' && n >= 2) drawWeld(x, y, r);
      else if (n >= 2) drawHinge(x, y, r);
      else { ctx.fillStyle = COLORS.stick; ctx.beginPath(); ctx.arc(x, y, w / 2 + 1, 0, 7); ctx.fill(); }
      if (p.id === d.head) {
        if (d.camera) drawFov(p.x, p.y, 0);
        drawHead(x, y, RW.HEAD_R * S.cam.s, 0, false, d.camera);
      }
      if (hov && hov.point === p) { ctx.strokeStyle = COLORS.hover; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(x, y, r + 4, 0, 7); ctx.stroke(); }
    }
    // stick being drawn
    if (S.drag && S.drag.kind === 'stick' && S.mouse) {
      const f = S.drag.from;
      const [fx, fy] = toScreen(f.x, f.y);
      let [mx, my] = S.mouse;
      const hp = hitPoint(mx, my);
      if (hp) [mx, my] = toScreen(hp.x, hp.y);
      else { const [wx, wy] = toWorld(mx, my); [mx, my] = toScreen(snap(wx), Math.max(0, snap(wy))); }
      ctx.globalAlpha = 0.5; drawSegment(fx, fy, mx, my, COLORS.stick, w); ctx.globalAlpha = 1;
      const len = Math.hypot(mx - fx, my - fy) / S.cam.s;
      ctx.fillStyle = '#1d2a3a'; ctx.font = '12px system-ui'; ctx.fillText(len.toFixed(2) + ' m', (fx + mx) / 2 + 12, (fy + my) / 2 - 12);
    }
    if (!d.sticks.length) {
      ctx.fillStyle = 'rgba(29,42,58,.6)'; ctx.font = '16px system-ui'; ctx.textAlign = 'center';
      ctx.fillText('Drag to draw a stick, or pick a robot on the left to start from', W / 2, H * 0.35);
    }
  }

  function drawRobot(robot, o) {
    const w = stickWidth(), r = jointR();
    ctx.globalAlpha = o.alpha;
    const col = o.lead ? COLORS.stick : COLORS.ghost;
    for (const st of robot.sticks) {
      const b = st.body, p = b.getPosition(), a = b.getAngle(), c = Math.cos(a), s = Math.sin(a);
      const [ax, ay] = toScreen(p.x - c * st.half, p.y - s * st.half), [bx, by] = toScreen(p.x + c * st.half, p.y + s * st.half);
      drawSegment(ax, ay, bx, by, col, w);
    }
    if (o.lead || o.alpha > 0.3) {
      for (const st of robot.sticks) if (st.accel) {
        const p = st.body.getPosition(); const [x, y] = toScreen(p.x, p.y);
        drawAccel(x, y, st.body.getAngle(), r);
      }
      for (const h of robot.hinges) { const p = h.joint.getAnchorA(); const [x, y] = toScreen(p.x, p.y); drawHinge(x, y, r); }
      for (const h of robot.welds) { const p = h.joint.getAnchorA(); const [x, y] = toScreen(p.x, p.y); drawWeld(x, y, r); }
      for (const sv of robot.servos) { const p = sv.joint.getAnchorA(); const [x, y] = toScreen(p.x, p.y); drawServo(x, y, r, -sv.joint.getJointAngle(), sv.continuous); }
    } else {
      // faint ghosts: skip the detail
      ctx.fillStyle = COLORS.servo;
      for (const sv of robot.servos) { const p = sv.joint.getAnchorA(); const [x, y] = toScreen(p.x, p.y); ctx.beginPath(); ctx.arc(x, y, r * 0.6, 0, 7); ctx.fill(); }
    }
    if (robot.head) {
      const hp = RW.headPos(robot), [hx, hy] = toScreen(hp.x, hp.y);
      if (robot.camera && (o.lead || o.sensors)) drawRays(robot.camera);
      drawHead(hx, hy, RW.HEAD_R * S.cam.s, robot.head.body.getAngle() - robot.head.angle0, o.fallen, !!robot.camera);
    }
    if (o.sensors) {
      // the accelerometer's reading, drawn as an arrow (it points "up" when still)
      for (const a of robot.accels) {
        const p = a.body.getPosition(), ang = a.body.getAngle(), c = Math.cos(ang), s = Math.sin(ang);
        const wx = c * a.f.x - s * a.f.y, wy = s * a.f.x + c * a.f.y;
        const k = 0.05;
        const [x0, y0] = toScreen(p.x, p.y), [x1, y1] = toScreen(p.x + wx * k, p.y + wy * k);
        ctx.strokeStyle = COLORS.accel; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
        const h = Math.atan2(y1 - y0, x1 - x0);
        ctx.fillStyle = COLORS.accel; ctx.beginPath(); ctx.moveTo(x1, y1);
        ctx.lineTo(x1 - 10 * Math.cos(h - 0.4), y1 - 10 * Math.sin(h - 0.4)); ctx.lineTo(x1 - 10 * Math.cos(h + 0.4), y1 - 10 * Math.sin(h + 0.4)); ctx.fill();
      }
    }
    ctx.globalAlpha = 1;
  }

  function drawTraining() {
    const T = S.trainer, L = T.leader();
    if (S.ghosts) for (const a of T.agents) if (a !== L) drawRobot(a.robot, { alpha: a.fallen ? 0.08 : 0.16, fallen: a.fallen });
    drawRobot(L.robot, { alpha: 1, lead: true, fallen: L.fallen });
  }

  function drawHud() {
    const hud = $('#hud');
    let html = '';
    if (S.mode === 'train' && S.trainer) {
      const T = S.trainer;
      const fell = T.agents.filter(a => a.fallen).length;
      html = `Generation ${T.generation + 1} · ${T.t.toFixed(1)} / ${T.timeLimit}s · ${fell} fallen over`;
      if (!S.running) html += ' · paused';
    } else if (S.mode === 'train') {
      html = 'Press Start learning';
    } else if (S.mode === 'watch') {
      const w = S.watch;
      if (!w) html = 'Nothing learned yet, train your robot first';
      else {
        html = `${w.robot.t.toFixed(1)}s · ${Math.max(0, w.dist).toFixed(2)} m · head tilt ${Math.round(w.tilt * 180 / Math.PI)}° · kept level ${Math.round(w.level * 100)}%`;
        if (w.finishTime != null && S.wlen !== 'train') html += `<br>🏁 crossed the finish at ${w.finishTime.toFixed(1)}s`;
        if (w.result === 'finish') html += `<div class="big-msg">🏁 Made it in ${w.robot.t.toFixed(1)}s!</div>`;
        if (w.result === 'time') html += `<div class="big-msg">${S.wlen === 'train' ? 'Out of time' : 'Time’s up'} at ${w.dist.toFixed(1)} m</div>`;
        if (w.result === 'stuck') html += `<div class="big-msg">Stuck at ${w.dist.toFixed(1)} m (no progress for 10 s)</div>`;
        if (w.result === 'end') html += `<div class="big-msg">Reached the end of the world! ${w.dist.toFixed(0)} m in ${w.robot.t.toFixed(0)}s</div>`;
        if (w.result === 'fell') html += `<div class="big-msg">💥 Fell over at ${Math.max(0, w.dist).toFixed(1)} m</div>`;
      }
    }
    if (hud.innerHTML !== html) hud.innerHTML = html;
  }

  function drawChart() {
    const cw = chart.clientWidth || 280, ch = chart.clientHeight || 130, dpr = window.devicePixelRatio || 1;
    if (chart.width !== Math.round(cw * dpr)) { chart.width = Math.round(cw * dpr); chart.height = Math.round(ch * dpr); }
    cctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    cctx.clearRect(0, 0, cw, ch);
    const hist = S.history;
    const pad = { l: 30, r: 8, t: 8, b: 18 };
    const maxY = Math.max(2, RW.FINISH * (hist.some(h => h.best > RW.FINISH * 0.5) ? 1.05 : 0.5), ...hist.map(h => h.best));
    const n = Math.max(10, hist.length);
    const X = i => pad.l + (i / (n - 1)) * (cw - pad.l - pad.r);
    const Y = v => ch - pad.b - (Math.max(-1, v) / maxY) * (ch - pad.t - pad.b);
    cctx.font = '10px system-ui'; cctx.fillStyle = '#94a0b8'; cctx.strokeStyle = 'rgba(148,160,184,.2)'; cctx.lineWidth = 1;
    const step = maxY > 12 ? 5 : maxY > 5 ? 2 : 1;
    for (let v = 0; v <= maxY; v += step) { cctx.beginPath(); cctx.moveTo(pad.l, Y(v)); cctx.lineTo(cw - pad.r, Y(v)); cctx.stroke(); cctx.textAlign = 'right'; cctx.fillText(v + 'm', pad.l - 4, Y(v) + 3); }
    if (maxY >= RW.FINISH) { cctx.setLineDash([4, 3]); cctx.strokeStyle = '#3ad29f'; cctx.beginPath(); cctx.moveTo(pad.l, Y(RW.FINISH)); cctx.lineTo(cw - pad.r, Y(RW.FINISH)); cctx.stroke(); cctx.setLineDash([]); }
    cctx.textAlign = 'center'; cctx.fillText('generation', (cw + pad.l) / 2, ch - 4);
    if (!hist.length) return;
    const line = (key, color, wdt) => {
      cctx.strokeStyle = color; cctx.lineWidth = wdt; cctx.beginPath();
      hist.forEach((h, i) => (i ? cctx.lineTo(X(i), Y(h[key])) : cctx.moveTo(X(i), Y(h[key]))));
      if (hist.length === 1) cctx.lineTo(X(0) + 2, Y(hist[0][key]));
      cctx.stroke();
    };
    line('avg', '#94a0b8', 1.5);
    line('best', '#ffb020', 2);
  }

  // ---------- panel ----------
  const fmtDist = v => (v == null ? '–' : Math.max(0, v).toFixed(1) + ' m');
  function bestText(b) {
    if (!b) return '–';
    return (b.time != null ? `🏁 ${b.time.toFixed(1)}s` : fmtDist(b.distance)) + (b.level != null ? ` · ${Math.round(b.level * 100)}%` : '');
  }
  function updateStats() {
    const T = S.trainer, last = S.history[S.history.length - 1];
    $('#gen').textContent = T ? T.generation + 1 : S.history.length;
    $('#best').textContent = bestText(S.bestEver);
    $('#fin').textContent = last ? `${last.finished} / ${last.fell || 0}` : '0 / 0';
  }
  let liveTick = 0;
  function updateLive() {
    if (++liveTick % 6) return;
    const T = S.trainer;
    $('#lead').textContent = fmtDist(T.leader().dist);
    $('#prog').style.width = (100 * T.t / T.timeLimit).toFixed(1) + '%';
    $('#gen').textContent = T.generation + 1;
  }
  function drawDepth() {
    const cv = $('#depth'), cam = S.watch && S.watch.robot.camera;
    $('#camRead').hidden = !!cam;
    cv.hidden = !cam;
    if (!cam) return;
    const cw = cv.clientWidth || 260, ch = 70, dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(cw * dpr)) { cv.width = Math.round(cw * dpr); cv.height = Math.round(ch * dpr); }
    const c = cv.getContext('2d');
    c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, cw, ch);
    const n = RW.CAM_RAYS, bw = cw / n;
    for (let i = 0; i < n; i++) {
      const near = 1 - cam.depth[i];
      c.fillStyle = `rgba(180,108,255,${0.15 + 0.85 * near})`;
      c.fillRect(i * bw + 1, ch - 14 - near * (ch - 18), bw - 2, near * (ch - 18) + 2);
      c.fillStyle = '#94a0b8'; c.font = '10px system-ui'; c.textAlign = 'center';
      c.fillText(cam.depth[i] < 1 ? (cam.depth[i] * RW.CAM_RANGE).toFixed(1) : '–', i * bw + bw / 2, ch - 2);
    }
  }
  function updateAccelReadout() {
    drawDepth();
    const w = S.watch, el = $('#accelRead');
    if (!w || !w.robot.accels.length) { el.textContent = 'No accelerometer fitted. Add one to a stick so the robot can feel which way is down.'; return; }
    el.innerHTML = w.robot.accels.map((a, i) => {
      const tilt = Math.atan2(a.f.x, a.f.y) * 180 / Math.PI;
      return `#${i + 1}: x ${a.f.x.toFixed(1)}, y ${a.f.y.toFixed(1)} m/s² · feels tilted ${tilt.toFixed(0)}°`;
    }).join('<br>') + '<br><small>The green arrow shows what it feels. Standing still it points up (gravity).</small>';
  }

  const HINTS = {
    stick: 'Drag from one point to another to place a stick. Start or end on an existing point to join them; where sticks meet is a free hinge.',
    servo: 'Click a joint (where two or more sticks meet) to put a servo there; it can swing ±90° from how you built it. Click it again for ±135°, ±180°, then continuous rotation (spins all the way round, like a wheel motor), then to remove it.',
    weld: 'Click a joint to weld it solid, so those sticks can’t bend there.',
    accel: 'Click a stick to fit an accelerometer. It feels gravity and bumps, so the brain knows which way is down.',
    block: 'Drag along the track to make a block: sideways for width, up for height.',
    ramp: 'Drag from the bottom of the slope to its top.',
    bump: 'Drag along the track to make a rounded bump: sideways for width, up for height.',
    cmove: 'Drag an obstacle to slide it along the track.',
    cerase: 'Click an obstacle to remove it.',
    camera: 'Click anywhere to fit or remove a time-of-flight camera on the head. It measures the distance along 9 rays ahead, so the robot can see the ground and obstacles coming.',
    head: 'Click a point to put the head there. The robot must keep its head level and off the ground: it starts out level the way you build it.',
    move: 'Drag a point to move it. Drop it on another point to join them.',
    erase: 'Click a point or stick to remove it.',
  };

  function updateUI() {
    $$('#modes button').forEach(b => b.classList.toggle('on', b.dataset.mode === S.mode));
    $$('#panel section').forEach(s => (s.hidden = s.dataset.for !== S.mode));
    $$('.tools button').forEach(b => b.classList.toggle('on', b.dataset.tool === S.tool));
    $('#hint').textContent = HINTS[S.tool];
    $$('.ctools button').forEach(b => b.classList.toggle('on', b.dataset.ctool === S.ctool));
    $('#chint').textContent = HINTS[S.ctool === 'erase' ? 'cerase' : S.ctool];
    $('#cparts').textContent = `${S.course.obstacles.length} obstacle${S.course.obstacles.length === 1 ? '' : 's'}`;
    const info = RW.analyse(S.design);
    const plural = (n, w) => `${n} ${w}${n === 1 ? '' : 's'}`;
    const nLocked = (S.locked || []).length;
    $('#locked').hidden = !nLocked;
    $('#locked').textContent = nLocked ? `${nLocked} servo channel${nLocked === 1 ? '' : 's'} can’t move: the sticks around ${nLocked === 1 ? 'it are' : 'them are'} braced into a rigid shape, so the joint can’t bend. Remove a brace stick, or move the servo to a joint that isn’t part of a triangle.` : '';
    $('#parts').textContent = `${plural(S.design.sticks.length, 'stick')} · ${plural(info.servos, 'servo channel')} · ${plural(info.accels, 'accelerometer')} · ${pointById(S.design.head) ? (S.design.camera ? '1 head with camera' : '1 head') : 'no head yet'}`;
    const run = $('#run');
    run.textContent = S.running ? '❚❚ Pause' : (S.history.length ? '▶ Keep learning' : '▶ Start learning');
    run.classList.toggle('pause', S.running);
    $$('#speed button').forEach(b => b.classList.toggle('on', String(S.speed) === b.dataset.speed));
    $$('#wspeed button').forEach(b => b.classList.toggle('on', String(S.wspeed) === b.dataset.speed));
    $('#pop').value = S.opts.popSize; $('#tlimit').value = S.opts.timeLimit;
    $('#strength').value = S.opts.strength; $('#strengthOut').textContent = (+S.opts.strength).toFixed(1) + '×';
    const b = S.bestEver;
    $('#winfo').textContent = b ? `Best brain so far, from generation ${b.generation}: ${b.time != null ? 'finished in ' + b.time.toFixed(1) + 's' : 'got ' + fmtDist(b.distance)}${b.fallen ? ' then fell over' : ''}, head kept level ${Math.round((b.level ?? 1) * 100)}% of the time.` : 'Nothing learned yet. Train your robot first.';
    $('#rfall').checked = S.opts.rules.fall; $('#rlevel').value = String(S.opts.rules.level);
    if (!S.trainer) { $('#lead').textContent = '–'; $('#prog').style.width = '0'; }
    updateStats(); drawChart(); updateAccelReadout();
  }

  let toastTimer;
  function toast(msg) {
    const t = $('#toast'); t.textContent = msg; t.classList.add('show');
    clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 2600);
  }

  function setMode(m) {
    if (m === S.mode) return;
    if (m === 'train' || m === 'watch') {
      const why = canTrain();
      if (why) { toast(why); return; }
    }
    if (S.mode === 'train') S.running = false;
    S.mode = m;
    if (m === 'watch') startWatch();
    if (m === 'train') { ensureTrainer(); S.cam.x = 1; }
    updateUI();
  }

  $$('#modes button').forEach(b => b.addEventListener('click', () => setMode(b.dataset.mode)));
  $$('.tools button').forEach(b => b.addEventListener('click', () => { S.tool = b.dataset.tool; updateUI(); }));
  $$('.ctools button').forEach(b => b.addEventListener('click', () => { S.ctool = b.dataset.ctool; updateUI(); }));
  for (const [key, c] of Object.entries(RW.COURSES)) {
    const b = document.createElement('button'); b.textContent = c.name;
    b.addEventListener('click', () => { S.course = c.course(); courseChanged(); toast(`${c.name} course loaded`); });
    $('#courses').appendChild(b);
  }
  $('#cscroll').addEventListener('input', e => { S.courseX = (+e.target.value / 1000) * (RW.COURSE_END + 2) - 1; });
  $('#cclear').addEventListener('click', () => { S.course = { obstacles: [] }; courseChanged(); });
  const presets = $('#presets');
  for (const [key, p] of Object.entries(RW.PRESETS)) {
    const b = document.createElement('button'); b.textContent = p.name;
    b.addEventListener('click', () => { setDesign(p.design()); toast(`${p.name} loaded. Now go to Train.`); });
    presets.appendChild(b);
  }
  $('#clear').addEventListener('click', () => setDesign({ points: [], sticks: [] }));
  $('#strength').addEventListener('input', e => { S.opts.strength = +e.target.value; $('#strengthOut').textContent = S.opts.strength.toFixed(1) + '×'; });
  $('#strength').addEventListener('change', () => designChanged());
  $('#run').addEventListener('click', () => {
    if (!S.running && !ensureTrainer()) return;
    S.running = !S.running; updateUI();
  });
  $$('#speed button').forEach(b => b.addEventListener('click', () => { S.speed = b.dataset.speed === 'max' ? 'max' : +b.dataset.speed; updateUI(); }));
  $$('#wspeed button').forEach(b => b.addEventListener('click', () => { S.wspeed = +b.dataset.speed; updateUI(); }));
  $('#pop').addEventListener('change', e => { S.opts.popSize = +e.target.value; restartTrainerKeepingBest(); });
  $('#tlimit').addEventListener('change', e => { S.opts.timeLimit = +e.target.value; restartTrainerKeepingBest(); });
  // Changing the rules changes how scores are counted, so the old best score
  // no longer compares; keep its brain as a starting point but let it be beaten.
  function rulesChanged() {
    if (S.bestEver) S.bestEver = { ...S.bestEver, fitness: -Infinity };
    restartTrainerKeepingBest();
  }
  $('#rfall').addEventListener('change', e => { S.opts.rules = { ...S.opts.rules, fall: e.target.checked }; rulesChanged(); });
  $('#rlevel').addEventListener('change', e => { S.opts.rules = { ...S.opts.rules, level: +e.target.value }; rulesChanged(); });
  $('#ghosts').addEventListener('change', e => { S.ghosts = e.target.checked; });
  $('#reset').addEventListener('click', () => { forget(); toast('Brain wiped. Starting from random again.'); });
  $('#replay').addEventListener('click', () => startWatch());
  $('#wlen').addEventListener('change', e => { S.wlen = e.target.value; startWatch(); });

  // Changing population or time limit restarts the generation but keeps the best brain as a seed.
  function restartTrainerKeepingBest() {
    const wasRunning = S.running;
    S.trainer = null; S.running = false;
    save();
    if (S.mode === 'train' && ensureTrainer()) S.running = wasRunning;
    updateUI();
  }

  // ---------- start ----------
  resize();
  if (!load()) S.design = RW.PRESETS.dog.design();
  S.nextId = 1 + Math.max(0, ...S.design.points.map(p => p.id), ...S.design.sticks.map(s => s.id));
  updateLocked();
  updateUI();
  requestAnimationFrame(frame);
  window.__RW_STATE = S; // handy for poking at from the console
})();
