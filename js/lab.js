// "How it learns": a learning lab with the simplest possible learner.
//
// A tiny inchworm with two servos. Servo 1 just swings with a steady beat.
// Servo 2 is driven by a brain of only TWO numbers, A and B:
//     servo 2 = A x beat + B x (the beat a quarter later)
// Because there are only two numbers, every possible brain is a point on a
// flat map, and we can draw the whole "map of every brain" (how far each pair
// of numbers crawls). Then we watch learning move across that map, one step
// at a time, with either method from the game:
//   - Copy the best (genetic algorithm): keep the winners, copy them with
//     small random changes.
//   - Nudge & compare (evolution strategy): one brain; try nudges both ways,
//     step in the direction that helped.
(function (RW) {
  'use strict';
  const RANGE = 4;          // A and B go from -4 to +4
  const N = 10;             // robots per generation
  const RACE_T = 8;         // seconds each robot gets
  const AMP = 0.5, FREQ = 1;
  const GRID = 41;          // map resolution
  const ELITE = 3;          // GA: winners kept
  const MUTATE = 0.5;       // GA: size of random changes
  const SIGMA = 0.5;        // ES: size of the nudges
  const ES_STEP = 1.6;      // ES: how far to step for a clear result
  const COLORS = ['#ff6b6b', '#ffa94d', '#ffd43b', '#94d82d', '#38d9a9', '#3bc9db', '#4dabf7', '#9775fa', '#e599f7', '#f783ac'];

  class TinyBrain {
    constructor(w, nSense) { this.w = w; this.freq = FREQ; this.inBuf = new Float32Array(nSense); this.out = new Float32Array(2); }
    forward(x) {
      const beat = x[1], later = x[2]; // the clock: sin and cos of the beat
      this.out[0] = AMP * beat;                                         // servo 1: just follows the beat
      this.out[1] = AMP * Math.tanh(this.w[0] * beat + this.w[1] * later); // servo 2: the two-number brain
      return this.out;
    }
  }

  const P = (id, x, y, kind, foot) => ({ id, x, y, kind: kind || 'free', ...(foot ? { foot: true } : {}) });
  const DESIGN = {
    points: [P(1, 0, 0, null, true), P(2, 0.4, 0.3, 'servo'), P(3, 0.8, 0.3, 'servo'), P(4, 1.2, 0, null, true)],
    sticks: [{ id: 5, a: 1, b: 2 }, { id: 6, a: 2, b: 3 }, { id: 7, a: 3, b: 4 }],
  };
  let NSENSE = 0;
  function makeRun(w) {
    if (!NSENSE) NSENSE = RW.inputCount(RW.analyse(DESIGN));
    const r = new RW.Run(DESIGN, new TinyBrain(w, NSENSE), { rules: { fall: false, level: 0 } });
    r.w = w;
    return r;
  }
  const travel = r => RW.comX(r.robot) - r.robot.startX; // how far it has crawled
  function tryBrain(w) { const r = makeRun(w); while (r.robot.t < RACE_T) r.step(); return travel(r); }

  function gauss() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }
  const clampW = v => Math.max(-RANGE, Math.min(RANGE, v));
  const fmt = v => (v >= 0 ? '+' : '−') + Math.abs(v).toFixed(1);
  const fmtW = w => `(${fmt(w[0])}, ${fmt(w[1])})`;
  const metres = v => `${v >= 0 ? '' : '−'}${Math.abs(v).toFixed(1)} m${v < -0.05 ? ' (backwards!)' : ''}`;

  // ---------- state ----------
  const L = {
    method: 'ga', gen: 0, phase: 'intro',
    pop: [], runs: null, scores: null,
    elite: null, parents: null,              // GA
    mean: null, eps: null, newMean: null,    // ES
    tried: [],                               // every brain tried so far: {w, v}
    trail: [],                               // best (GA) or mean (ES) per generation
    best: null,                              // best brain ever: {w, v, gen}
    auto: false, speed: 3, showMap: false,
    map: new Float32Array(GRID * GRID).fill(NaN), mapNext: 0, mapMax: null,
    raceT: 0, holdT: 0,
  };
  RW.Lab = { state: L };

  function reset() {
    Object.assign(L, { gen: 0, phase: 'intro', pop: [], runs: null, scores: null, elite: null, parents: null, mean: null, eps: null, newMean: null, tried: [], trail: [], best: null, raceT: 0, holdT: 0 });
    refresh();
  }

  // ---------- one generation, step by step ----------
  function firstPopulation() {
    L.gen = 1;
    if (L.method === 'ga') {
      L.pop = Array.from({ length: N }, () => [clampW(gauss() * 0.6), clampW(gauss() * 0.6)]);
      L.parents = null;
    } else {
      L.mean = [gauss() * 0.3, gauss() * 0.3];
      makeNudges();
    }
    L.runs = L.pop.map(w => makeRun(w));
    L.phase = 'make';
  }
  function makeNudges() {
    L.eps = Array.from({ length: N / 2 }, () => [gauss(), gauss()]);
    L.pop = [];
    for (const e of L.eps) {
      L.pop.push([clampW(L.mean[0] + SIGMA * e[0]), clampW(L.mean[1] + SIGMA * e[1])]);
      L.pop.push([clampW(L.mean[0] - SIGMA * e[0]), clampW(L.mean[1] - SIGMA * e[1])]);
    }
  }
  function startRace() {
    if (!L.runs) L.runs = L.pop.map(w => makeRun(w));
    L.scores = null; L.raceT = 0; L.phase = 'race';
  }
  function finishRace() {
    for (const r of L.runs) while (r.robot.t < RACE_T) r.step();
    L.scores = L.runs.map(travel);
    L.pop.forEach((w, i) => {
      L.tried.push({ w, v: L.scores[i] });
      if (!L.best || L.scores[i] > L.best.v) L.best = { w, v: L.scores[i], gen: L.gen };
    });
    L.phase = 'score';
  }
  function choose() {
    if (L.method === 'ga') {
      L.elite = [...L.scores.keys()].sort((a, b) => L.scores[b] - L.scores[a]).slice(0, ELITE);
    } else {
      // for each pair, which way did better? Blend those directions into one step.
      const g = [0, 0];
      L.pairWin = L.eps.map((e, j) => {
        const plus = L.scores[2 * j], minus = L.scores[2 * j + 1];
        const d = plus - minus;
        g[0] += d * e[0]; g[1] += d * e[1];
        return plus >= minus ? 2 * j : 2 * j + 1;
      });
      const len = Math.hypot(g[0], g[1]) || 1;
      // step towards the blended direction; bigger when the pairs agreed more
      const agree = Math.min(1, len / (L.eps.length * 1.5));
      const step = ES_STEP * SIGMA * (0.4 + 0.6 * agree);
      L.newMean = [clampW(L.mean[0] + step * g[0] / len), clampW(L.mean[1] + step * g[1] / len)];
    }
    L.phase = 'choose';
  }
  function nextGeneration() {
    if (L.method === 'ga') {
      const bestI = L.elite[0];
      L.trail.push(L.pop[bestI]);
      const keep = L.elite.map(i => L.pop[i]);
      L.parents = [];
      const next = keep.map((w, k) => { L.parents.push({ from: w, kept: true }); return w; });
      while (next.length < N) {
        const p = keep[(Math.random() * keep.length) | 0];
        next.push([clampW(p[0] + gauss() * MUTATE), clampW(p[1] + gauss() * MUTATE)]);
        L.parents.push({ from: p, kept: false });
      }
      L.pop = next;
    } else {
      L.trail.push(L.mean);
      L.oldMean = L.mean;
      L.mean = L.newMean;
      makeNudges();
    }
    L.gen++;
    L.runs = L.pop.map(w => makeRun(w)); L.scores = null;
    L.phase = 'next';
  }

  function advance() {
    switch (L.phase) {
      case 'intro': firstPopulation(); break;
      case 'make': case 'next': startRace(); break;
      case 'race': finishRace(); break;
      case 'score': choose(); break;
      case 'choose': nextGeneration(); break;
    }
    L.holdT = 0;
    refresh();
  }

  // ---------- per frame ----------
  RW.Lab.update = function (dt) {
    // fill in the map of every brain a little each frame
    if (L.mapNext < GRID * GRID) {
      const t0 = performance.now();
      while (L.mapNext < GRID * GRID && performance.now() - t0 < 8) {
        const i = L.mapNext++, gx = i % GRID, gy = (i / GRID) | 0;
        const w = [-RANGE + 2 * RANGE * gx / (GRID - 1), RANGE - 2 * RANGE * gy / (GRID - 1)];
        L.map[i] = tryBrain(w);
        if (!L.mapMax || L.map[i] > L.mapMax.v) L.mapMax = { w, v: L.map[i] };
      }
      if (L.mapNext >= GRID * GRID || L.mapNext % 200 < 3) refreshStats();
    }
    if (L.phase === 'race') {
      if (L.speed === 0) { finishRace(); refresh(); return; }
      let n = Math.round(dt * L.speed / RW.DT);
      while (n-- > 0 && L.runs[0].robot.t < RACE_T) for (const r of L.runs) r.step();
      L.raceT = L.runs[0].robot.t;
      if (L.raceT >= RACE_T - 1e-6) { finishRace(); refresh(); }
      return;
    }
    if (L.auto && L.phase !== 'intro') {
      L.holdT += dt;
      const wait = L.speed === 0 ? 0.6 : 2.2;
      if (L.holdT > wait) advance();
    }
  };

  // ---------- drawing ----------
  function heat(v) {
    // forwards = green, backwards = red, nothing = dark
    const t = Math.max(-1, Math.min(1, v / 4));
    if (t >= 0) return `rgb(${Math.round(24 + 20 * t)},${Math.round(30 + 170 * t)},${Math.round(44 + 70 * t)})`;
    return `rgb(${Math.round(24 + 200 * -t)},${Math.round(30 + 20 * -t)},${Math.round(44 + 30 * -t)})`;
  }

  RW.Lab.draw = function (ctx, W, H) {
    ctx.fillStyle = '#10141c'; ctx.fillRect(0, 0, W, H);
    const wide = W > H * 1.0;
    const pad = 14;
    const mapSize = wide ? Math.min(H - 2 * pad - 30, W * 0.46) : Math.min(W - 2 * pad - 30, H * 0.5);
    const mx = pad + 26, my = pad + 18;
    drawMap(ctx, mx, my, mapSize);
    if (wide) drawLanes(ctx, mx + mapSize + 30, pad, W - (mx + mapSize + 30) - pad, H - 2 * pad);
    else drawLanes(ctx, pad, my + mapSize + 34, W - 2 * pad, H - (my + mapSize + 34) - pad);
  };

  function drawMap(ctx, x0, y0, size) {
    const toX = a => x0 + (a + RANGE) / (2 * RANGE) * size;
    const toY = b => y0 + (RANGE - b) / (2 * RANGE) * size;
    // background: the map (if shown), or only what has been tried so far
    ctx.fillStyle = '#1b2130'; ctx.fillRect(x0, y0, size, size);
    const cell = size / (GRID - 1);
    if (L.showMap) {
      for (let i = 0; i < L.mapNext; i++) {
        const gx = i % GRID, gy = (i / GRID) | 0;
        ctx.fillStyle = heat(L.map[i]);
        ctx.fillRect(x0 + gx * cell - cell / 2, y0 + gy * cell - cell / 2, cell + 1, cell + 1);
      }
    } else {
      for (const t of L.tried) { ctx.fillStyle = heat(t.v); ctx.globalAlpha = 0.55; ctx.fillRect(toX(t.w[0]) - 3, toY(t.w[1]) - 3, 6, 6); }
      ctx.globalAlpha = 1;
    }
    // frame, axes, labels
    ctx.save(); ctx.beginPath(); ctx.rect(x0, y0, size, size); ctx.clip();
    ctx.strokeStyle = 'rgba(255,255,255,.18)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(toX(0), y0); ctx.lineTo(toX(0), y0 + size); ctx.moveTo(x0, toY(0)); ctx.lineTo(x0 + size, toY(0)); ctx.stroke();
    ctx.restore();
    ctx.strokeStyle = '#33405a'; ctx.strokeRect(x0, y0, size, size);
    ctx.fillStyle = '#94a0b8'; ctx.font = '11px system-ui'; ctx.textAlign = 'center';
    for (const a of [-4, -2, 0, 2, 4]) ctx.fillText(String(a), toX(a), y0 + size + 13);
    ctx.fillText('number A →', x0 + size / 2, y0 + size + 27);
    ctx.save(); ctx.translate(x0 - 16, y0 + size / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('number B →', 0, 0); ctx.restore();
    ctx.textAlign = 'left'; ctx.fillStyle = '#c9d1e0'; ctx.font = '12px system-ui';
    ctx.fillText(L.showMap ? (L.mapNext < GRID * GRID ? `Map of every brain: drawing… ${Math.round(100 * L.mapNext / (GRID * GRID))}%` : 'Map of every brain: green crawls forwards, red backwards')
      : 'Every brain tried so far (green = forwards, red = backwards)', x0, y0 - 6);

    // learning's path so far
    if (L.trail.length) {
      ctx.strokeStyle = 'rgba(255,255,255,.55)'; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
      ctx.beginPath(); L.trail.forEach((w, i) => (i ? ctx.lineTo(toX(w[0]), toY(w[1])) : ctx.moveTo(toX(w[0]), toY(w[1]))));
      const now = L.method === 'es' ? L.mean : null;
      if (now) ctx.lineTo(toX(now[0]), toY(now[1]));
      ctx.stroke(); ctx.setLineDash([]);
    }

    const ph = L.phase;
    // GA: who is whose parent
    if (L.method === 'ga' && ph === 'next' && L.parents) {
      L.pop.forEach((w, i) => {
        const p = L.parents[i];
        if (p && !p.kept) arrow(ctx, toX(p.from[0]), toY(p.from[1]), toX(w[0]), toY(w[1]), 'rgba(255,255,255,.6)', 1.3);
      });
    }
    // the robots' brains
    L.pop.forEach((w, i) => {
      const x = toX(w[0]), y = toY(w[1]);
      let alpha = 1;
      if (ph === 'choose' && L.method === 'ga' && !L.elite.includes(i)) alpha = 0.25;
      if (ph === 'choose' && L.method === 'es') alpha = 0.4;
      ctx.globalAlpha = alpha;
      ctx.fillStyle = L.scores ? heat(L.scores[i]) : '#e8ecf4';
      ctx.beginPath(); ctx.arc(x, y, 9, 0, 7); ctx.fill();
      ctx.strokeStyle = COLORS[i]; ctx.lineWidth = 2.5; ctx.stroke();
      if (ph === 'choose' && L.method === 'ga' && L.elite.includes(i)) { ctx.strokeStyle = '#ffd43b'; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(x, y, 14, 0, 7); ctx.stroke(); }
      if (L.method === 'ga' && ph === 'next' && L.parents && L.parents[i].kept) { ctx.strokeStyle = '#ffd43b'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 13, 0, 7); ctx.stroke(); }
      ctx.fillStyle = L.scores && Math.abs(L.scores[i]) > 2 ? '#0b0e14' : '#10141c';
      ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), x, y + 0.5);
      ctx.textBaseline = 'alphabetic';
    });
    ctx.globalAlpha = 1;
    // ES: the one brain (the star), its nudge pairs, and the step, on top of the dots
    if (L.method === 'es' && L.mean) {
      if (L.eps && (ph === 'make' || ph === 'next' || ph === 'race' || ph === 'score')) {
        ctx.strokeStyle = 'rgba(255,255,255,.25)'; ctx.lineWidth = 1;
        for (let j = 0; j < L.eps.length; j++) {
          const a = L.pop[2 * j], b = L.pop[2 * j + 1];
          ctx.beginPath(); ctx.moveTo(toX(a[0]), toY(a[1])); ctx.lineTo(toX(b[0]), toY(b[1])); ctx.stroke();
        }
      }
      if (ph === 'choose') {
        for (let j = 0; j < L.eps.length; j++) {
          const win = L.pairWin[j], lose = win % 2 === 0 ? win + 1 : win - 1;
          arrow(ctx, toX(L.pop[lose][0]), toY(L.pop[lose][1]), toX(L.pop[win][0]), toY(L.pop[win][1]), '#ffffff', 2);
        }
        arrow(ctx, toX(L.mean[0]), toY(L.mean[1]), toX(L.newMean[0]), toY(L.newMean[1]), '#ffd43b', 3.5);
      }
      if (ph === 'next' && L.oldMean) arrow(ctx, toX(L.oldMean[0]), toY(L.oldMean[1]), toX(L.mean[0]), toY(L.mean[1]), '#ffd43b', 3.5);
      star(ctx, toX(L.mean[0]), toY(L.mean[1]), 11);
    }
  }

  function arrow(ctx, x0, y0, x1, y1, color, width) {
    const d = Math.hypot(x1 - x0, y1 - y0);
    if (d < 4) return;
    const ux = (x1 - x0) / d, uy = (y1 - y0) / d, hx = x1 - ux * 9, hy = y1 - uy * 9;
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(hx, hy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(hx - uy * 5, hy + ux * 5); ctx.lineTo(hx + uy * 5, hy - ux * 5); ctx.fill();
  }
  function star(ctx, x, y, r) {
    ctx.fillStyle = '#ffd43b'; ctx.strokeStyle = '#7a5b00'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < 10; i++) { const a = -Math.PI / 2 + i * Math.PI / 5, rr = i % 2 ? r * 0.45 : r; ctx.lineTo(x + Math.cos(a) * rr, y + Math.sin(a) * rr); }
    ctx.closePath(); ctx.fill(); ctx.stroke();
  }

  // The race: one lane per robot.
  function drawLanes(ctx, x0, y0, w, h) {
    const n = L.pop.length || N, laneH = Math.min(46, h / n), trackX = x0 + 92, trackW = w - 92 - 64;
    const s = Math.min(trackW / 6, laneH / 0.75); // pixels per metre; the robot must fit its lane
    const toX = m => trackX + (m + 1) * s;
    ctx.font = '11px system-ui';
    ctx.fillStyle = '#c9d1e0'; ctx.textAlign = 'left';
    ctx.fillText(L.phase === 'race' ? `The race: ${L.raceT.toFixed(1)} / ${RACE_T} s` : L.scores ? 'The race: finished' : 'The race', x0, y0 + 10);
    for (let i = 0; i < n; i++) {
      const top = y0 + 18 + i * laneH, gy = top + laneH * 0.86;
      ctx.save(); ctx.beginPath(); ctx.rect(x0, top, w, laneH); ctx.clip();
      ctx.fillStyle = i % 2 ? '#161b26' : '#19202c'; ctx.fillRect(trackX - 6, top, trackW + 12, laneH);
      // start line and metre marks
      ctx.fillStyle = 'rgba(255,255,255,.25)';
      for (let m = 0; m <= 5; m++) if (toX(m) < trackX + trackW) ctx.fillRect(toX(m) - 0.5, top + 2, 1, laneH - 4);
      ctx.fillStyle = '#3a4a2a'; ctx.fillRect(trackX - 6, gy, trackW + 12, 2);
      // label
      ctx.fillStyle = COLORS[i]; ctx.beginPath(); ctx.arc(x0 + 8, top + laneH / 2, 6, 0, 7); ctx.fill();
      ctx.fillStyle = '#e8ecf4'; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.fillText(`${i + 1}`, x0 + 18, top + laneH / 2);
      if (L.pop[i]) { ctx.fillStyle = '#94a0b8'; ctx.fillText(fmtW(L.pop[i]), x0 + 30, top + laneH / 2); }
      // robot
      const run = L.runs && L.runs[i];
      if (run) {
        ctx.save(); ctx.beginPath(); ctx.rect(trackX - 6, top, trackW + 12, laneH); ctx.clip(); // keep the robot on its track
        const sx = run.robot.startX;
        const P = (x, y) => [toX(x - sx), gy - y * s];
        ctx.strokeStyle = '#c9d1e0'; ctx.lineWidth = Math.max(2, s * 0.06); ctx.lineCap = 'round';
        for (const st of run.robot.sticks) {
          const b = st.body, p = b.getPosition(), a = b.getAngle(), c = Math.cos(a), sn = Math.sin(a);
          const [ax, ay] = P(p.x - c * st.half, p.y - sn * st.half), [bx, by] = P(p.x + c * st.half, p.y + sn * st.half);
          ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
        }
        ctx.fillStyle = '#ffb020';
        run.robot.servos.forEach((sv, k) => { const p = sv.joint.getAnchorA(), [x, y] = P(p.x, p.y); ctx.beginPath(); ctx.arc(x, y, k === 1 ? 4 : 3, 0, 7); ctx.fill(); });
        ctx.restore();
        const d = travel(run);
        ctx.textAlign = 'right'; ctx.fillStyle = d > 0.05 ? '#8ce99a' : d < -0.05 ? '#ff8787' : '#94a0b8';
        ctx.fillText(`${d >= 0 ? '' : '−'}${Math.abs(d).toFixed(2)} m`, x0 + w, top + laneH / 2);
      }
      ctx.restore();
      ctx.textBaseline = 'alphabetic';
    }
  }

  // ---------- words: what is going on ----------
  function explain() {
    const ga = L.method === 'ga';
    const ranked = L.scores ? [...L.scores.keys()].sort((a, b) => L.scores[b] - L.scores[a]) : null;
    const top = ranked && ranked[0], bottom = ranked && ranked[ranked.length - 1];
    switch (L.phase) {
      case 'intro': return {
        title: 'Meet the simplest learner',
        html: `<p>This little inchworm has two servos. <b>Servo 1</b> just swings back and forth to a steady beat. <b>Servo 2</b> is controlled by a brain made of only <b>two numbers</b>, A and B:</p>
          <p class="formula">servo 2 = A × beat + B × (beat, a quarter later)</p>
          <p>A and B decide how far servo 2 swings, and whether it moves just before or just after servo 1. Some pairs make the inchworm crawl forwards, some backwards, most do very little. Nobody knows which pairs are good.</p>
          <p>Can learning find good numbers on its own? Press <b>Start</b>, then keep pressing the green button to go one step at a time (or tick <i>Play it automatically</i>).</p>` };
      case 'make': return ga ? {
        title: `Generation 1: ten random brains`,
        html: `<p>Each robot gets its own two random numbers. On the map, <b>each dot is one robot's brain</b>: how far across is A, how far up is B. They start near the middle, where the numbers are small.</p><p>The lanes on the right show the same ten robots, ready to race.</p>` } : {
        title: 'One brain, five nudges',
        html: `<p>This method keeps just <b>one brain</b>: the ★ at ${fmtW(L.mean)}.</p><p>To find out which way to change it, it makes 5 random <b>nudges</b> and tries each one both ways, + and −. That's 10 robots, in pairs on opposite sides of the ★ (the faint lines join each pair).</p>` };
      case 'race': return {
        title: 'They all try',
        html: `<p>Each robot gets ${RACE_T} seconds to crawl as far as it can. Nobody tells it how: its two numbers make servo 2 move, and the robot goes wherever that takes it.</p><p>Watch the lanes: some creep forwards, some shuffle backwards.</p>` };
      case 'score': return {
        title: `Generation ${L.gen}: the scores`,
        html: `<p>Robot <b>${top + 1}</b> did best: ${metres(L.scores[top])} with brain ${fmtW(L.pop[top])}. Robot <b>${bottom + 1}</b> did worst: ${metres(L.scores[bottom])}.</p>
          <p>The dots are now coloured by score: green went forwards, red went backwards.</p>
          ${L.best && L.best.gen < L.gen ? `<p class="tip-dim">Best ever so far: ${metres(L.best.v)} in generation ${L.best.gen}.</p>` : ''}` };
      case 'choose': return ga ? {
        title: 'Keep the best',
        html: `<p>The ${ELITE} best brains survive (gold rings): robots ${L.elite.map(i => i + 1).join(', ')}.</p><p>The other ${N - ELITE} are thrown away. In this method, <b>only the winners pass anything on</b>, like breeding the fastest racehorses.</p>` } : {
        title: 'Compare each pair',
        html: `<p>For each nudge: did the + side or the − side go further? The small arrows point <b>from the loser to the winner</b> of each pair: that's a direction that helped.</p>
          <p>All five arrows are blended into one: the big gold arrow. Even the losers count, because they show which way <b>not</b> to go.</p>` };
      case 'next': return ga ? {
        title: `Generation ${L.gen}: the children`,
        html: `<p>The ${ELITE} winners stay as they are. Each of the ${N - ELITE} new robots copies a winner's two numbers and adds a small random change: a <b>mutation</b>. The arrows go from each parent to its child.</p>
          <p>Some mutations will help and some won't. The next race decides.</p>` } : {
        title: `Generation ${L.gen}: take a step`,
        html: `<p>The ★ moved along the gold arrow, to ${fmtW(L.mean)}. New nudges are made around it, and we go again.</p>
          <p>Notice it never jumps: it walks across the map, a step at a time, towards wherever robots did better.</p>` };
    }
    return { title: '', html: '' };
  }

  function refreshStats() {
    const el = document.getElementById('labStats');
    if (!el) return;
    const mapDone = L.mapNext >= GRID * GRID;
    const parts = [];
    if (L.gen) parts.push(`Generation ${L.gen}. Robots tried: ${L.tried.length}.`);
    if (L.best) parts.push(`Best so far: ${metres(L.best.v)} with ${fmtW(L.best.w)}.`);
    if (mapDone && L.mapMax) parts.push(`The best brain on the whole map crawls ${metres(L.mapMax.v)} (at about ${fmtW(L.mapMax.w)}). We only know that because we tried all ${GRID * GRID} brains; learning has to find it without that.`);
    else parts.push(`Drawing the map of every brain in the background: ${Math.round(100 * L.mapNext / (GRID * GRID))}%.`);
    el.textContent = parts.join(' ');
  }

  function refresh() {
    const t = document.getElementById('labText');
    if (!t) return;
    const ex = explain();
    t.innerHTML = `<h3>${ex.title}</h3>${ex.html}`;
    const next = document.getElementById('labNext');
    next.textContent = { intro: 'Start ▶', make: 'Race them ▶', next: 'Race them ▶', race: 'Skip to the end ▶', score: L.method === 'ga' ? 'Pick the winners ▶' : 'Compare the pairs ▶', choose: L.method === 'ga' ? 'Make the next generation ▶' : 'Take the step ▶' }[L.phase] || 'Next step ▶';
    document.querySelectorAll('#labMethod button').forEach(b => b.classList.toggle('on', b.dataset.method === L.method));
    document.querySelectorAll('#labSpeed button').forEach(b => b.classList.toggle('on', +b.dataset.speed === L.speed));
    document.getElementById('labMap').checked = L.showMap;
    document.getElementById('labAuto').checked = L.auto;
    refreshStats();
  }

  RW.Lab.init = function () {
    document.getElementById('labNext').addEventListener('click', advance);
    document.getElementById('labAuto').addEventListener('change', e => { L.auto = e.target.checked; L.holdT = 0; if (L.auto && L.phase === 'intro') advance(); });
    document.getElementById('labMap').addEventListener('change', e => { L.showMap = e.target.checked; });
    document.getElementById('labReset').addEventListener('click', reset);
    document.querySelectorAll('#labSpeed button').forEach(b => b.addEventListener('click', () => { L.speed = +b.dataset.speed; refresh(); }));
    document.querySelectorAll('#labMethod button').forEach(b => b.addEventListener('click', () => {
      if (L.method === b.dataset.method) return;
      L.method = b.dataset.method; reset();
    }));
    refresh();
  };
  RW.Lab.enter = refresh;
})(window.RW = window.RW || {});
