// The robot's brain (a tiny neural network) and how it learns (evolution).
//
// Genome = [frequency gene, hidden-layer weights, output-layer weights, stiffness].
// The brain's size (hidden units) and whether it has memory come from the
// design (design.brain), so changing them gives the robot a fresh brain.
// With memory, each hidden unit also sees what every hidden unit was doing a
// tick ago (a recurrent, "Elman" network), so the brain can carry information
// forward in time: "I just tripped", "the front leg is on its way back".
// Each generation every genome drives its own copy of the robot in its own
// world; the ones that get furthest become the parents of the next generation.
(function (RW) {
  'use strict';
  const HIDDEN = 8; // default size

  function gauss() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  // Inputs the network sees: the sensors, plus (with memory) the hidden units' last values.
  RW.netInputCount = info => RW.inputCount(info) + (info.memory ? info.hidden : 0);
  RW.genomeLength = function (info) {
    const nIn = RW.netInputCount(info), nH = info.hidden;
    return 1 + nH * nIn + info.servos * (nH + 1) + info.servos; // + one stiffness gene per servo
  };

  // A new random brain. Extra capacity starts out quiet, so a bigger brain or
  // one with memory begins by behaving much like the small one, and evolution
  // can grow into the extra room instead of starting from chaos:
  //  - memory connections start near zero (otherwise the loop starts jittery)
  //  - output weights shrink as the hidden layer grows (otherwise more units
  //    add up to pinned, all-or-nothing outputs)
  RW.randomGenome = function (info) {
    const g = new Float32Array(RW.genomeLength(info));
    for (let i = 0; i < g.length; i++) g[i] = gauss() * 0.6;
    const nIn = RW.netInputCount(info), nSense = RW.inputCount(info), nH = info.hidden;
    if (info.memory) for (let j = 0; j < nH; j++) for (let i = nSense; i < nIn; i++) g[1 + j * nIn + i] = gauss() * 0.05;
    const scale = Math.sqrt(8 / nH), oStart = 1 + nH * nIn;
    for (let o = 0; o < info.servos; o++) for (let j = 0; j < nH; j++) g[oStart + o * (nH + 1) + 1 + j] *= scale;
    return g;
  };

  class Brain {
    constructor(genome, info) {
      this.g = genome;
      this.nH = info.hidden;
      this.nSense = RW.inputCount(info);
      this.memory = !!info.memory;
      this.nIn = RW.netInputCount(info);
      this.nOut = info.servos;
      this.freq = 0.3 + 1.7 / (1 + Math.exp(-genome[0])); // 0.3 .. 2 Hz
      this.inBuf = new Float32Array(this.nIn);
      this.h = new Float32Array(this.nH);
      this.out = new Float32Array(this.nOut);
      // The last genes set each servo's stiffness (how hard it holds its target).
      const g0 = genome.length - this.nOut;
      this.gain = new Float32Array(this.nOut);
      for (let i = 0; i < this.nOut; i++) this.gain[i] = RW.GAIN_MIN + (RW.GAIN_MAX - RW.GAIN_MIN) / (1 + Math.exp(-genome[g0 + i]));
    }
    forward(x) {
      const g = this.g, nIn = this.nIn, nH = this.nH;
      // memory: the hidden units' values from last tick become extra inputs
      if (this.memory) for (let j = 0; j < nH; j++) x[this.nSense + j] = this.h[j];
      let k = 1;
      for (let j = 0; j < nH; j++) {
        let s = 0;
        for (let i = 0; i < nIn; i++) s += g[k++] * x[i];
        this.h[j] = Math.tanh(s);
      }
      for (let o = 0; o < this.nOut; o++) {
        let s = g[k++]; // bias
        for (let j = 0; j < nH; j++) s += g[k++] * this.h[j];
        this.out[o] = Math.tanh(s);
      }
      return this.out;
    }
  }
  RW.Brain = Brain;
  RW.HIDDEN = HIDDEN;

  // Genetic algorithm: keep the best few, breed the rest from tournament winners.
  RW.evolve = function (pop, fitness, opts) {
    const n = pop.length;
    const order = [...pop.keys()].sort((a, b) => fitness[b] - fitness[a]);
    const elite = Math.max(1, Math.round(n * 0.06));
    const sigma = opts.sigma, rate = opts.rate;
    const pick = () => {
      let best = -1;
      for (let i = 0; i < 3; i++) { const c = (Math.random() * n) | 0; if (best < 0 || fitness[c] > fitness[best]) best = c; }
      return pop[best];
    };
    const next = [];
    for (let i = 0; i < elite; i++) next.push(Float32Array.from(pop[order[i]]));
    while (next.length < n) {
      const a = pick(), b = pick();
      const child = new Float32Array(a.length);
      const cross = Math.random() < 0.6;
      for (let i = 0; i < a.length; i++) {
        let v = cross && Math.random() < 0.5 ? b[i] : a[i];
        if (Math.random() < rate) v += gauss() * sigma;
        child[i] = v;
      }
      next.push(child);
    }
    return next;
  };

  // Runs one generation at a time: N worlds, N robots, N brains.
  class Trainer {
    constructor(design, opts) {
      this.design = design;
      this.info = RW.analyse(design);
      this.popSize = opts.popSize || 40;
      this.timeLimit = opts.timeLimit || 15;
      this.strength = opts.strength || 1;
      this.rules = opts.rules || RW.DEFAULT_RULES;
      this.course = opts.course || null;
      this.generation = 0;
      this.history = [];              // [{best, fit, avg, finished, fell, level}]
      this.bestEver = null;           // {genome, fitness, distance, time, level, fallen, generation}
      this.stall = 0;
      this.pop = [];
      if (opts.seed) this.pop.push(Float32Array.from(opts.seed.genome)), this.bestEver = opts.seed;
      while (this.pop.length < this.popSize) this.pop.push(RW.randomGenome(this.info));
      this.startGeneration();
    }

    startGeneration() {
      this.t = 0;
      this.agents = this.pop.map(genome => {
        const run = new RW.Run(this.design, new Brain(genome, this.info), { strength: this.strength, rules: this.rules, course: this.course });
        run.genome = genome;
        return run;
      });
    }

    fitnessOf(a) { return a.fitness(this.timeLimit); }

    // Advance every robot one tick. Returns true when the generation ended.
    step() {
      for (const a of this.agents) if (!a.done) a.step();
      this.t += RW.DT;
      if (this.t >= this.timeLimit || this.agents.every(a => a.done)) { this.endGeneration(); return true; }
      return false;
    }

    leader() {
      let best = null, bf = -Infinity;
      for (const a of this.agents) { const f = this.fitnessOf(a); if (f > bf) { bf = f; best = a; } }
      return best || this.agents[0];
    }

    endGeneration() {
      const fit = this.agents.map(a => this.fitnessOf(a));
      let bi = 0;
      for (let i = 1; i < fit.length; i++) if (fit[i] > fit[bi]) bi = i;
      const n = this.agents.length, b = this.agents[bi];
      this.history.push({
        best: b.dist, fit: fit[bi], level: b.level,
        avg: this.agents.reduce((s, a) => s + a.dist, 0) / n,
        finished: this.agents.filter(a => a.finishTime != null).length,
        fell: this.agents.filter(a => a.fallen).length,
      });
      if (!this.bestEver || fit[bi] > this.bestEver.fitness + 1e-6) {
        this.bestEver = { genome: Float32Array.from(b.genome), fitness: fit[bi], distance: b.dist, time: b.finishTime, level: b.level, fallen: b.fallen, generation: this.generation + 1 };
        this.stall = 0;
      } else this.stall++;
      // Stuck for a while? Mutate harder to shake things up.
      const sigma = this.stall > 8 ? 0.6 : 0.25;
      this.pop = RW.evolve(this.pop, fit, { sigma, rate: 0.1 });
      this.generation++;
      this.startGeneration();
    }
  }
  RW.Trainer = Trainer;
})(window.RW = window.RW || {});
