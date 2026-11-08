// The robot's brain (a tiny neural network) and how it learns (evolution).
//
// Genome = [frequency gene, hidden-layer weights, output-layer weights].
// Each generation every genome drives its own copy of the robot in its own
// world; the ones that get furthest become the parents of the next generation.
(function (RW) {
  'use strict';
  const HIDDEN = 8;

  function gauss() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  RW.genomeLength = function (info) {
    const nIn = RW.inputCount(info);
    return 1 + HIDDEN * nIn + info.servos * (HIDDEN + 1);
  };

  RW.randomGenome = function (info) {
    const g = new Float32Array(RW.genomeLength(info));
    for (let i = 0; i < g.length; i++) g[i] = gauss() * 0.6;
    return g;
  };

  class Brain {
    constructor(genome, info) {
      this.g = genome;
      this.nIn = RW.inputCount(info);
      this.nOut = info.servos;
      this.freq = 0.3 + 1.7 / (1 + Math.exp(-genome[0])); // 0.3 .. 2 Hz
      this.inBuf = new Float32Array(this.nIn);
      this.h = new Float32Array(HIDDEN);
      this.out = new Float32Array(this.nOut);
    }
    forward(x) {
      const g = this.g, nIn = this.nIn;
      let k = 1;
      for (let j = 0; j < HIDDEN; j++) {
        let s = 0;
        for (let i = 0; i < nIn; i++) s += g[k++] * x[i];
        this.h[j] = Math.tanh(s);
      }
      for (let o = 0; o < this.nOut; o++) {
        let s = g[k++]; // bias
        for (let j = 0; j < HIDDEN; j++) s += g[k++] * this.h[j];
        this.out[o] = Math.tanh(s);
      }
      return this.out;
    }
  }
  RW.Brain = Brain;

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
      this.generation = 0;
      this.history = [];              // [{best, avg, finished}]
      this.bestEver = null;           // {genome, fitness, distance, time}
      this.stall = 0;
      this.pop = [];
      if (opts.seed) this.pop.push(Float32Array.from(opts.seed.genome)), this.bestEver = opts.seed;
      while (this.pop.length < this.popSize) this.pop.push(RW.randomGenome(this.info));
      this.startGeneration();
    }

    startGeneration() {
      this.t = 0;
      this.agents = this.pop.map(genome => {
        const world = RW.makeWorld();
        const robot = RW.buildRobot(world, this.design, this.strength);
        return { world, robot, brain: new Brain(genome, this.info), genome, done: false, finishTime: null, dist: 0 };
      });
    }

    fitnessOf(a) {
      if (a.finishTime != null) return RW.FINISH + 2 * (this.timeLimit - a.finishTime);
      return a.dist;
    }

    // Advance every robot one tick. Returns true when the generation ended.
    step() {
      for (const a of this.agents) {
        if (a.done) continue;
        RW.stepRobot(a.world, a.robot, a.brain);
        a.dist = RW.comX(a.robot) - a.robot.startX;
        if (!isFinite(a.dist)) { a.dist = 0; a.done = true; }
        else if (a.dist >= RW.FINISH) { a.done = true; a.finishTime = a.robot.t; }
      }
      this.t += RW.DT;
      if (this.t >= this.timeLimit || this.agents.every(a => a.done)) { this.endGeneration(); return true; }
      return false;
    }

    leader() {
      let best = null;
      for (const a of this.agents) if (!best || this.fitnessOf(a) > this.fitnessOf(best)) best = a;
      return best;
    }

    endGeneration() {
      const fit = this.agents.map(a => this.fitnessOf(a));
      let bi = 0;
      for (let i = 1; i < fit.length; i++) if (fit[i] > fit[bi]) bi = i;
      const avg = fit.reduce((s, f) => s + f, 0) / fit.length;
      const finished = this.agents.filter(a => a.finishTime != null).length;
      this.history.push({ best: this.agents[bi].dist, fit: fit[bi], avg: this.agents.reduce((s, a) => s + a.dist, 0) / fit.length, finished });
      if (!this.bestEver || fit[bi] > this.bestEver.fitness + 1e-6) {
        const a = this.agents[bi];
        this.bestEver = { genome: Float32Array.from(a.genome), fitness: fit[bi], distance: a.dist, time: a.finishTime, generation: this.generation + 1 };
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
