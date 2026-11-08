// Physics side of the game: turns a robot design into planck.js bodies and
// joints, reads the accelerometers, and drives the servos from a brain.
//
// A design is plain data, in metres, y up, ground at y = 0:
//   { points: [{id, x, y, kind}], sticks: [{id, a, b, accel}] }
//   kind: 'free' (unpowered hinge) | 'servo' (motorised hinge) | 'weld' (rigid)
(function (RW) {
  'use strict';
  const pl = planck;

  RW.GRAVITY = 10;
  RW.DT = 1 / 60;
  RW.FINISH = 20;            // metres to the finish line
  RW.STICK_R = 0.04;         // half thickness of a stick
  RW.FOOT_R = 0.05;          // round ends so sticks roll over the ground
  RW.HEAD_R = 0.13;          // radius of the head
  RW.SERVO_RANGE = 1.0;      // +/- radians from the built pose (~57 degrees)
  RW.SERVO_MAX_SPEED = 7;    // rad/s
  // Time-of-flight camera on the head: a fan of distance-measuring rays.
  RW.CAM_RAYS = 9;
  RW.CAM_RANGE = 3;          // metres
  RW.CAM_UP = 20 * Math.PI / 180;    // top ray, above the head's forward direction
  RW.CAM_DOWN = -60 * Math.PI / 180; // bottom ray, looking down at the ground ahead
  RW.camRayAngle = i => RW.CAM_UP + (RW.CAM_DOWN - RW.CAM_UP) * i / (RW.CAM_RAYS - 1);

  RW.START_ZONE = 2.5;       // obstacles can't go before this (the robot starts here)
  RW.COURSE_END = RW.FINISH + 3;

  // A course is { obstacles: [{id, type, x, w, h, dir}] }, all sitting on the ground:
  //   block: a box from x to x+w, h tall
  //   ramp:  a slope over x..x+w rising to h (dir 1 = up to the right, -1 = down)
  //   bump:  a rounded hill x..x+w, h tall
  RW.obstaclePoly = function (o) {
    const { x, w, h } = o;
    if (o.type === 'ramp') return o.dir < 0 ? [[x, 0], [x + w, 0], [x, h]] : [[x, 0], [x + w, 0], [x + w, h]];
    if (o.type === 'bump') {
      const pts = [];
      for (let i = 0; i <= 8; i++) { const a = Math.PI * (1 - i / 8); pts.push([x + w / 2 + Math.cos(a) * w / 2, Math.sin(a) * h]); }
      return pts;
    }
    return [[x, 0], [x + w, 0], [x + w, h], [x, h]];
  };

  // The world: ground, end walls, and the course's obstacles.
  RW.makeWorld = function (course) {
    const world = new pl.World({ gravity: new pl.Vec2(0, -RW.GRAVITY) });
    const ground = world.createBody();
    ground.createFixture(new pl.EdgeShape(new pl.Vec2(-20, 0), new pl.Vec2(RW.FINISH + 40, 0)), { friction: 1.0 });
    // Walls so nothing falls off the ends of the world
    ground.createFixture(new pl.EdgeShape(new pl.Vec2(-20, 0), new pl.Vec2(-20, 30)), {});
    ground.createFixture(new pl.EdgeShape(new pl.Vec2(RW.FINISH + 40, 0), new pl.Vec2(RW.FINISH + 40, 30)), {});
    for (const ob of (course && course.obstacles) || []) {
      const poly = RW.obstaclePoly(ob);
      ground.createFixture(new pl.PolygonShape(poly.map(p => new pl.Vec2(p[0], p[1]))), { friction: 1.0 });
    }
    return world;
  };

  // Which sticks meet at each point, in creation order (the first is the "base"
  // that the other sticks hinge against).
  RW.pointSticks = function (design) {
    const m = new Map();
    for (const p of design.points) m.set(p.id, []);
    for (const s of design.sticks) { m.get(s.a).push(s); m.get(s.b).push(s); }
    return m;
  };

  // How many servo channels / accelerometers a design has (sets the brain's size).
  RW.analyse = function (design) {
    const ps = RW.pointSticks(design);
    let servos = 0;
    for (const p of design.points) {
      const n = ps.get(p.id).length;
      if (p.kind === 'servo' && n >= 2) servos += n - 1;
    }
    const accels = design.sticks.filter(s => s.accel).length;
    const hasHead = design.head && design.sticks.some(s => s.a === design.head || s.b === design.head);
    const camera = design.camera && hasHead ? RW.CAM_RAYS : 0;
    return { servos, accels, camera };
  };

  // Shift the design so its lowest point sits on the ground and its left edge on the start line.
  function placement(design) {
    let minX = Infinity, minY = Infinity;
    for (const p of design.points) { minX = Math.min(minX, p.x); minY = Math.min(minY, p.y); }
    return { dx: -minX, dy: RW.FOOT_R + 0.01 - minY };
  }

  RW.buildRobot = function (world, design, strength) {
    const { dx, dy } = placement(design);
    const pts = new Map(design.points.map(p => [p.id, { x: p.x + dx, y: p.y + dy, kind: p.kind }]));
    const robot = { sticks: [], servos: [], hinges: [], welds: [], accels: [], mass: 0, t: 0 };
    const bodyOf = new Map();

    for (const s of design.sticks) {
      const a = pts.get(s.a), b = pts.get(s.b);
      const len = Math.hypot(b.x - a.x, b.y - a.y);
      const ang = Math.atan2(b.y - a.y, b.x - a.x);
      const body = world.createDynamicBody({ position: new pl.Vec2((a.x + b.x) / 2, (a.y + b.y) / 2), angle: ang, angularDamping: 0.5 });
      const fd = { density: 4, friction: 1.0, filterGroupIndex: -1 }; // -1: a robot never collides with itself
      body.createFixture(new pl.BoxShape(len / 2, RW.STICK_R), fd);
      body.createFixture(new pl.CircleShape(new pl.Vec2(-len / 2, 0), RW.FOOT_R), fd);
      body.createFixture(new pl.CircleShape(new pl.Vec2(len / 2, 0), RW.FOOT_R), fd);
      robot.mass += body.getMass();
      const st = { body, half: len / 2, accel: !!s.accel, id: s.id };
      robot.sticks.push(st);
      bodyOf.set(s.id, body);
      if (s.accel) robot.accels.push({ body, pv: null, f: { x: 0, y: RW.GRAVITY }, raw: { x: 0, y: RW.GRAVITY } });
    }

    // The head: a ball fixed to the end of the first stick at its point. Its
    // "level" is however it was built; tilt is measured from that.
    if (design.head && pts.has(design.head)) {
      const stick = design.sticks.find(s => s.a === design.head || s.b === design.head);
      if (stick) {
        const body = bodyOf.get(stick.id), P = pts.get(design.head);
        const local = body.getLocalPoint(new pl.Vec2(P.x, P.y));
        const fixture = body.createFixture(new pl.CircleShape(local, RW.HEAD_R), { density: 1.5, friction: 0.6, filterGroupIndex: -1 });
        robot.mass += fixture.getDensity() * Math.PI * RW.HEAD_R * RW.HEAD_R;
        robot.head = { body, local, fixture, angle0: body.getAngle() };
        if (design.camera) robot.camera = { depth: new Float32Array(RW.CAM_RAYS).fill(1), hits: [] };
      }
    }

    // Size of the robot sets how much torque its servos have.
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const p of pts.values()) { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y); }
    const size = Math.max(0.3, Math.max(maxX - minX, maxY - minY));
    const torque = 0.6 * robot.mass * RW.GRAVITY * size * (strength || 1);

    const ps = RW.pointSticks(design);
    for (const p of design.points) {
      const list = ps.get(p.id);
      if (list.length < 2) continue;
      const P = pts.get(p.id), anchor = new pl.Vec2(P.x, P.y);
      const base = bodyOf.get(list[0].id);
      for (let i = 1; i < list.length; i++) {
        const other = bodyOf.get(list[i].id);
        if (p.kind === 'weld') {
          robot.welds.push({ joint: world.createJoint(new pl.WeldJoint({ frequencyHz: 0, dampingRatio: 0 }, base, other, anchor)) });
        } else if (p.kind === 'servo') {
          const j = world.createJoint(new pl.RevoluteJoint({
            enableMotor: true, maxMotorTorque: torque, motorSpeed: 0,
            enableLimit: true, lowerAngle: -RW.SERVO_RANGE, upperAngle: RW.SERVO_RANGE,
          }, base, other, anchor));
          robot.servos.push({ joint: j, target: 0, pointId: p.id });
        } else {
          // free hinge, with a little friction so it doesn't flop forever
          const j = world.createJoint(new pl.RevoluteJoint({ enableMotor: true, maxMotorTorque: torque * 0.02, motorSpeed: 0 }, base, other, anchor));
          robot.hinges.push({ joint: j, pointId: p.id });
        }
      }
    }
    robot.startX = RW.comX(robot);
    return robot;
  };

  // How the head is doing: tilt from its built pose (radians, -pi..pi), and
  // whether it is touching the ground (which counts as falling over).
  RW.headState = function (robot) {
    const h = robot.head;
    if (!h) return null;
    let tilt = h.body.getAngle() - h.angle0;
    tilt = Math.atan2(Math.sin(tilt), Math.cos(tilt));
    let touching = false;
    for (let ce = h.body.getContactList(); ce; ce = ce.next) {
      const c = ce.contact;
      if (c.isTouching() && (c.getFixtureA() === h.fixture || c.getFixtureB() === h.fixture)) { touching = true; break; }
    }
    return { tilt, touching };
  };

  RW.headPos = function (robot) {
    return robot.head && robot.head.body.getWorldPoint(robot.head.local);
  };

  // One attempt by one brain: steps the robot and keeps score.
  //   rules.fall:  touching the ground with the head (or turning it upside
  //                down) ends the run and halves the distance
  //   rules.level: 0 = don't care, higher = stricter about keeping the head level
  class Run {
    constructor(design, brain, opts) {
      this.world = RW.makeWorld(opts.course);
      this.robot = RW.buildRobot(this.world, design, opts.strength);
      this.brain = brain;
      this.rules = opts.rules || RW.DEFAULT_RULES;
      this.dist = 0; this.levelSum = 0; this.ticks = 0;
      this.done = false; this.fallen = false; this.finishTime = null; this.tilt = 0;
    }
    get level() { return this.ticks ? this.levelSum / this.ticks : 1; }
    step() {
      RW.stepRobot(this.world, this.robot, this.brain);
      this.ticks++;
      this.dist = RW.comX(this.robot); // from the start line at x = 0
      if (!isFinite(this.dist)) { this.dist = 0; this.done = this.fallen = true; return; }
      const hs = RW.headState(this.robot);
      if (hs) {
        this.tilt = hs.tilt;
        this.levelSum += Math.pow(Math.max(0, Math.cos(hs.tilt)), this.rules.level);
        if (this.rules.fall && (hs.touching || Math.abs(hs.tilt) > Math.PI / 2)) { this.done = this.fallen = true; return; }
      } else this.levelSum += 1;
      if (this.dist >= RW.FINISH) { this.done = true; this.finishTime = this.robot.t; }
    }
    fitness(timeLimit) {
      let f = this.finishTime != null ? RW.FINISH + 2 * (timeLimit - this.finishTime) : this.dist;
      if (this.fallen) f = Math.min(f, this.dist) * 0.5;
      return f * (f > 0 ? this.level : 1);
    }
  }
  RW.Run = Run;
  RW.DEFAULT_RULES = { fall: true, level: 2 };

  // Which servos can't actually move? A servo whose sticks are braced into a
  // rigid shape (a triangle, say) is locked solid by the other sticks. Test it
  // the honest way: in a weightless world with nothing to touch, try to swing
  // each servo on its own and see whether it moves.
  RW.findLockedServos = function (design) {
    const locked = [];
    const n = RW.analyse(design).servos;
    for (let k = 0; k < n; k++) {
      const world = new pl.World({ gravity: new pl.Vec2(0, 0) });
      const robot = RW.buildRobot(world, design, 1);
      let lo = 0, hi = 0;
      for (let i = 0; i < 90; i++) {
        robot.servos.forEach((s, j) => {
          const target = j === k ? (i < 45 ? 0.6 : -0.6) : 0;
          s.joint.setMotorSpeed(Math.max(-RW.SERVO_MAX_SPEED, Math.min(RW.SERVO_MAX_SPEED, 12 * (target - s.joint.getJointAngle()))));
        });
        world.step(RW.DT, 8, 3);
        const a = robot.servos[k].joint.getJointAngle();
        lo = Math.min(lo, a); hi = Math.max(hi, a);
      }
      if (hi - lo < 0.15) locked.push({ index: k, pointId: robot.servos[k].pointId });
    }
    return locked;
  };

  RW.comX = function (robot) {
    let sx = 0, sm = 0;
    for (const s of robot.sticks) { const m = s.body.getMass(); sx += s.body.getWorldCenter().x * m; sm += m; }
    return sx / sm;
  };

  // Accelerometer: measures proper acceleration (gravity included) in the
  // stick's own frame, like a real MEMS chip. At rest it reads 10 m/s^2 "up"
  // relative to the stick, so it tells the brain which way is down.
  function readAccels(robot) {
    for (const a of robot.accels) {
      const v = a.body.getLinearVelocity();
      let ax = 0, ay = 0;
      if (a.pv) { ax = (v.x - a.pv.x) / RW.DT; ay = (v.y - a.pv.y) / RW.DT; }
      a.pv = { x: v.x, y: v.y };
      ay += RW.GRAVITY; // proper acceleration = a - g
      const th = a.body.getAngle(), c = Math.cos(th), s = Math.sin(th);
      a.raw = { x: c * ax + s * ay, y: -s * ax + c * ay };
      a.f.x += 0.2 * (a.raw.x - a.f.x);
      a.f.y += 0.2 * (a.raw.y - a.f.y);
    }
  }

  // Time-of-flight camera: each ray measures how far it is to the ground or an
  // obstacle (the robot can't see itself). depth[i] is that distance as a
  // fraction of the range, 1 meaning "nothing within range".
  function readCamera(world, robot) {
    const cam = robot.camera;
    if (!cam) return;
    const h = robot.head, c = h.body.getWorldPoint(h.local);
    const look = h.body.getAngle() - h.angle0;
    for (let i = 0; i < RW.CAM_RAYS; i++) {
      const a = look + RW.camRayAngle(i), dx = Math.cos(a), dy = Math.sin(a);
      const p1 = new planck.Vec2(c.x + dx * RW.HEAD_R, c.y + dy * RW.HEAD_R);
      const p2 = new planck.Vec2(p1.x + dx * RW.CAM_RANGE, p1.y + dy * RW.CAM_RANGE);
      let best = 1;
      world.rayCast(p1, p2, (fixture, point, normal, fraction) => {
        if (!fixture.getBody().isStatic()) return -1; // ignore the robot's own parts
        best = Math.min(best, fraction);
        return fraction;
      });
      cam.depth[i] = best;
      cam.hits[i] = { x0: p1.x, y0: p1.y, x1: p1.x + (p2.x - p1.x) * best, y1: p1.y + (p2.y - p1.y) * best, hit: best < 1 };
    }
  }

  // Inputs to the brain: a clock (so it can learn a rhythm), where each servo
  // is, what each accelerometer feels, and what the camera sees.
  RW.inputCount = function (info) { return 5 + info.servos + 2 * info.accels + (info.camera || 0); };

  function gatherInputs(robot, phase, out) {
    let k = 0;
    out[k++] = 1;
    out[k++] = Math.sin(phase); out[k++] = Math.cos(phase);
    out[k++] = Math.sin(2 * phase); out[k++] = Math.cos(2 * phase);
    for (const s of robot.servos) out[k++] = s.joint.getJointAngle() / RW.SERVO_RANGE;
    for (const a of robot.accels) {
      out[k++] = Math.max(-2, Math.min(2, a.f.x / RW.GRAVITY));
      out[k++] = Math.max(-2, Math.min(2, a.f.y / RW.GRAVITY));
    }
    if (robot.camera) for (let i = 0; i < RW.CAM_RAYS; i++) out[k++] = 1 - robot.camera.depth[i]; // near = big
    return out;
  }

  // One physics tick: think, set the servos, step the world.
  RW.stepRobot = function (world, robot, brain) {
    readAccels(robot);
    // the camera takes a frame every 3rd tick (20 per second), like a real ToF sensor
    if (robot.camera && (robot.camera.tick = (robot.camera.tick || 0) + 1) % 3 === 1) readCamera(world, robot);
    const phase = 2 * Math.PI * brain.freq * robot.t;
    const outs = brain.forward(gatherInputs(robot, phase, brain.inBuf));
    for (let i = 0; i < robot.servos.length; i++) {
      const s = robot.servos[i];
      s.target = outs[i] * RW.SERVO_RANGE;
      const err = s.target - s.joint.getJointAngle();
      s.joint.setMotorSpeed(Math.max(-RW.SERVO_MAX_SPEED, Math.min(RW.SERVO_MAX_SPEED, 12 * err)));
    }
    world.step(RW.DT, 8, 3);
    robot.t += RW.DT;
  };
})(window.RW = window.RW || {});
