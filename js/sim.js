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
  RW.SERVO_RANGE = 1.0;      // +/- radians from the built pose (~57 degrees)
  RW.SERVO_MAX_SPEED = 7;    // rad/s

  // The world: flat ground for now. Obstacles go in `level.obstacles` later.
  RW.makeWorld = function (level) {
    const world = new pl.World({ gravity: new pl.Vec2(0, -RW.GRAVITY) });
    const ground = world.createBody();
    ground.createFixture(new pl.EdgeShape(new pl.Vec2(-20, 0), new pl.Vec2(RW.FINISH + 40, 0)), { friction: 1.0 });
    // Walls so nothing falls off the ends of the world
    ground.createFixture(new pl.EdgeShape(new pl.Vec2(-20, 0), new pl.Vec2(-20, 30)), {});
    ground.createFixture(new pl.EdgeShape(new pl.Vec2(RW.FINISH + 40, 0), new pl.Vec2(RW.FINISH + 40, 30)), {});
    for (const ob of (level && level.obstacles) || []) {
      ground.createFixture(new pl.PolygonShape(ob.map(p => new pl.Vec2(p[0], p[1]))), { friction: 1.0 });
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
    return { servos, accels };
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

  // Inputs to the brain: a clock (so it can learn a rhythm), where each servo
  // is, and what each accelerometer feels.
  RW.inputCount = function (info) { return 5 + info.servos + 2 * info.accels; };

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
    return out;
  }

  // One physics tick: think, set the servos, step the world.
  RW.stepRobot = function (world, robot, brain) {
    readAccels(robot);
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
