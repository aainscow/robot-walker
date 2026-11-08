# Robot Walker

A 2D game: build a robot from sticks, servos and an accelerometer, then let it
learn by evolution to walk to the finish line 20 m away.

Open `index.html` in a browser (it works straight from the file), or serve the
folder: `python3 -m http.server 8107`.

## How it plays

1. **Build**: drag to draw sticks. Where sticks meet is a free hinge; click it
   with the Servo tool to motorise it, or Weld to make it rigid. A servo swings
   ±90° from how you built it; click it again for ±135°, ±180°, then continuous
   rotation (it spins all the way round and the brain sets its speed), then
   once more to remove it. Each servo's stiffness (how hard it holds its
   target) is learned along with the brain. Click a stick
   with the Accelerometer tool to fit a sensor, and put the **Head** on a point.
   The **Camera** tool fits a time-of-flight camera to the head: 9 rays fanned
   from 20° above to 60° below the head's forward direction, each measuring the
   distance (up to 3 m) to the ground or an obstacle, 20 times a second. The
   brain gets those distances as extra inputs.
   The **Foot** tool fits a rubber foot to a point. Bare sticks are hard
   plastic and slide (friction 0.4); rubber feet grip (2.0) and bounce a
   little. On a 35° slope the presets slide 3-4 m without feet and not at all
   with them.
   A servo whose sticks are braced into a rigid shape (a triangle) can't bend,
   so the editor test-swings every servo and rings any that can't move in red.
   Or start from a preset.
2. **Course**: drag on the track to add blocks, ramps (drag from the foot to
   the top) and bumps; Move slides one along, Erase removes it. The green start
   area stays clear. Or pick a course: Flat, Bumpy, Hurdles, Stairs, Hill.
   Changing the course keeps the brain, so you can train on flat ground first
   and then make it harder.
3. **Train**: every generation, 40 copies of the robot each get their own brain
   and try for 15 s. The furthest become the parents of the next generation.
   **Rules:** the head must stay up. If it touches the ground or bumps into an
   obstacle the robot has fallen over: its run ends and its distance counts half.
   Its score is also scaled by how level it kept its head ("Don't care",
   "A bit" or "Strictly"). Level means the angle you built the head at.
   **Brain** settings choose the network: 4, 8 (default), 16 or 24 hidden
   units, and optional memory (each hidden unit also sees what all of them
   did a tick ago, so it can carry information through time). New brains
   start their extra capacity quiet, so they begin like the small one. In
   tests (dog on Hurdles, walker on flat, 3 runs each, 40 generations) the
   8-unit brain without memory still learned best: more numbers to tune
   means slower evolution.
4. **Watch best**: replay the best brain so far in real time, with the
   accelerometer's reading drawn as a green arrow.
   "Run for" lets the replay carry on past the finish line and the training
   time limit (30 s, 1 or 3 minutes, or as long as it can) on flat ground
   that goes on for 500 m. It stops early if the robot falls or makes no
   progress for 10 s.
   **Show the brain working** draws the robot's neural network live under the
   replay: every input with its current value (clock, joint angles,
   accelerometer, camera), the 8 hidden units, and each servo's target next to
   where it really is. Connections glow with the signal they carry right now
   (orange pushes up, blue pushes down). Hover anything in it (an input, a
   hidden unit H1-H8, or a servo output) for a plain-words explanation of what
   it is and what its value means right now, with what is pushing it hardest;
   click to keep it highlighted, or click a servo on the robot. The ❚❚ button
   pauses the replay so you can look at one moment, and "How does the brain
   work?" walks through it step by step.

The design and the best brain are saved in the browser's localStorage.

## Code

- `js/sim.js`: design → planck.js bodies and joints; servo control; accelerometer.
- `js/brain.js`: the neural network (clock + servo angles + accelerometer in,
  servo targets out) and the genetic algorithm (`Trainer`).
- `js/presets.js`: starter robots.
- `js/game.js`: editor, training loop, replay, drawing.
- `vendor/planck.min.js`: planck.js 1.5.1 (MIT), a Box2D port.

Courses are `{ obstacles: [{type: 'block'|'ramp'|'bump', x, w, h, dir}] }` in
metres from the start line; `RW.obstaclePoly` turns each into a convex polygon.

## Licence

MIT, see [LICENSE](LICENSE). planck.js (in `vendor/`) is also MIT, see
[vendor/planck-LICENSE](vendor/planck-LICENSE).
