# Robot Walker

A 2D game: build a robot from sticks, servos and an accelerometer, then let it
learn by evolution to walk to the finish line 20 m away.

Open `index.html` in a browser (it works straight from the file), or serve the
folder: `python3 -m http.server 8107`.

## How it plays

1. **Build**: drag to draw sticks. Where sticks meet is a free hinge; click it
   with the Servo tool to motorise it, or Weld to make it rigid. Click a stick
   with the Accelerometer tool to fit a sensor, and put the **Head** on a point.
   Or start from a preset.
2. **Course**: drag on the track to add blocks, ramps (drag from the foot to
   the top) and bumps; Move slides one along, Erase removes it. The green start
   area stays clear. Or pick a course: Flat, Bumpy, Hurdles, Stairs, Hill.
   Changing the course keeps the brain, so you can train on flat ground first
   and then make it harder.
3. **Train**: every generation, 40 copies of the robot each get their own brain
   and try for 15 s. The furthest become the parents of the next generation.
   **Rules:** the head must stay up. If it touches the ground (or turns upside
   down, or bumps into an obstacle) the robot has fallen over: its run ends and its distance counts half.
   Its score is also scaled by how level it kept its head ("Don't care",
   "A bit" or "Strictly"). Level means the angle you built the head at.
4. **Watch best**: replay the best brain so far in real time, with the
   accelerometer's reading drawn as a green arrow.

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
