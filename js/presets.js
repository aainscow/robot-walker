// Ready-made robots to start from. Coordinates in metres, ground at y = 0.
// `head` is the index of the point the head sits on.
(function (RW) {
  'use strict';
  function make(points, sticks, head) {
    return {
      points: points.map(([x, y, kind], i) => ({ id: i + 1, x, y, kind: kind || 'free' })),
      sticks: sticks.map(([a, b, accel], i) => ({ id: i + 1, a: a + 1, b: b + 1, accel: !!accel })),
      head: head == null ? null : head + 1,
    };
  }

  RW.PRESETS = {
    inchworm: {
      name: 'Inchworm',
      design: () => make(
        [[0, 0], [0.45, 0.35, 'servo'], [0.95, 0.35, 'servo'], [1.4, 0], [1.15, 0.75]],
        [[0, 1], [1, 2, true], [2, 3], [2, 4]], 4),
    },
    dog: {
      name: 'Dog (4 legs)',
      design: () => ({ ...make(
        [[0.2, 0.75, 'servo'], [1.3, 0.75, 'servo'], [0.1, 0.4, 'servo'], [0.15, 0], [1.4, 0.4, 'servo'], [1.35, 0], [1.55, 1.05]],
        [[0, 1, true], [0, 2], [2, 3], [1, 4], [4, 5], [1, 6]], 6), camera: true }),
    },
    walker: {
      name: 'Walker (2 legs)',
      design: () => make(
        [[0.5, 0.9, 'servo'], [0.5, 1.5], [0.38, 0.47, 'servo'], [0.38, 0], [0.62, 0.47, 'servo'], [0.62, 0]],
        [[0, 1, true], [0, 2], [2, 3], [0, 4], [4, 5]], 1),
    },
    spinner: {
      name: 'Spinner (wheel legs)',
      // two legs on continuous-rotation servos that can spin all the way round
      design: () => {
        const d = make(
          [[0.3, 0.45, 'servo'], [1.2, 0.45, 'servo'], [0.3, 0.05], [1.2, 0.05], [0.75, 0.45, 'weld'], [0.9, 1.0]],
          [[0, 4, true], [4, 1], [0, 2], [1, 3], [4, 5]], 5);
        d.points[0].range = 360; d.points[1].range = 360;
        return d;
      },
    },
    crawler: {
      name: 'Crawler (one arm)',
      design: () => make(
        [[0.1, 0.05, 'weld'], [0.9, 0.05, 'servo'], [1.2, 0.45, 'servo'], [1.55, 0.05], [0, 0.5]],
        [[0, 1, true], [1, 2], [2, 3], [0, 4]], 4),
    },
  };

  // Ready-made courses. Obstacles: [type, x, w, h, dir].
  function course(list) {
    return { obstacles: list.map(([type, x, w, h, dir], i) => ({ id: i + 1, type, x, w, h, dir: dir || 1 })) };
  }
  RW.COURSES = {
    flat: { name: 'Flat', course: () => course([]) },
    bumps: { name: 'Bumpy', course: () => course([['bump', 4, 1.2, 0.12], ['bump', 7.5, 1.6, 0.2], ['bump', 11, 1, 0.15], ['bump', 14, 2, 0.25], ['bump', 17.5, 1.2, 0.18]]) },
    hurdles: { name: 'Hurdles', course: () => course([['block', 5, 0.15, 0.1], ['block', 9, 0.15, 0.15], ['block', 13, 0.15, 0.2], ['block', 17, 0.15, 0.25]]) },
    stairs: { name: 'Stairs', course: () => course([['block', 5, 6, 0.1], ['block', 6, 4, 0.2], ['block', 7, 2, 0.3], ['ramp', 11, 1.5, 0.1, -1]]) },
    hill: { name: 'Hill', course: () => course([['ramp', 5, 3, 0.5, 1], ['block', 8, 3, 0.5], ['ramp', 11, 3, 0.5, -1]]) },
  };
})(window.RW = window.RW || {});
