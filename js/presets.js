// Ready-made robots to start from. Coordinates in metres, ground at y = 0.
(function (RW) {
  'use strict';
  function make(points, sticks) {
    return {
      points: points.map(([x, y, kind], i) => ({ id: i + 1, x, y, kind: kind || 'free' })),
      sticks: sticks.map(([a, b, accel], i) => ({ id: i + 1, a: a + 1, b: b + 1, accel: !!accel })),
    };
  }

  RW.PRESETS = {
    inchworm: {
      name: 'Inchworm',
      design: () => make(
        [[0, 0], [0.45, 0.35, 'servo'], [0.95, 0.35, 'servo'], [1.4, 0]],
        [[0, 1], [1, 2, true], [2, 3]]),
    },
    dog: {
      name: 'Dog (4 legs)',
      design: () => make(
        [[0.2, 0.75, 'servo'], [1.3, 0.75, 'servo'], [0.1, 0.4, 'servo'], [0.15, 0], [1.4, 0.4, 'servo'], [1.35, 0]],
        [[0, 1, true], [0, 2], [2, 3], [1, 4], [4, 5]]),
    },
    walker: {
      name: 'Walker (2 legs)',
      design: () => make(
        [[0.5, 0.9, 'servo'], [0.5, 1.5], [0.38, 0.47, 'servo'], [0.38, 0], [0.62, 0.47, 'servo'], [0.62, 0]],
        [[0, 1, true], [0, 2], [2, 3], [0, 4], [4, 5]]),
    },
    crawler: {
      name: 'Crawler (one arm)',
      design: () => make(
        [[0, 0.05], [0.8, 0.05, 'servo'], [1.1, 0.45, 'servo'], [1.45, 0.05]],
        [[0, 1, true], [1, 2], [2, 3]]),
    },
  };
})(window.RW = window.RW || {});
