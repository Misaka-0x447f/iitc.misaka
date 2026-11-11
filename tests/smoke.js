'use strict';

const assert = require('assert');
const S2 = require('../src/s2cell.js');
const Core = require('../src/core.js');

// S2 L16 cell corners for (1.348, 103.85), cross-checked against Python s2sphere.
const cell = S2.Cell.fromLatLng(1.348, 103.85, 16);
const corners = cell.getCornerLatLngs();
const expected = [
  [1.3475961522082458, 103.84916146773058],
  [1.3475877506670146, 103.85061085254229],
  [1.3487593214954698, 103.8506108525423],
  [1.3487677303354688, 103.84916146773058]
];
for (let i = 0; i < 4; i++) {
  assert.ok(Math.abs(corners[i].lat - expected[i][0]) < 1e-9, 'corner lat ' + i);
  assert.ok(Math.abs(corners[i].lng - expected[i][1]) < 1e-9, 'corner lng ' + i);
}

// Synthetic grid with a single L8 next to the centre; BFS must find it.
const portals = [];
let id = 0;
for (let i = -5; i <= 5; i++) {
  for (let j = -5; j <= 5; j++) {
    const isL8 = i === 0 && j === 1;
    portals.push({
      guid: 'g' + id,
      title: 'p' + id,
      lat: 1.348 + i * 0.0009,
      lng: 103.85 + j * 0.0009,
      level: isL8 ? 8 : 3,
      team: 0
    });
    id++;
  }
}

const centerIndex = portals.findIndex((p) => p.lat === 1.348 && p.lng === 103.85);
const res = Core.analyze(portals, {
  centerIndex,
  top: 5,
  suggest: 3,
  jumpRadius: 500,
  S2
});

assert.ok(res.nearest.length > 0, 'nearest list is empty');
assert.ok(res.reachable.length > 0, 'reachable list is empty');
assert.strictEqual(res.portals[res.reachable[0].idx].level, 8, 'first reachable is not L8');
assert.ok(res.reachable[0].hops >= 1, 'L8 should need at least one hop');
assert.strictEqual(res.fparent[centerIndex], -1, 'centre parent should be -1');

console.log('smoke ok');
