/*
 * Trimmed S2 cell geometry (level-N square cells on the cube face).
 * Derived from s2-geometry.js by AJ ONeal / Jon Atkins (MIT / Apache-2.0 / ISC).
 * Only the pieces needed to map lat/lng -> level-N cell and get its 4 corners.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.DroneL8S2 = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  var DEG_TO_RAD = Math.PI / 180;
  var RAD_TO_DEG = 180 / Math.PI;

  function latLngToXYZ(lat, lng) {
    var phi = lat * DEG_TO_RAD;
    var theta = lng * DEG_TO_RAD;
    var c = Math.cos(phi);
    return [Math.cos(theta) * c, Math.sin(theta) * c, Math.sin(phi)];
  }

  function xyzToLatLng(xyz) {
    return {
      lat: Math.atan2(xyz[2], Math.sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])) * RAD_TO_DEG,
      lng: Math.atan2(xyz[1], xyz[0]) * RAD_TO_DEG
    };
  }

  function largestAbsComponent(xyz) {
    var t = [Math.abs(xyz[0]), Math.abs(xyz[1]), Math.abs(xyz[2])];
    if (t[0] > t[1]) return t[0] > t[2] ? 0 : 2;
    return t[1] > t[2] ? 1 : 2;
  }

  function faceXYZToUV(face, xyz) {
    var u, v;
    switch (face) {
      case 0: u = xyz[1] / xyz[0]; v = xyz[2] / xyz[0]; break;
      case 1: u = -xyz[0] / xyz[1]; v = xyz[2] / xyz[1]; break;
      case 2: u = -xyz[0] / xyz[2]; v = -xyz[1] / xyz[2]; break;
      case 3: u = xyz[2] / xyz[0]; v = xyz[1] / xyz[0]; break;
      case 4: u = xyz[2] / xyz[1]; v = -xyz[0] / xyz[1]; break;
      case 5: u = -xyz[1] / xyz[2]; v = -xyz[0] / xyz[2]; break;
      default: throw new Error('Invalid face');
    }
    return [u, v];
  }

  function xyzToFaceUV(xyz) {
    var face = largestAbsComponent(xyz);
    if (xyz[face] < 0) face += 3;
    return [face, faceXYZToUV(face, xyz)];
  }

  function faceUVToXYZ(face, uv) {
    var u = uv[0], v = uv[1];
    switch (face) {
      case 0: return [1, u, v];
      case 1: return [-u, 1, v];
      case 2: return [-u, -v, 1];
      case 3: return [-1, -v, -u];
      case 4: return [v, -1, -u];
      case 5: return [v, u, -1];
      default: throw new Error('Invalid face');
    }
  }

  function singleSTtoUV(st) {
    if (st >= 0.5) return (4 * st * st - 1) / 3;
    return (1 - 4 * (1 - st) * (1 - st)) / 3;
  }

  function stToUV(st) {
    return [singleSTtoUV(st[0]), singleSTtoUV(st[1])];
  }

  function singleUVtoST(uv) {
    if (uv >= 0) return 0.5 * Math.sqrt(1 + 3 * uv);
    return 1 - 0.5 * Math.sqrt(1 - 3 * uv);
  }

  function uvToST(uv) {
    return [singleUVtoST(uv[0]), singleUVtoST(uv[1])];
  }

  function stToIJ(st, order) {
    var max = 1 << order;
    function one(s) {
      var ij = Math.floor(s * max);
      return Math.max(0, Math.min(max - 1, ij));
    }
    return [one(st[0]), one(st[1])];
  }

  function ijToST(ij, order, offsets) {
    var max = 1 << order;
    return [(ij[0] + offsets[0]) / max, (ij[1] + offsets[1]) / max];
  }

  function Cell(face, ij, level) {
    this.face = face;
    this.ij = ij;
    this.level = level;
  }

  Cell.fromLatLng = function (lat, lng, level) {
    var faceuv = xyzToFaceUV(latLngToXYZ(lat, lng));
    var st = uvToST(faceuv[1]);
    var ij = stToIJ(st, level);
    return new Cell(faceuv[0], ij, level);
  };

  Cell.prototype.getLatLng = function () {
    var st = ijToST(this.ij, this.level, [0.5, 0.5]);
    return xyzToLatLng(faceUVToXYZ(this.face, stToUV(st)));
  };

  Cell.prototype.getCornerLatLngs = function () {
    var offsets = [[0, 0], [1, 0], [1, 1], [0, 1]];
    var out = [];
    for (var i = 0; i < 4; i++) {
      var st = ijToST(this.ij, this.level, offsets[i]);
      out.push(xyzToLatLng(faceUVToXYZ(this.face, stToUV(st))));
    }
    return out;
  };

  return { Cell: Cell, latLngToXYZ: latLngToXYZ, xyzToLatLng: xyzToLatLng };
});
