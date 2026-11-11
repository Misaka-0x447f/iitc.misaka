// ==UserScript==
// @id             bfs.iitc.misaka
// @name           bfs.iitc.misaka
// @category       Info
// @version        2026.11.11.163205
// @namespace      https://github.com/Misaka-0x447f/iitc.misaka
// @downloadURL    https://raw.githubusercontent.com/Misaka-0x447f/iitc.misaka/master/dist/bfs.iitc.misaka.user.js
// @updateURL      https://raw.githubusercontent.com/Misaka-0x447f/iitc.misaka/master/dist/bfs.iitc.misaka.user.js
// @description    以选中 portal 为中心：最近 L8、无人机可达路线、不可达时的替代起点
// @author         Misaka-0x447f
// @match          https://intel.ingress.com/*
// @grant          none
// ==/UserScript==

function wrapper(plugin_info) {
  if (typeof window.plugin !== 'function') window.plugin = function () {};

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


/*
 * Core planning logic for the Drone L8 planner.
 *
 * Visibility model (same as IITC Drone Helper / community measurements):
 *   drone at portal A can select portal B  <=>  cell16(B) intersects the
 *   500 m disc around A.  Edges are directed; one-way edges are normal.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.DroneL8Core = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  var EARTH_R = 6371010.0;
  var DEG = Math.PI / 180;
  var S2_LEVEL = 16;

  function hav(lat1, lng1, lat2, lng2) {
    var p1 = lat1 * DEG, p2 = lat2 * DEG;
    var dp = p2 - p1, dl = (lng2 - lng1) * DEG;
    var a = Math.sin(dp / 2) * Math.sin(dp / 2) +
      Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) * Math.sin(dl / 2);
    return 2 * EARTH_R * Math.asin(Math.min(1, Math.sqrt(a)));
  }

  // local east/north meters around (lat0, lng0)
  function projX(lng, lat0, lng0) { return (lng - lng0) * 111320 * Math.cos(lat0 * DEG); }
  function projY(lat, lat0) { return (lat - lat0) * 110540; }

  function distPointSeg(px, py, ax, ay, bx, by) {
    var dx = bx - ax, dy = by - ay;
    var d2 = dx * dx + dy * dy;
    var t = d2 > 1e-12 ? ((px - ax) * dx + (py - ay) * dy) / d2 : 0;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    var qx = ax + t * dx, qy = ay + t * dy;
    var ex = px - qx, ey = py - qy;
    return Math.sqrt(ex * ex + ey * ey);
  }

  function pointInPoly(px, py, poly) {
    var inside = false;
    for (var i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      var yi = poly[i][1], yj = poly[j][1];
      if ((yi > py) !== (yj > py)) {
        var x = ((poly[j][0] - poly[i][0]) * (py - yi)) / (yj - yi) + poly[i][0];
        if (px < x) inside = !inside;
      }
    }
    return inside;
  }

  function distPointPoly(px, py, poly) {
    if (pointInPoly(px, py, poly)) return 0;
    var best = Infinity;
    for (var i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      var d = distPointSeg(px, py, poly[j][0], poly[j][1], poly[i][0], poly[i][1]);
      if (d < best) best = d;
    }
    return best;
  }

  // Build the directed drone graph. S2lib = { Cell }.
  // Edges: A -> B iff cell16(B) intersects the 500 m disc around A.
  function build(portals, S2lib, jumpRadius) {
    var n = portals.length;
    var radius = jumpRadius || 500;
    var cells = new Array(n);
    var i, j, p, c, cs;

    for (i = 0; i < n; i++) {
      p = portals[i];
      c = S2lib.Cell.fromLatLng(p.lat, p.lng, S2_LEVEL);
      cs = c.getCornerLatLngs();
      var minLat = Infinity, maxLat = -Infinity, minLng = Infinity, maxLng = -Infinity;
      var arr = new Array(4);
      for (j = 0; j < 4; j++) {
        var k = cs[j];
        arr[j] = [k.lat, k.lng];
        if (k.lat < minLat) minLat = k.lat;
        if (k.lat > maxLat) maxLat = k.lat;
        if (k.lng < minLng) minLng = k.lng;
        if (k.lng > maxLng) maxLng = k.lng;
      }
      var dLat = radius / 110540;
      var dLng = radius / (111320 * Math.cos(p.lat * DEG));
      cells[i] = {
        corners: arr,
        minLat: minLat - dLat, maxLat: maxLat + dLat,
        minLng: minLng - dLng, maxLng: maxLng + dLng
      };
    }

    // spatial hash on the inflated cell bboxes (grid ~550 m)
    var GRID = 0.005;
    var buckets = Object.create(null);
    for (i = 0; i < n; i++) {
      var cb = cells[i];
      var gy0 = Math.floor(cb.minLat / GRID), gy1 = Math.floor(cb.maxLat / GRID);
      var gx0 = Math.floor(cb.minLng / GRID), gx1 = Math.floor(cb.maxLng / GRID);
      for (var gy = gy0; gy <= gy1; gy++) {
        for (var gx = gx0; gx <= gx1; gx++) {
          var key = gy + ':' + gx;
          (buckets[key] || (buckets[key] = [])).push(i);
        }
      }
    }

    var adj = new Array(n);
    var radj = new Array(n);
    for (i = 0; i < n; i++) { adj[i] = []; radj[i] = []; }

    for (i = 0; i < n; i++) {
      p = portals[i];
      var buck = buckets[Math.floor(p.lat / GRID) + ':' + Math.floor(p.lng / GRID)];
      if (!buck) continue;
      for (var bi = 0; bi < buck.length; bi++) {
        var b = buck[bi];
        if (b === i) continue;
        var cbb = cells[b];
        if (p.lat < cbb.minLat || p.lat > cbb.maxLat || p.lng < cbb.minLng || p.lng > cbb.maxLng) continue;
        var poly = new Array(4);
        for (var t = 0; t < 4; t++) {
          poly[t] = [projX(cbb.corners[t][1], p.lat, p.lng), projY(cbb.corners[t][0], p.lat)];
        }
        if (distPointPoly(0, 0, poly) <= radius) {
          adj[i].push(b);
          radj[b].push(i);
        }
      }
    }
    return { adj: adj, radj: radj, cells: cells };
  }

  function bfs(adj, start) {
    var n = adj.length;
    var dist = new Array(n).fill(-1);
    var parent = new Array(n).fill(-1);
    dist[start] = 0;
    var q = [start], head = 0;
    while (head < q.length) {
      var u = q[head++];
      var ns = adj[u];
      for (var k = 0; k < ns.length; k++) {
        var v = ns[k];
        if (dist[v] === -1) {
          dist[v] = dist[u] + 1;
          parent[v] = u;
          q.push(v);
        }
      }
    }
    return { dist: dist, parent: parent };
  }

  function pathForward(parent, target) {
    var out = [];
    var x = target;
    while (x !== -1) { out.push(x); x = parent[x]; }
    return out.reverse();
  }

  // path from start to target using reverse-BFS `next` array
  function pathFromNext(next, start, target) {
    var out = [start];
    var guard = 0;
    while (out[out.length - 1] !== target && guard++ < 10000) {
      var y = next[out[out.length - 1]];
      if (y === -1) break;
      out.push(y);
    }
    return out;
  }

  /*
   * portals: [{guid, title, lat, lng, level, team}]
   * opts: {centerIndex, meLat, meLng, top, suggest, titleContains, jumpRadius, S2}
   */
  function analyze(portals, opts) {
    var n = portals.length;
    var g = build(portals, opts.S2, opts.jumpRadius);
    var center = opts.centerIndex;
    var cLat = portals[center].lat, cLng = portals[center].lng;
    var meLat = opts.meLat != null ? opts.meLat : cLat;
    var meLng = opts.meLng != null ? opts.meLng : cLng;

    var l8 = [];
    for (var i = 0; i < n; i++) {
      if (portals[i].level === 8) {
        if (opts.titleContains && portals[i].title.toLowerCase().indexOf(opts.titleContains.toLowerCase()) < 0) continue;
        l8.push(i);
      }
    }

    if (opts.maxDist) {
      l8 = l8.filter(function (idx) {
        return hav(cLat, cLng, portals[idx].lat, portals[idx].lng) <= opts.maxDist;
      });
    }

    var f = bfs(g.adj, center);
    var byDist = l8.slice().sort(function (a, b) {
      return hav(cLat, cLng, portals[a].lat, portals[a].lng) - hav(cLat, cLng, portals[b].lat, portals[b].lng);
    });

    var nearest = byDist.slice(0, opts.top || 10).map(function (idx) {
      var p = portals[idx];
      return {
        idx: idx,
        dist: hav(cLat, cLng, p.lat, p.lng),
        hops: f.dist[idx],
        team: p.team,
        title: p.title
      };
    });

    var reach = l8.filter(function (idx) { return f.dist[idx] >= 0; })
      .sort(function (a, b) {
        return f.dist[a] - f.dist[b] || hav(cLat, cLng, portals[a].lat, portals[a].lng) - hav(cLat, cLng, portals[b].lat, portals[b].lng);
      })
      .slice(0, opts.top || 10)
      .map(function (idx) {
        return {
          idx: idx,
          hops: f.dist[idx],
          path: pathForward(f.parent, idx),
          dist: hav(cLat, cLng, portals[idx].lat, portals[idx].lng)
        };
      });

    var unreachable = byDist.filter(function (idx) { return f.dist[idx] < 0; }).slice(0, opts.suggest || 3);
    var suggestions = unreachable.map(function (t) {
      var r = bfs(g.radj, t); // r.parent[x] = x 到 t 的下一跳
      var cands = [];
      for (var x = 0; x < n; x++) {
        if (x === t || r.dist[x] < 0) continue;
        cands.push({ idx: x, walk: hav(meLat, meLng, portals[x].lat, portals[x].lng), hops: r.dist[x] });
      }
      function mk(c) {
        return {
          idx: c.idx, walk: c.walk, hops: c.hops,
          path: pathFromNext(r.parent, c.idx, t)
        };
      }
      var nearest = cands.slice().sort(function (a, b) { return a.walk - b.walk; }).slice(0, 3).map(mk);
      var fastest = cands.slice().sort(function (a, b) {
        return a.hops - b.hops || a.walk - b.walk;
      }).slice(0, 3).map(mk);
      return {
        target: t,
        distFromCenter: hav(cLat, cLng, portals[t].lat, portals[t].lng),
        nearestStarts: nearest,
        fastestStarts: fastest
      };
    });

    return {
      portals: portals,
      center: center,
      me: { lat: meLat, lng: meLng },
      stats: { portals: n, l8: l8.length },
      nearest: nearest,
      reachable: reach,
      fparent: f.parent,
      suggestions: suggestions
    };
  }

  return { hav: hav, build: build, bfs: bfs, analyze: analyze, pathForward: pathForward, pathFromNext: pathFromNext };
});


/*
 * IITC UI layer for the Drone L8 planner.
 *
 * Search termination: two inputs
 *   - count:  stop once this many qualifying L8 portals are found
 *   - maxKm:  hard distance cap (km) from the centre portal
 * Fetch radius grows in steps (1, 2, 3, 4, 6 ... km) until either is met.
 *
 * Data is pulled from Intel's getEntities via IITC's tile/postAjax helpers,
 * so it is not limited to portals currently rendered on screen.
 *
 * Runtime validation: every run checks tile coverage, boundary sampling,
 * duplicate/bad records and route truncation, and reports warnings in the panel.
 */
(function () {
  'use strict';

  var S2 = window.DroneL8S2;
  var Core = window.DroneL8Core;
  if (!S2 || !Core) throw new Error('DroneL8: missing core deps');

  var PLUGIN = (window.plugin.droneL8 = window.plugin.droneL8 || {});
  var TEAM_COLOR = { 0: '#bbb', 1: '#3f6', 2: '#39f' };
  var TEAM_NAME = { 0: '中立', 1: '绿', 2: '蓝' };
  var TILE_CHUNK = 5;
  var TILE_CONCURRENCY = 4;
  var TILE_RETRIES = 2;
  var RADIUS_STEPS_KM = [1, 2, 3, 4, 6, 8, 10, 12, 16, 20];
  var BOUNDARY_MARGIN_M = 150;
  var CACHE_PREFIX = 'bfs.iitc.misaka.cache.v1.';
  var CACHE_TTL_MS = 7 * 24 * 60 * 60 * 1000;
  var storageOk = (function () {
    try {
      var probe = CACHE_PREFIX + '__probe';
      localStorage.setItem(probe, '1');
      localStorage.removeItem(probe);
      return true;
    } catch (e) { return false; }
  })();

  var state = {
    centerGuid: null,
    centerPortal: null,
    selectedGuid: null,
    meLat: null,
    meLng: null,
    count: 5,
    maxKm: 6,
    suggest: 3,
    titleFilter: ''
  };
  var layers = null;
  var busy = false;
  var tileCache = Object.create(null);
  var tileCount = 0;

  function $(id) { return document.getElementById(id); }

  function td(text, cls) {
    var d = document.createElement('td');
    if (cls) d.className = cls;
    d.textContent = text;
    return d;
  }

  function teamSpan(team) {
    var s = document.createElement('span');
    s.className = 'drone-l8-team';
    s.style.color = TEAM_COLOR[team] || '#bbb';
    s.textContent = TEAM_NAME[team] || '?';
    return s;
  }

  function ensureLayers() {
    if (layers) return;
    layers = L.layerGroup();
    if (window.addLayerGroup) window.addLayerGroup('Drone L8', layers, true);
    else layers.addTo(window.map);
  }

  function clearLayers() {
    ensureLayers();
    layers.clearLayers();
  }

  function setStatus(msg, isErr) {
    var el = $('drone-l8-status');
    if (!el) return;
    el.textContent = msg || '';
    el.style.color = isErr ? '#f66' : '';
  }

  function drawPath(portals, idxs, color, label, fit) {
    if (!idxs || !idxs.length) return;
    var latlngs = idxs.map(function (i) { return [portals[i].lat, portals[i].lng]; });
    L.polyline(latlngs, { color: color, weight: 4, opacity: 0.9 }).addTo(layers);
    idxs.forEach(function (i, k) {
      L.circleMarker([portals[i].lat, portals[i].lng], {
        radius: k === 0 ? 7 : 5, color: color, fillColor: color, fillOpacity: 0.9, weight: 2
      }).addTo(layers);
    });
    var first = portals[idxs[0]];
    L.circle([first.lat, first.lng], {
      radius: 500, color: '#0ff', weight: 1, opacity: 0.6, fill: false
    }).addTo(layers);
    if (label) setStatus(label);
    if (fit === false) return;
    if (idxs.length > 1) window.map.fitBounds(L.latLngBounds(latlngs), { padding: [40, 40] });
    else window.map.setView(latlngs[0], 17);
  }

  // jump the map to a result portal; keep the current result list untouched
  function jumpTo(portals, idx, path, color) {
    var p = portals[idx];
    if (!p) return;
    clearLayers();
    if (path && path.length > 1) drawPath(portals, path, color || '#ffd400', null, false);
    window.map.setView([p.lat, p.lng], 17, { animate: false });
    L.circleMarker([p.lat, p.lng], {
      radius: 9, color: '#fff', weight: 2, fillColor: color || '#ffd400', fillOpacity: 0.95
    }).addTo(layers);
    setStatus(p.title || p.guid);
  }

  // ---------- Intel data fetch ----------

  function tilesApi() {
    return (window.IITC && IITC.map && IITC.map.tiles) || null;
  }

  function tilesForDisc(lat, lng, radiusM) {
    var api = tilesApi();
    if (!api) return null;
    var dz = api.getDataZoomForMapZoom(16);
    var tp = api.getMapZoomParameters(dz);
    var dLat = radiusM / 110540;
    var dLng = radiusM / (111320 * Math.cos(lat * Math.PI / 180));
    var x1 = api.lngToTile(lng - dLng, tp);
    var x2 = api.lngToTile(lng + dLng, tp);
    var y1 = api.latToTile(lat + dLat, tp);
    var y2 = api.latToTile(lat - dLat, tp);
    var xMin = Math.max(0, Math.min(x1, x2));
    var xMax = Math.min(tp.tilesPerEdge - 1, Math.max(x1, x2));
    var yMin = Math.max(0, Math.min(y1, y2));
    var yMax = Math.min(tp.tilesPerEdge - 1, Math.max(y1, y2));
    var ids = [];
    for (var y = yMin; y <= yMax; y++) {
      for (var x = xMin; x <= xMax; x++) ids.push(api.pointToTileId(tp, x, y));
    }
    return ids;
  }

  function parseTile(map, id) {
    var tile = (map && map[id]) || {};
    var ents = tile.gameEntities || [];
    var out = [];
    for (var i = 0; i < ents.length; i++) {
      var e = ents[i];
      if (!e || !e[2] || e[2][0] !== 'p') continue;
      var d = e[2];
      out.push({
        guid: e[0],
        title: d[8] || '',
        lat: d[2] / 1e6,
        lng: d[3] / 1e6,
        level: d[4],
        team: d[1] === 'E' ? 1 : (d[1] === 'R' ? 2 : 0)
      });
    }
    return out;
  }

  function postOnce(chunk) {
    return new Promise(function (resolve) {
      var done = false;
      function finish(v) { if (!done) { done = true; resolve(v); } }
      try {
        window.postAjax('getEntities', { tileKeys: chunk }, function (data) {
          finish({ ok: true, map: (data && data.result && data.result.map) || {} });
        }, function () {
          finish({ ok: false });
        });
      } catch (e) {
        finish({ ok: false });
      }
    });
  }

  function fetchTiles(ids, onProgress) {
    var stats = { expected: ids.length, cached: 0, requested: 0, retried: 0, failed: [] };
    var pending = [];
    for (var ii = 0; ii < ids.length; ii++) {
      var tid = ids[ii];
      if (tid in tileCache) continue;
      var stored = loadTile(tid);
      if (stored) { tileCache[tid] = stored; tileCount++; continue; }
      pending.push(tid);
    }
    stats.cached = stats.expected - pending.length;
    stats.requested = pending.length;
    var total = pending.length, done = 0;
    if (!total) { onProgress(0, 0); return Promise.resolve(stats); }

    function requestChunk(chunk, attempt) {
      return postOnce(chunk).then(function (res) {
        var bad = [];
        if (res.ok) {
          for (var i = 0; i < chunk.length; i++) {
            var id = chunk[i];
            var tile = res.map[id];
            if (!tile || !Array.isArray(tile.gameEntities)) { bad.push(id); continue; }
            var ps = parseTile(res.map, id);
            tileCache[id] = ps;
            tileCount++;
            saveTile(id, ps);
          }
        } else {
          bad = chunk.slice();
        }
        if (bad.length && attempt < TILE_RETRIES) {
          stats.retried++;
          return new Promise(function (r) {
            setTimeout(function () { r(requestChunk(bad, attempt + 1)); }, 300 + attempt * 300);
          });
        }
        if (bad.length) for (var j = 0; j < bad.length; j++) stats.failed.push(bad[j]);
      });
    }

    function worker() {
      return new Promise(function (resolve) {
        function step() {
          if (!pending.length) return resolve();
          var chunk = pending.splice(0, TILE_CHUNK);
          requestChunk(chunk, 0).then(function () {
            done += chunk.length;
            onProgress(done, total);
            step();
          });
        }
        step();
      });
    }

    var workers = [];
    for (var w = 0; w < TILE_CONCURRENCY; w++) workers.push(worker());
    return Promise.all(workers).then(function () { return stats; });
  }

  function allPortals() {
    var seen = Object.create(null);
    var out = [];
    var dup = 0;
    for (var id in tileCache) {
      var ps = tileCache[id];
      for (var i = 0; i < ps.length; i++) {
        var p = ps[i];
        if (seen[p.guid]) { dup++; continue; }
        seen[p.guid] = 1;
        out.push(p);
      }
    }
    return { portals: out, duplicates: dup };
  }

  function cacheKeys() {
    var out = [];
    try {
      for (var i = 0; i < localStorage.length; i++) {
        var k = localStorage.key(i);
        if (k && k.indexOf(CACHE_PREFIX) === 0) out.push(k);
      }
    } catch (e) {}
    return out;
  }

  function cacheSize() {
    if (!storageOk) return 0;
    var n = 0;
    var keys = cacheKeys();
    for (var i = 0; i < keys.length; i++) {
      var v = localStorage.getItem(keys[i]) || '';
      n += (keys[i].length + v.length) * 2;
    }
    return n;
  }

  function formatBytes(n) {
    if (n < 1024) return n + ' B';
    if (n < 1024 * 1024) return (n / 1024).toFixed(1) + ' KB';
    return (n / 1024 / 1024).toFixed(2) + ' MB';
  }

  function removeStored(id) {
    if (!storageOk) return;
    try { localStorage.removeItem(CACHE_PREFIX + id); } catch (e) {}
  }

  function evictOldest() {
    var keys = cacheKeys();
    var oldestKey = null, oldestTime = Infinity;
    for (var i = 0; i < keys.length; i++) {
      try {
        var o = JSON.parse(localStorage.getItem(keys[i]));
        if (!o || !o.t) { localStorage.removeItem(keys[i]); continue; }
        if (o.t < oldestTime) { oldestTime = o.t; oldestKey = keys[i]; }
      } catch (e) { localStorage.removeItem(keys[i]); }
    }
    if (!oldestKey) return false;
    localStorage.removeItem(oldestKey);
    return true;
  }

  function loadTile(id) {
    if (!storageOk) return null;
    try {
      var raw = localStorage.getItem(CACHE_PREFIX + id);
      if (!raw) return null;
      var o = JSON.parse(raw);
      if (!o || !o.t || !o.p || Date.now() - o.t > CACHE_TTL_MS) { removeStored(id); return null; }
      var out = [];
      for (var i = 0; i < o.p.length; i++) {
        var a = o.p[i];
        out.push({ guid: a[0], title: a[1] || '', lat: a[2], lng: a[3], level: a[4], team: a[5] });
      }
      return out;
    } catch (e) { removeStored(id); return null; }
  }

  function saveTile(id, portals) {
    if (!storageOk) return;
    var packed = [];
    for (var i = 0; i < portals.length; i++) {
      var p = portals[i];
      packed.push([p.guid, p.title || '', p.lat, p.lng, p.level, p.team]);
    }
    var payload = JSON.stringify({ t: Date.now(), p: packed });
    var key = CACHE_PREFIX + id;
    for (var attempt = 0; attempt < 50; attempt++) {
      try { localStorage.setItem(key, payload); return; }
      catch (e) { if (!evictOldest()) return; }
    }
  }

  function pruneExpired() {
    if (!storageOk) return;
    var now = Date.now();
    var keys = cacheKeys();
    for (var i = 0; i < keys.length; i++) {
      try {
        var o = JSON.parse(localStorage.getItem(keys[i]));
        if (!o || !o.t || now - o.t > CACHE_TTL_MS) localStorage.removeItem(keys[i]);
      } catch (e) { localStorage.removeItem(keys[i]); }
    }
  }

  function updateCacheLabel() {
    var el = $('drone-l8-cache');
    if (!el) return;
    el.textContent = '缓存 ' + formatBytes(cacheSize()) + '（TTL 7 天）';
  }

  function clearPersistentCache() {
    if (storageOk) {
      var keys = cacheKeys();
      for (var i = 0; i < keys.length; i++) localStorage.removeItem(keys[i]);
    }
    tileCache = Object.create(null);
    tileCount = 0;
    updateCacheLabel();
  }

  function centerInfo() {
    var m = window.portals && window.portals[state.centerGuid];
    if (!m) return null;
    var ll = IITC.portal.getLatLng ? IITC.portal.getLatLng(m) : m.getLatLng();
    var d = m.options.data || {};
    return {
      guid: state.centerGuid,
      title: d.title || '',
      lat: ll.lat,
      lng: ll.lng,
      level: m.options.level,
      team: m.options.team
    };
  }

  function portalTitle(guid) {
    var m = (window.portals || {})[guid];
    var t = m && m.options && m.options.data && m.options.data.title;
    return t || (guid ? guid.substring(0, 8) : '');
  }

  function centerLabel() {
    var el = $('drone-l8-center');
    if (!el) return;
    if (!state.centerGuid) { el.textContent = '中心：未选中 portal'; return; }
    var t = (state.centerPortal && state.centerPortal.guid === state.centerGuid)
      ? state.centerPortal.title : portalTitle(state.centerGuid);
    el.textContent = '中心：' + (t || state.centerGuid.substring(0, 8));
  }

  function selectionLabel() {
    var el = $('drone-l8-sel');
    if (!el) return;
    if (!state.selectedGuid) { el.textContent = ''; return; }
    el.textContent = '选中：' + portalTitle(state.selectedGuid) +
      (state.selectedGuid === state.centerGuid ? '（当前中心）' : '（点“分析”以此为心）');
  }

  function currentCenter() {
    if (state.centerPortal && state.centerPortal.guid === state.centerGuid) return state.centerPortal;
    return centerInfo();
  }

  function qualifies(p, center, maxM, filter) {
    if (p.level !== 8) return false;
    if (filter && p.title.toLowerCase().indexOf(filter.toLowerCase()) < 0) return false;
    return Core.hav(center.lat, center.lng, p.lat, p.lng) <= maxM;
  }

  function countQualifying(portals, center, maxM, filter) {
    var n = 0;
    for (var i = 0; i < portals.length; i++) if (qualifies(portals[i], center, maxM, filter)) n++;
    return n;
  }

  // ---------- runtime validation ----------

  function validateCoverage(center, radiusM, ids, stats) {
    var v = { warnings: [] };
    var api = tilesApi();
    var fetched = 0;
    for (var i = 0; i < ids.length; i++) if (ids[i] in tileCache) fetched++;
    v.tilesExpected = ids.length;
    v.tilesFetched = fetched;
    v.tilesMissing = ids.length - fetched;
    v.retried = stats.retried;
    v.failed = (stats.failed || []).slice();

    // boundary sample: tiles hit by points on the search circle must be in the enum
    var checks = 0, miss = 0;
    if (api) {
      var tp = api.getMapZoomParameters(api.getDataZoomForMapZoom(16));
      var cosLat = Math.cos(center.lat * Math.PI / 180);
      for (var k = 0; k < 24; k++) {
        var th = k / 24 * 2 * Math.PI;
        var la = center.lat + (radiusM * Math.cos(th)) / 110540;
        var lo = center.lng + (radiusM * Math.sin(th)) / (111320 * cosLat);
        var id = api.pointToTileId(tp, api.lngToTile(lo, tp), api.latToTile(la, tp));
        checks++;
        if (ids.indexOf(id) < 0) miss++;
      }
    }
    v.sampleChecks = checks;
    v.sampleMiss = miss;

    if (v.tilesMissing) v.warnings.push(v.tilesMissing + ' 个瓦片未拉到（已重试），结果可能缺 portal');
    if (miss) v.warnings.push('边界采样 ' + miss + '/' + checks + ' 未命中，瓦片枚举异常');
    return v;
  }

  function validateData(portals, duplicates, v) {
    var bad = 0, badGuid = 0;
    for (var i = 0; i < portals.length; i++) {
      var p = portals[i];
      if (!p.guid) badGuid++;
      if (!isFinite(p.lat) || !isFinite(p.lng) || p.level < 0 || p.level > 8) bad++;
    }
    v.badRecords = bad + badGuid;
    v.duplicates = duplicates;
    if (v.badRecords) v.warnings.push(v.badRecords + ' 条 portal 记录字段异常');
    if (duplicates) v.warnings.push(duplicates + ' 条重复 portal 记录（跨瓦片）');
    return v;
  }

  // mark reachable routes whose hops run close to the fetch boundary
  function boundaryFlags(res, radiusM) {
    var flags = {};
    var c = res.portals[res.center];
    res.reachable.forEach(function (r) {
      var maxd = 0;
      for (var i = 0; i < r.path.length; i++) {
        var p = res.portals[r.path[i]];
        var d = Core.hav(c.lat, c.lng, p.lat, p.lng);
        if (d > maxd) maxd = d;
      }
      if (maxd > radiusM - BOUNDARY_MARGIN_M) flags[r.idx] = Math.round(maxd);
    });
    return flags;
  }

  // ---------- analysis ----------

  function analyzeWith(portals, center, me) {
    var centerIdx = -1;
    for (var i = 0; i < portals.length; i++) {
      if (portals[i].guid === center.guid) { centerIdx = i; break; }
    }
    if (centerIdx < 0) {
      centerIdx = portals.length;
      portals.push(center);
    }
    return Core.analyze(portals, {
      centerIndex: centerIdx,
      meLat: me.lat,
      meLng: me.lng,
      top: state.count,
      suggest: state.suggest,
      titleContains: state.titleFilter || null,
      maxDist: state.maxKm * 1000,
      jumpRadius: 500,
      S2: S2
    });
  }

  function run() {
    if (busy) { setStatus('正在跑，稍等…', true); return; }
    var center = currentCenter();
    if (!center) { setStatus('先选中一个 portal 作为中心', true); return; }
    var me = (state.meLat != null) ? { lat: state.meLat, lng: state.meLng } : { lat: center.lat, lng: center.lng };

    busy = true;
    setStatus('准备加载数据…');
    var started = performance.now();
    var maxKm = state.maxKm;
    var steps = RADIUS_STEPS_KM.filter(function (r) { return r <= maxKm; });
    if (!steps.length || steps[steps.length - 1] < maxKm) steps.push(maxKm);
    var idx = 0;
    var fetchStats = { retried: 0, failed: [] };

    function mergeStats(a, b) {
      a.retried += b.retried;
      a.failed = a.failed.concat(b.failed);
    }

    function nextStepAfter(r) {
      for (var i = 0; i < RADIUS_STEPS_KM.length; i++) {
        if (RADIUS_STEPS_KM[i] > r) return RADIUS_STEPS_KM[i];
      }
      return r;
    }

    function finish(stopKm, fetchKm, found) {
      var pack = allPortals();
      var portals = pack.portals;
      var res = analyzeWith(portals, center, me);
      var secs = (performance.now() - started) / 1000;
      var ids = tilesForDisc(center.lat, center.lng, fetchKm * 1000) || [];
      var v = validateCoverage(center, fetchKm * 1000, ids, fetchStats);
      validateData(portals, pack.duplicates, v);
      var flags = boundaryFlags(res, fetchKm * 1000);
      if (Object.keys(flags).length) {
        v.warnings.push(Object.keys(flags).length + ' 条可达路线贴近拉取边界（≤' + BOUNDARY_MARGIN_M + ' m），可能不是最优');
      }
      var unreachable = res.nearest.filter(function (r) { return r.hops < 0; }).length;
      if (unreachable) v.warnings.push(unreachable + ' 个最近 L8 不可达，可能受加载边界影响');
      v.ok = v.warnings.length === 0;
      PLUGIN.lastValidation = v;

      busy = false;
      state.centerPortal = res.portals[res.center];
      state.centerGuid = state.centerPortal.guid;
      centerLabel();
      selectionLabel();
      clearLayers();
      render(res, portals.length, stopKm, fetchKm, found, secs, v, flags);
      var c = res.portals[res.center];
      L.circle([c.lat, c.lng], { radius: 500, color: '#0ff', weight: 1, opacity: 0.6, fill: false }).addTo(layers);
      L.circleMarker([c.lat, c.lng], { radius: 7, color: '#ffd400', fillColor: '#ffd400', fillOpacity: 0.9 }).addTo(layers);
      if (state.meLat != null) {
        L.circleMarker([state.meLat, state.meLng], { radius: 7, color: '#f0f', fillColor: '#f0f', fillOpacity: 0.9 }).addTo(layers);
      }
      var area = fetchKm > stopKm
        ? ('半径 ' + stopKm + ' km（外扩到 ' + fetchKm + ' km）')
        : ('半径 ' + stopKm + ' km');
      updateCacheLabel();
      setStatus('完成：找到 ' + found + ' 个（上限 ' + state.count + '），' + area +
        '，' + portals.length + ' 个 portal，用时 ' + secs.toFixed(1) + ' s');
    }

    function fetchRadius(radiusKm, cb) {
      var ids = tilesForDisc(center.lat, center.lng, radiusKm * 1000);
      if (!ids || !window.postAjax) { cb(); return; }
      setStatus('拉取数据：半径 ' + radiusKm + ' km …');
      fetchTiles(ids, function (done, total) {
        if (total) setStatus('拉取数据：半径 ' + radiusKm + ' km，' + done + '/' + total + ' 个瓦片 …');
      }).then(function (stats) {
        mergeStats(fetchStats, stats);
        cb();
      });
    }

    function attempt() {
      var radiusKm = Math.min(steps[idx], maxKm);
      fetchRadius(radiusKm, function () {
        var pack = allPortals();
        var found = countQualifying(pack.portals, center, maxKm * 1000, state.titleFilter);
        if (found < state.count && radiusKm < maxKm && idx < steps.length - 1) {
          idx++;
          attempt();
          return;
        }
        var stopKm = radiusKm;
        var fetchKm = radiusKm;
        if (found >= state.count && radiusKm < maxKm) {
          fetchKm = Math.min(nextStepAfter(stopKm), maxKm);
        }
        if (fetchKm > stopKm) {
          fetchRadius(fetchKm, function () {
            setStatus('计算中（' + allPortals().portals.length + ' 个 portal）…');
            finish(stopKm, fetchKm, found);
          });
        } else {
          setStatus('计算中（' + pack.portals.length + ' 个 portal）…');
          finish(stopKm, fetchKm, found);
        }
      });
    }

    attempt();
  }

  // ---------- rendering ----------

  function renderValidation(v) {
    var el = $('drone-l8-validate');
    if (!el) return;
    el.textContent = '';
    var line = document.createElement('div');
    line.className = 'drone-l8-vline';
    line.textContent = '校验：瓦片 ' + v.tilesFetched + '/' + v.tilesExpected +
      ' · 边界采样 ' + (v.sampleChecks - v.sampleMiss) + '/' + v.sampleChecks +
      ' · 重复 ' + v.duplicates + ' · 坏记录 ' + v.badRecords +
      (v.retried ? ' · 重试 ' + v.retried : '');
    el.appendChild(line);
    v.warnings.forEach(function (w) {
      var wl = document.createElement('div');
      wl.className = 'drone-l8-warn';
      wl.textContent = '⚠ ' + w;
      el.appendChild(wl);
    });
  }

  function render(res, nPortals, stopKm, fetchKm, found, secs, v, flags) {
    var box = $('drone-l8-results');
    if (!box) return;
    box.textContent = '';
    renderValidation(v);

    var head = document.createElement('div');
    head.className = 'drone-l8-meta';
    var areaText = fetchKm > stopKm ? ('半径 ' + stopKm + ' km / 外扩 ' + fetchKm + ' km') : ('半径 ' + stopKm + ' km');
    head.textContent = '找到 ' + found + ' 个符合条件 L8（上限 ' + state.count + '）· ' + areaText +
      ' / 距离上限 ' + state.maxKm + ' km · ' + nPortals + ' 个 portal · ' + secs.toFixed(1) + ' s';
    box.appendChild(head);

    var h1 = document.createElement('div');
    h1.className = 'drone-l8-h';
    h1.textContent = '离中心最近的 L8';
    box.appendChild(h1);

    var table = document.createElement('table');
    table.className = 'drone-l8-table';
    res.nearest.forEach(function (r) {
      var tr = document.createElement('tr');
      tr.className = 'drone-l8-clickable';
      tr.title = '点击跳转到该 portal';
      var name = document.createElement('td');
      name.textContent = r.title + (flags[r.idx] ? ' ⚠贴边' : '');
      tr.appendChild(name);
      var team = document.createElement('td');
      team.appendChild(teamSpan(r.team));
      tr.appendChild(team);
      tr.appendChild(td(Math.round(r.dist) + ' m', 'drone-l8-num'));
      tr.appendChild(td(r.hops >= 0 ? r.hops + ' 跳' : '不可达', 'drone-l8-num'));
      var path = r.hops >= 0 ? Core.pathForward(res.fparent, r.idx) : null;
      tr.onclick = function () { jumpTo(res.portals, r.idx, path, '#ffd400'); };
      table.appendChild(tr);
    });
    box.appendChild(table);

    if (res.reachable.length) {
      var h2 = document.createElement('div');
      h2.className = 'drone-l8-h';
      h2.textContent = '可达 L8（按跳数）';
      box.appendChild(h2);
      var t2 = document.createElement('table');
      t2.className = 'drone-l8-table';
      res.reachable.forEach(function (r) {
        var q = res.portals[r.idx];
        var tr = document.createElement('tr');
        tr.className = 'drone-l8-clickable';
        var n1 = document.createElement('td');
        n1.textContent = q.title + (flags[r.idx] ? ' ⚠贴边' : '');
        tr.appendChild(n1);
        tr.appendChild(td(r.hops + ' 跳', 'drone-l8-num'));
        tr.onclick = function () { jumpTo(res.portals, r.idx, r.path, '#ffd400'); };
        t2.appendChild(tr);
      });
      box.appendChild(t2);
    }

    if (res.suggestions.length) {
      var h3 = document.createElement('div');
      h3.className = 'drone-l8-h';
      h3.textContent = '不可达的最近 L8：替代起点';
      box.appendChild(h3);

      res.suggestions.forEach(function (s) {
        var target = res.portals[s.target];
        var wrap = document.createElement('div');
        wrap.className = 'drone-l8-suggest';

        var t = document.createElement('div');
        t.className = 'drone-l8-target drone-l8-clickable';
        t.textContent = '目标 ' + target.title + '（离中心 ' + Math.round(s.distFromCenter) + ' m）';
        t.onclick = function () { jumpTo(res.portals, s.target, null, '#f80'); };
        wrap.appendChild(t);

        if (!s.nearestStarts.length) {
          var none = document.createElement('div');
          none.className = 'drone-l8-start';
          none.textContent = '没有任何 portal 能飞到它（加载范围内无链路）';
          wrap.appendChild(none);
        }
        [['离你最近', s.nearestStarts], ['效率最高', s.fastestStarts]].forEach(function (pair) {
          pair[1].forEach(function (st, k) {
            var p = res.portals[st.idx];
            var line = document.createElement('div');
            line.className = 'drone-l8-start drone-l8-clickable';
            line.textContent = pair[0] + (k === 0 ? '' : ' #' + (k + 1)) + '起点：' + p.title +
              '（离你 ' + Math.round(st.walk) + ' m，到目标 ' + st.hops + ' 跳）';
            line.onclick = function () {
              clearLayers();
              drawPath(res.portals, st.path, k === 0 ? '#f80' : '#fa0', '起点路线：' + st.hops + ' 跳');
            };
            wrap.appendChild(line);
          });
        });
        box.appendChild(wrap);
      });
    }
  }

  // ---------- panel ----------

  function buildPanel() {
    if ($('drone-l8-panel')) return;

    var panel = document.createElement('div');
    panel.id = 'drone-l8-panel';
    panel.className = 'plugin';
    panel.innerHTML =
      '<div class="drone-l8-title">bfs.iitc.misaka</div>' +
      '<div id="drone-l8-center" class="drone-l8-center">中心：未选中 portal</div>' +
      '<div id="drone-l8-sel" class="drone-l8-center"></div>' +
      '<div class="drone-l8-row">' +
      '  <label>数量 <input id="drone-l8-count" type="number" min="1" max="50" value="5"></label>' +
      '  <label>距离 ≤ <input id="drone-l8-maxkm" type="number" min="1" max="20" value="6"> km</label>' +
      '</div>' +
      '<div class="drone-l8-row">' +
      '  <label>推荐 <input id="drone-l8-suggest" type="number" min="0" max="10" value="3"></label>' +
      '  <input id="drone-l8-filter" type="text" placeholder="标题过滤（可空，如 ITO EN）">' +
      '</div>' +
      '<div class="drone-l8-row">' +
      '  <button id="drone-l8-run">分析</button>' +
      '  <button id="drone-l8-me">我在地图中心</button>' +
      '  <button id="drone-l8-clear">清除</button>' +
      '</div>' +
      '<div class="drone-l8-row">' +
      '  <span id="drone-l8-cache" class="drone-l8-meta"></span>' +
      '  <button id="drone-l8-clear-cache">清除缓存</button>' +
      '</div>' +
      '<div id="drone-l8-status"></div>' +
      '<div id="drone-l8-validate"></div>' +
      '<div id="drone-l8-results"></div>';

    var host = $('sidebar') || document.body;
    host.appendChild(panel);

    var style = document.createElement('style');
    style.textContent =
      '#drone-l8-panel{padding:6px;font-size:12px;line-height:1.45;color:inherit}' +
      '#drone-l8-panel .drone-l8-title{font-weight:bold;font-size:13px;margin-bottom:4px}' +
      '#drone-l8-panel .drone-l8-center{opacity:.85;margin-bottom:4px;word-break:break-all}' +
      '#drone-l8-panel .drone-l8-row{display:flex;gap:4px;margin:4px 0;align-items:center;flex-wrap:wrap}' +
      '#drone-l8-panel input[type=number]{width:48px}' +
      '#drone-l8-panel input[type=text]{flex:1;min-width:0}' +
      '#drone-l8-panel .drone-l8-h{margin:8px 0 3px;font-weight:bold;border-bottom:1px solid rgba(255,255,255,.2)}' +
      '#drone-l8-panel .drone-l8-meta{opacity:.75;margin:4px 0}' +
      '#drone-l8-panel .drone-l8-table{width:100%;border-collapse:collapse}' +
      '#drone-l8-panel .drone-l8-table td{padding:1px 2px;border-bottom:1px dotted rgba(255,255,255,.12);vertical-align:top}' +
      '#drone-l8-panel .drone-l8-num{text-align:right;white-space:nowrap}' +
      '#drone-l8-panel .drone-l8-clickable{cursor:pointer}' +
      '#drone-l8-panel .drone-l8-clickable:hover{background:rgba(255,255,255,.12)}' +
      '#drone-l8-panel .drone-l8-suggest{margin:6px 0;padding:4px;border:1px solid rgba(255,255,255,.15);border-radius:4px}' +
      '#drone-l8-panel .drone-l8-target{font-weight:bold}' +
      '#drone-l8-panel .drone-l8-start{margin-top:2px}' +
      '#drone-l8-panel #drone-l8-status{margin:4px 0;opacity:.85;word-break:break-all}' +
      '#drone-l8-panel #drone-l8-validate{margin:2px 0;font-size:11px}' +
      '#drone-l8-panel .drone-l8-vline{opacity:.75}' +
      '#drone-l8-panel .drone-l8-warn{color:#fa0;margin-top:1px}';
    document.head.appendChild(style);

    $('drone-l8-run').onclick = function () {
      state.count = Math.max(1, Math.min(50, parseInt($('drone-l8-count').value, 10) || 5));
      state.maxKm = Math.max(1, Math.min(20, parseInt($('drone-l8-maxkm').value, 10) || 6));
      state.suggest = Math.max(0, Math.min(10, parseInt($('drone-l8-suggest').value, 10) || 0));
      state.titleFilter = ($('drone-l8-filter').value || '').trim();
      var guid = state.selectedGuid || window.selectedPortal || state.centerGuid;
      if (guid) state.centerGuid = guid;
      run();
    };
    $('drone-l8-me').onclick = function () {
      var c = window.map.getCenter();
      state.meLat = c.lat; state.meLng = c.lng;
      setStatus('已把地图中心设为你：' + c.lat.toFixed(5) + ',' + c.lng.toFixed(5));
      run();
    };
    $('drone-l8-clear').onclick = function () {
      clearLayers();
      setStatus('');
    };
    $('drone-l8-clear-cache').onclick = function () {
      clearPersistentCache();
      setStatus('持久缓存已清除');
    };
    updateCacheLabel();
  }

  function hook() {
    if (!window.addHook) return;
    window.addHook('portalSelected', function (data) {
      var guid = (data && data.selectedPortalGuid) || window.selectedPortal;
      if (!guid) return;
      state.selectedGuid = guid;
      selectionLabel();
    });
  }

  PLUGIN.setup = function () {
    pruneExpired();
    ensureLayers();
    buildPanel();
    hook();
    state.selectedGuid = window.selectedPortal || null;
    centerLabel();
    selectionLabel();
    setStatus('选中 portal 后点“分析”');
  };
})();


  var setup = window.plugin.droneL8.setup;
  setup.info = plugin_info;
  if (!window.bootPlugins) window.bootPlugins = [];
  window.bootPlugins.push(setup);
  if (window.iitcLoaded && typeof setup === 'function') setup();
}

var info = {};
if (typeof GM_info !== 'undefined' && GM_info.script) {
  info.script = {
    version: GM_info.script.version,
    name: GM_info.script.name,
    description: GM_info.script.description
  };
}
var script = document.createElement('script');
script.appendChild(document.createTextNode('(' + wrapper + ')(' + JSON.stringify(info) + ');'));
(document.body || document.head || document.documentElement).appendChild(script);
