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
