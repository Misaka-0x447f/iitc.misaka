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

  var state = {
    centerGuid: null,
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

  function drawPath(portals, idxs, color, label) {
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
    if (idxs.length > 1) window.map.fitBounds(L.latLngBounds(latlngs), { padding: [40, 40] });
    else window.map.setView(latlngs[0], 17);
    if (label) setStatus(label);
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
    var pending = ids.filter(function (id) { return !(id in tileCache); });
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
            tileCache[id] = parseTile(res.map, id);
            tileCount++;
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

  function centerLabel() {
    var el = $('drone-l8-center');
    if (!el) return;
    if (!state.centerGuid) { el.textContent = '中心：未选中 portal'; return; }
    var m = (window.portals || {})[state.centerGuid];
    var title = m && m.options && m.options.data && m.options.data.title;
    el.textContent = '中心：' + (title || state.centerGuid.substring(0, 8));
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
    var center = centerInfo();
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
      if (r.hops >= 0) tr.title = '点击画出从中心出发的路线';
      var name = document.createElement('td');
      name.textContent = r.title + (flags[r.idx] ? ' ⚠贴边' : '');
      tr.appendChild(name);
      var team = document.createElement('td');
      team.appendChild(teamSpan(r.team));
      tr.appendChild(team);
      tr.appendChild(td(Math.round(r.dist) + ' m', 'drone-l8-num'));
      tr.appendChild(td(r.hops >= 0 ? r.hops + ' 跳' : '不可达', 'drone-l8-num'));
      if (r.hops >= 0) {
        tr.className = 'drone-l8-clickable';
        tr.setAttribute('data-idx', r.idx);
        tr.onclick = function () {
          clearLayers();
          drawPath(res.portals, Core.pathForward(res.fparent, parseInt(this.getAttribute('data-idx'), 10)), '#ffd400', '路线：' + r.hops + ' 跳');
        };
      }
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
        tr.onclick = function () {
          clearLayers();
          drawPath(res.portals, r.path, '#ffd400', '路线：' + r.hops + ' 跳');
        };
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
        t.className = 'drone-l8-target';
        t.textContent = '目标 ' + target.title + '（离中心 ' + Math.round(s.distFromCenter) + ' m）';
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
  }

  function hook() {
    if (!window.addHook) return;
    window.addHook('portalSelected', function (data) {
      var guid = (data && data.selectedPortalGuid) || window.selectedPortal;
      if (!guid) return;
      state.centerGuid = guid;
      centerLabel();
      if (document.getElementById('drone-l8-panel')) run();
    });
  }

  PLUGIN.setup = function () {
    ensureLayers();
    buildPanel();
    hook();
    if (window.selectedPortal) state.centerGuid = window.selectedPortal;
    centerLabel();
    if (state.centerGuid) run();
  };
})();
