#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_route.py — 以 IITC 中选中的 portal 为中心做无人机规划：

  1) 列出离中心最近的 L8 portal（直线距离）
  2) 从中心 portal 出发，无人机可达的 L8（有向图，one-way 可接受）
  3) 若某个 L8 从中心不可达，推荐替代起点：
       - 离你最近的可行起点 portal
       - 路线效率最高的起点 portal（跳数最少）

可见集模型（社区实测，IITC Drone Helper 同款）：
  无人机停在 A 时，Scanner 载入「以 A 为圆心、500m 圆盘覆盖到的全部
  S2 L16 cell」里的 portal。=> B 可选 <=> cell16(B) ∈ cover16(A)。
  最远约 700m；A→B 是有向边，存在 one-way。

用法:
  python3 drone_route.py area.json              # 直接读 area.json 里的 selectedPortal
  python3 drone_route.py area.json --center-guid <guid>
  python3 drone_route.py area.json --me-lat 1.3 --me-lng 103.8
"""
import argparse
import json
import math
import sys
from collections import defaultdict, deque

EARTH_R = 6371010.0
S2_OK = True
try:
    import s2sphere
except ImportError:
    S2_OK = False

TEAM = {0: "中立", 1: "绿", 2: "蓝"}


def hav(lat1, lng1, lat2, lng2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lng2 - lng1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


class Planner:
    def __init__(self, portals, jump_radius=500.0):
        self.p = portals
        self.n = len(portals)
        self.jump_radius = jump_radius
        self._cell = [None] * self.n
        self._cover = [None] * self.n
        self._index = defaultdict(list)
        if S2_OK:
            for i, q in enumerate(portals):
                c = s2sphere.CellId.from_lat_lng(
                    s2sphere.LatLng.from_degrees(q["lat"], q["lng"])
                ).parent(16)
                self._cell[i] = c.id()
                self._index[c.id()].append(i)

    def cover(self, i):
        if self._cover[i] is None:
            q = self.p[i]
            ll = s2sphere.LatLng.from_degrees(q["lat"], q["lng"])
            cap = s2sphere.Cap.from_axis_angle(
                ll.to_point(), s2sphere.Angle.from_radians(self.jump_radius / EARTH_R)
            )
            rc = s2sphere.RegionCoverer()
            rc.min_level = 16
            rc.max_level = 16
            rc.max_cells = 4096
            self._cover[i] = [c.id() for c in rc.get_covering(cap)]
        return self._cover[i]

    def neighbors(self, i):
        if S2_OK:
            out = []
            for c in self.cover(i):
                out.extend(self._index.get(c, ()))
            return [j for j in out if j != i]
        q = self.p[i]
        return [
            j for j, r in enumerate(self.p)
            if j != i and hav(q["lat"], q["lng"], r["lat"], r["lng"]) <= 700.0
        ]


def bfs(adj, start):
    """返回 (dist, parent)；parent[x] 是从 start 方向发现 x 的节点。"""
    dist = {start: 0}
    parent = {start: None}
    dq = deque([start])
    while dq:
        u = dq.popleft()
        for v in adj[u]:
            if v not in dist:
                dist[v] = dist[u] + 1
                parent[v] = u
                dq.append(v)
    return dist, parent


def path_from(nxt, x, target):
    """从 x 沿 nxt 走到 target（nxt[x] 是 x 的下一跳）。"""
    seq = [x]
    while seq[-1] != target:
        y = nxt.get(seq[-1])
        if y is None:
            break
        seq.append(y)
    return seq


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    selected = None
    if isinstance(raw, dict):
        selected = raw.get("selectedPortal")
        raw = raw.get("portals", [])
    out = []
    for q in raw:
        if "lat" not in q or "lng" not in q:
            continue
        out.append(
            {
                "guid": q.get("guid", ""),
                "title": q.get("title", "") or "",
                "lat": float(q["lat"]),
                "lng": float(q["lng"]),
                "level": int(q.get("level", 0)),
                "team": int(q.get("team", 0)),
            }
        )
    return out, selected


def intel(lat, lng):
    return f"https://intel.ingress.com/intel?ll={lat:.6f},{lng:.6f}&z=17&pll={lat:.6f},{lng:.6f}"


def name(q):
    return q["title"] or q["guid"][:8]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json_file")
    ap.add_argument("--center-guid", default=None, help="中心 portal 的 guid（默认用文件里的 selectedPortal）")
    ap.add_argument("--me-lat", type=float, default=None)
    ap.add_argument("--me-lng", type=float, default=None)
    ap.add_argument("--top", type=int, default=10, help="列多少个 L8")
    ap.add_argument("--suggest", type=int, default=3, help="对多少个不可达 L8 做替代起点推荐")
    ap.add_argument("--title-contains", default=None, help="只考虑标题包含该子串的 L8")
    ap.add_argument("--jump-radius", type=float, default=500.0)
    ap.add_argument("--pairs", action="store_true", help="额外输出互相 1 跳的 L8 对")
    args = ap.parse_args()

    portals, selected = load(args.json_file)
    if not portals:
        sys.exit("没有读到 portal")

    # 中心
    center = None
    if args.center_guid:
        for i, q in enumerate(portals):
            if q["guid"] == args.center_guid:
                center = i
                break
        if center is None:
            sys.exit(f"guid {args.center_guid} 不在数据里")
    elif selected:
        for i, q in enumerate(portals):
            if q["guid"] == selected:
                center = i
                break
    if center is None:
        if args.me_lat is None or args.me_lng is None:
            sys.exit("没有 selectedPortal，也没有 --center-guid / --me-lat,lng")
        center = min(range(len(portals)),
                     key=lambda i: hav(args.me_lat, args.me_lng, portals[i]["lat"], portals[i]["lng"]))

    c = portals[center]
    me_lat = args.me_lat if args.me_lat is not None else c["lat"]
    me_lng = args.me_lng if args.me_lng is not None else c["lng"]

    P = Planner(portals, args.jump_radius)
    if not S2_OK:
        print("[warn] 没装 s2sphere，用 700m 直线半径近似\n")

    print(f"中心 portal（IITC 选中）: {name(c)}  L{c['level']} {TEAM.get(c['team'],'?')}")
    print(f"  {c['lat']:.6f},{c['lng']:.6f}   {intel(c['lat'], c['lng'])}")
    if args.me_lat is None:
        print("  （没给 --me-lat/lng，\"离你\"按中心 portal 算）")
    print()

    # 图（全量，方向 A->B）
    adj = [P.neighbors(i) for i in range(P.n)]
    radj = [[] for _ in range(P.n)]
    for a, ns in enumerate(adj):
        for b in ns:
            radj[b].append(a)

    l8 = [i for i, q in enumerate(portals) if q["level"] == 8]
    if args.title_contains:
        l8 = [i for i in l8 if args.title_contains.lower() in portals[i]["title"].lower()]
    print(f"共 {P.n} 个 portal，L8 {len(l8)} 个"
          + (f"（标题含 \"{args.title_contains}\"）" if args.title_contains else ""))
    if not l8:
        print("没有符合条件 的 L8。")
        return

    # 从中心出发
    fdist, fparent = bfs(adj, center)

    # 最近的 L8（按到中心直线距离）
    l8_sorted = sorted(l8, key=lambda i: hav(c["lat"], c["lng"], portals[i]["lat"], portals[i]["lng"]))
    print(f"\n== 最近的 {min(args.top, len(l8_sorted))} 个 L8（按到中心直线距离）==")
    print(f"{'#':>2}  {'标题':<36} {'距离':>7}  {'阵营':<3} {'从中心可达':<10} 跳数")
    for k, i in enumerate(l8_sorted[: args.top], 1):
        q = portals[i]
        d = hav(c["lat"], c["lng"], q["lat"], q["lng"])
        ok = i in fdist
        print(f"{k:>2}  {name(q):<36} {d:>6.0f}m  {TEAM.get(q['team'],'?'):<3} "
              f"{'是' if ok else '否':<10} {str(fdist[i]) + ' 跳' if ok else '-'}")

    # 从中心可达的 L8（按跳数）
    reach = sorted([i for i in l8 if i in fdist], key=lambda i: (fdist[i], hav(c["lat"], c["lng"], portals[i]["lat"], portals[i]["lng"])))
    print("\n== 从中心可达的 L8（按跳数 = 小时数）==")
    if not reach:
        print("  无")
    for i in reach[: args.top]:
        q = portals[i]
        pth = path_from(fparent, i, center)[::-1]  # 正序 center -> ... -> i
        total = sum(hav(portals[pth[k]]["lat"], portals[pth[k]]["lng"],
                        portals[pth[k + 1]]["lat"], portals[pth[k + 1]]["lng"])
                    for k in range(len(pth) - 1))
        print(f"  {name(q)}  L8 {TEAM.get(q['team'],'?')}  {fdist[i]} 跳 / 约 {fdist[i]} 小时 / 路径 {total/1000:.2f} km")
        print(f"     " + " -> ".join(name(portals[k]) for k in pth))

    # 不可达的 L8：推荐替代起点
    unreachable = [i for i in l8_sorted if i not in fdist]
    print(f"\n== 从中心不可达的最近 {min(args.suggest, len(unreachable))} 个 L8：替代起点推荐 ==")
    if not unreachable:
        print("  无（最近的 L8 都能从中心飞到）")
    for t in unreachable[: args.suggest]:
        qt = portals[t]
        print(f"\n目标: {name(qt)}  L8 {TEAM.get(qt['team'],'?')}  "
              f"离中心 {hav(c['lat'], c['lng'], qt['lat'], qt['lng']):.0f} m")
        rdist, rnext = bfs(radj, t)  # rnext[x] = x 到 t 的下一跳
        cands = [i for i in rdist if i != t]
        if not cands:
            print("  没有任何 portal 能飞到它（孤立）")
            continue
        nearest = min(cands, key=lambda i: (hav(me_lat, me_lng, portals[i]["lat"], portals[i]["lng"]), rdist[i]))
        fastest = min(cands, key=lambda i: (rdist[i], hav(me_lat, me_lng, portals[i]["lat"], portals[i]["lng"])))
        for label, i in (("离你最近的起点", nearest), ("效率最高的起点", fastest)):
            q = portals[i]
            walk = hav(me_lat, me_lng, q["lat"], q["lng"])
            print(f"  {label}: {name(q)}  L{q['level']} {TEAM.get(q['team'],'?')}  "
                  f"离你 {walk:.0f} m / 到目标 {rdist[i]} 跳")
            print(f"     {intel(q['lat'], q['lng'])}")
            seq = path_from(rnext, i, t)
            print("     路径: " + " -> ".join(name(portals[k]) for k in seq))

    # 可选：互相 1 跳的 L8 对
    if args.pairs:
        l8set = set(l8)
        pairs = []
        for a in l8:
            if a not in fdist:
                continue
            for b in adj[a]:
                if b in l8set and b > a and b in fdist and a in adj[b]:
                    d = hav(portals[a]["lat"], portals[a]["lng"], portals[b]["lat"], portals[b]["lng"])
                    pairs.append((fdist[a] + fdist[b], d, a, b))
        pairs.sort()
        print("\n== 从中心可达、互相 1 跳的 L8 对 ==")
        for _, d, a, b in pairs[: args.top]:
            qa, qb = portals[a], portals[b]
            print(f"  {name(qa)} <-> {name(qb)}  {d:.0f} m  "
                  f"({TEAM.get(qa['team'],'?')}/{TEAM.get(qb['team'],'?')})  {fdist[a]}+{fdist[b]} 跳")
        if not pairs:
            print("  无")


if __name__ == "__main__":
    main()
