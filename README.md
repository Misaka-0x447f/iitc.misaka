# bfs.iitc.misaka — Ingress 无人机找 L8

`dist/bfs.iitc.misaka.user.js`：在 Intel 页面右侧加一个 “bfs.iitc.misaka” 面板。
以当前选中的 portal 为中心，**按半径直接向 Intel 拉取 portal 数据**（不依赖画面已加载的 portal），
在该数据集上做无人机可达性 BFS，列出最近的 L8、可达路线、以及不可达时的替代起点。

## 结构

- `src/`：源码（`s2cell.js` 几何、`core.js` 建图与 BFS、`ui.js` 面板与拉取）
- `build.py`：拼接 `src/*.js` 生成 `dist/bfs.iitc.misaka.user.js`
- `dist/`：构建产物，**提交入库**，勿手改
- `tools/`：命令行/导出备用路径（`drone_route.py`、`iitc_export.js`）
- `tests/`：冒烟测试与合成数据
- `agents/`：Agent 索引、知识库、设计文档
- `tmp/<YYYYMMDD>/`：本地任务记录（不入库）

## 安装

- 桌面：Tampermonkey（或 Violentmonkey）→ 打开安装地址或本地 `dist/bfs.iitc.misaka.user.js` → 安装 → 刷新 `intel.ingress.com`。
- IITC Mobile：Plugins → Add external plugins → 选文件。

安装地址（公开仓库后）：

```
https://raw.githubusercontent.com/Misaka-0x447f/iitc.misaka/master/dist/bfs.iitc.misaka.user.js
```

## 用法

1. 在 IITC 里点选一个 portal（中心，例如 `ITO EN (-)`）。
2. 点“分析”开始拉数据（选中的 portal 作为中心）；也可先调参数：
   - **数量**：找到多少个符合条件的 L8 就停止扩张；
   - **距离 ≤**：目标 L8 的距离上限（km），也是扩张的硬上限；
   - **推荐**：对多少个“从中心不可达”的 L8 给替代起点；
   - **标题过滤**：只统计标题含该子串的 L8（如 `ITO EN`）。
3. 结果（点击任意结果行可跳转到对应 portal，有路线时同时画线；跳转不会清空结果）：
   - **离中心最近的 L8**：直线距离 + 可达/跳数；
   - **可达 L8（按跳数）**：跳数 = 小时数；
   - **不可达的最近 L8：替代起点**：每个目标给“离你最近”和“效率最高”各 3 个起点。
4. “我在地图中心”：把当前地图中心设为你本人位置（默认用中心 portal）。

## 数据与运行时校验

- 用 `IITC.map.tiles` 计算 `dataZoom=15` 的瓦片，`window.postAjax('getEntities', {tileKeys})` 拉取；
  每瓦片约 1.2 km，含该格全部等级 portal。
- 每请求 5 个瓦片、4 路并发、失败重试 2 次。
- 缓存是插件自有的内存 `tileCache`（按 tile id），只在当前页面会话内有效，无 TTL、不持久化；
  不复用 IITC 的 `mapDataRequest.cache`，也不经过 IITC 的渲染缓存。
- 搜索从 1 km 起按 1→2→3→4→6…km 扩张，找到“数量”个符合条件 L8 或到达距离上限即停；
  停止后再向下一个档位外扩一档（不超过距离上限）补全路径上下文。
- 每次运行自我校验并在面板显示：
  - 瓦片拉取数/预期数、搜索圆 24 点采样覆盖、跨瓦片重复、坏记录；
  - 可达路线贴近拉取边界时标 `⚠贴边`；
  - 最近 L8 不可达时提示可能受边界影响；
  - 详细结果在 `window.plugin.droneL8.lastValidation`。

## 模型与假设

- 无人机在 A 时，Scanner 载入「A 的 500 m 圆盘覆盖到的全部 S2 L16 cell」里的 portal，
  最远可选中约 700 m；A→B 是**有向边**，one-way 可接受。
- 移动 1 次/小时（社区文档，用前在游戏里确认）；跳数 = 小时数。
- 单 portal 4 次 hack 后 burnout；两个 L8 互跳可错开 4h 窗口，长期约 2× 产出。
- 无人机所在 portal 变色会召回无人机 → 选冷门、稳定的 portal。
- 无人机只能 hack，不能 deploy / link / field；掉落等级受 agent 等级和 portal 等级双重限制。
- 未建模：带 key 的 1.25 km 扩展；链接是否阻挡（按社区结论：不阻挡）。
- 瓦片缓存不设 TTL：区域内 portal 归属/等级变化后，需要刷新页面重拉。

## 构建

```
python3 build.py        # 输出 dist/bfs.iitc.misaka.user.js
node tests/smoke.js
```

- 改完源码后重新构建，并把 `dist/` 产物一起提交；没有 CI，也没有 dist 分支。
- 构建会自动写入 `@downloadURL` / `@updateURL`，指向
  `https://raw.githubusercontent.com/Misaka-0x447f/iitc.misaka/master/dist/bfs.iitc.misaka.user.js`，
  Tampermonkey 会按 `@version`（UTC 日期时间）自动检查更新。
- 如需换地址，用 `DRONE_L8_DOWNLOAD_URL` / `DRONE_L8_UPDATE_URL` 覆盖；`DRONE_L8_NAMESPACE` / `DRONE_L8_AUTHOR` 同理。
