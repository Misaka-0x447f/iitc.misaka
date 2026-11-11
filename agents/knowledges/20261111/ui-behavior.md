# UI 行为与缓存

- 结果点击：跳转到对应 portal（`map.setView` 到 17 级），有路线时同时画线；不改变搜索中心。
- 结果保留：`portalSelected` 只更新“选中”行，不自动重跑；只有点“分析”才重算，
  所以切换选中、跳转地图都不会清空当前结果列表。
- 缓存：`IndexedDB` 持久化（库 `bfs.iitc.misaka`；stores `tiles`、`details`、`meta`），TTL 7 天；
  tile 存紧凑 portal 数组，details 存 mod 列表，`meta.bytes` 维护体积计数；面板显示体积、提供清除按钮。
  不复用 IITC 的 `mapDataRequest.cache`，也不经过 IITC 的渲染缓存。
- 换中心/换区域只补拉缺失瓦片；刷新页面后仍可从 localStorage 命中。

- mod 标签：分析完成后对展示的 L8 异步请求 `getPortalDetails`（最多 30 个、3 并发），
  命中 `Ito En Transmuter (-)` / `(+)` 显示标签；details 持久缓存 7 天，与 tile 缓存一起计入体积与清除。
