# UI 行为与缓存

- 结果点击：跳转到对应 portal（`map.setView` 到 17 级），有路线时同时画线；不改变搜索中心。
- 结果保留：`portalSelected` 只更新“选中”行，不自动重跑；只有点“分析”才重算，
  所以切换选中、跳转地图都不会清空当前结果列表。
- 缓存：`localStorage` 持久化（key 前缀 `bfs.iitc.misaka.cache.v1.`，按 tile id），TTL 7 天；
  只存解析后的 portal 数组（紧凑数组格式），面板显示体积、提供清除按钮；
  超配额时按最旧优先淘汰。不复用 IITC 的 `mapDataRequest.cache`，也不经过 IITC 的渲染缓存。
- 换中心/换区域只补拉缺失瓦片；刷新页面后仍可从 localStorage 命中。
