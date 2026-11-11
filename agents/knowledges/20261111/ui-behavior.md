# UI 行为与缓存

- 结果点击：跳转到对应 portal（`map.setView` 到 17 级），有路线时同时画线；不改变搜索中心。
- 结果保留：`portalSelected` 只更新“选中”行，不自动重跑；只有点“分析”才重算，
  所以切换选中、跳转地图都不会清空当前结果列表。
- 缓存：插件自有的内存 `tileCache`（key = tile id），按页面会话存活，无 TTL、不持久化；
  不复用 IITC 的 `mapDataRequest.cache`，也不经过 IITC 的渲染缓存。
- 换中心/换区域只补拉缺失瓦片；刷新页面后缓存清空，全部重拉。
