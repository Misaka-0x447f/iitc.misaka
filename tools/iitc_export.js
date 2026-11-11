// 在已登录的 IITC (intel.ingress.com) 浏览器控制台执行。
// 前提：先点选目标 portal（作为中心/起点），再把地图平移到目标区域（z 15~16），
// 来回平移/缩放一圈让 tile 加载完，最后跑本脚本。
//
// 输出：{ selectedPortal, portals: [...] } 的 JSON（同时 console.log + 复制到剪贴板）
(() => {
  const out = [];
  for (const guid in window.portals) {
    const p = window.portals[guid];
    const ll = p.getLatLng();
    out.push({
      guid,
      title: (p.options.data && p.options.data.title) || p.options.title || "",
      lat: ll.lat,
      lng: ll.lng,
      level: p.options.level,
      team: p.options.team   // 0=中立 1=绿 2=蓝
    });
  }
  const payload = {
    selectedPortal: window.selectedPortal || null,  // IITC 当前选中的 portal guid
    portals: out
  };
  const s = JSON.stringify(payload);
  console.log(s);
  copy(s);
  return out.length + " portals, selected=" + payload.selectedPortal;
})();
