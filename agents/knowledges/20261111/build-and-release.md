# 构建与发布注意

- `build.py` 的 `parts` 是硬编码顺序；新增 `src/*.js` 必须同步加入。
  已加检查：缺失文件或未登记的 src 文件会直接报错，避免静默漏内容。
- `dist/bfs.iitc.misaka.user.js` **直接提交入库**，安装走 GitHub raw；没有 CI、没有 dist 分支、没有自动发布。
- 根目录禁止出现同名 `bfs.iitc.misaka.user.js`（`.gitignore` 已挡），避免和 `dist/` 产物混淆。
- userscript header 默认写入 `@downloadURL` / `@updateURL`，指向
  `https://raw.githubusercontent.com/Misaka-0x447f/iitc.misaka/master/dist/bfs.iitc.misaka.user.js`，
  靠 `@version`（UTC 日期时间）触发 Tampermonkey 自动更新；可用同名环境变量覆盖。
- `tests/fixtures/` 只有数据；自动化只有 `tests/smoke.js`（S2 角点对照 + 合成数据 BFS）。
- `@version` 默认取 UTC 日期时间 `YYYY.MM.DD.HHMMSS`，可用 `DRONE_L8_VERSION` 覆盖。
- `@id` / `@name` 固定为 `bfs.iitc.misaka`，发布后不要改；`@namespace` / `@author` 由环境变量注入，
  本地默认 `https://github.com/Misaka-0x447f/iitc.misaka` / `Misaka-0x447f`。
- `.gitignore` 忽略：根目录同名产物、Python/Node 缓存与虚拟环境、OS/编辑器文件、`tmp/`、日志与 `.env`。
- Release 附件链接（`releases/latest/download/...`）实测只能触发下载，不会弹 Tampermonkey 安装页；
  一键安装用 raw dist 链接。
