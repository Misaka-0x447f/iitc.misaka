# Vendored / legacy 例外

## src/s2cell.js

第三方 S2 几何实现的精简移植，包含 `switch/case`（6 个 cube face 穷尽分支 + default throw）。
AGENTS.md 前端部分禁止 `switch + case`，此处保留为 vendored 例外：

- 面编号映射是穷尽分支，改成 if/return 链更易写错；
- 该文件已与 Python `s2sphere` 的 cell 角点逐点比对通过，重写数学部分风险高于风格收益；
- 若将来重构，必须重新做角点比对再替换。

## src/core.js / src/ui.js

无构建链的经典浏览器脚本：IIFE + UMD 挂到 `window.DroneL8Core`，再由 `build.py` 原样拼接进
userscript 注入页面。大量 `var` / `function` 是刻意选择，用于保证 userscript 注入环境的兼容性。
AGENTS.md 对 `function` / `class` 只要求“减少”，故保留现状；若改用现代语法，需要先解决
Tampermonkey 沙箱与单文件拼接约束，属于一次性架构决定。
