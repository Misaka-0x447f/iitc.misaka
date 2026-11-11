#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 src/ 拼成可在 IITC 里安装的 userscript（bfs.iitc.misaka）。

环境变量（CI 用，本地可省略）：
  DRONE_L8_VERSION       写入 @version
  DRONE_L8_DOWNLOAD_URL  写入 @downloadURL
  DRONE_L8_UPDATE_URL    写入 @updateURL
"""
import os
import pathlib
from datetime import datetime, timezone

root = pathlib.Path(__file__).parent
parts = [root / 'src' / n for n in ('s2cell.js', 'core.js', 'ui.js')]

missing = [p.name for p in parts if not p.exists()]
if missing:
    raise SystemExit('missing src files: ' + ', '.join(missing))

registered = {p.name for p in parts}
extra = sorted(p.name for p in (root / 'src').glob('*.js') if p.name not in registered)
if extra:
    raise SystemExit('unregistered src files, add them to parts: ' + ', '.join(extra))

body = '\n\n'.join(p.read_text(encoding='utf-8') for p in parts)

version = os.environ.get('DRONE_L8_VERSION') or datetime.now(timezone.utc).strftime('%Y.%m.%d.%H%M%S')
namespace = os.environ.get('DRONE_L8_NAMESPACE', 'https://github.com/Misaka-0x447f/iitc.misaka')
author = os.environ.get('DRONE_L8_AUTHOR', 'Misaka-0x447f')
urls = ''
RAW_URL = 'https://raw.githubusercontent.com/Misaka-0x447f/iitc.misaka/master/dist/bfs.iitc.misaka.user.js'
download_url = os.environ.get('DRONE_L8_DOWNLOAD_URL', RAW_URL)
update_url = os.environ.get('DRONE_L8_UPDATE_URL', RAW_URL)
if download_url:
    urls += '// @downloadURL    ' + download_url + '\n'
if update_url:
    urls += '// @updateURL      ' + update_url + '\n'

header = '''// ==UserScript==
// @id             bfs.iitc.misaka
// @name           bfs.iitc.misaka
// @category       Info
// @version        __VERSION__
// @namespace      __NAMESPACE__
__URLS__// @description    以选中 portal 为中心：最近 L8、无人机可达路线、不可达时的替代起点
// @author         __AUTHOR__
// @match          https://intel.ingress.com/*
// @grant          none
// ==/UserScript==

function wrapper(plugin_info) {
  if (typeof window.plugin !== 'function') window.plugin = function () {};

''' + body + '''

  var setup = window.plugin.droneL8.setup;
  setup.info = plugin_info;
  if (!window.bootPlugins) window.bootPlugins = [];
  window.bootPlugins.push(setup);
  if (window.iitcLoaded && typeof setup === 'function') setup();
}

var info = {};
if (typeof GM_info !== 'undefined' && GM_info.script) {
  info.script = {
    version: GM_info.script.version,
    name: GM_info.script.name,
    description: GM_info.script.description
  };
}
var script = document.createElement('script');
script.appendChild(document.createTextNode('(' + wrapper + ')(' + JSON.stringify(info) + ');'));
(document.body || document.head || document.documentElement).appendChild(script);
'''

header = (header.replace('__VERSION__', version)
          .replace('__NAMESPACE__', namespace)
          .replace('__AUTHOR__', author)
          .replace('__URLS__', urls))

out = root / 'dist' / 'bfs.iitc.misaka.user.js'
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(header, encoding='utf-8')
print('wrote', out, len(header.encode('utf-8')), 'bytes, version', version)
