#!/usr/bin/env python3
"""
Web 控制台（内嵌于 simple_udp_receiver.py）
==========================================
在接收器进程里起一个 HTTP 服务（:20005），浏览器打开即可操作：
  - 设备表（ID/IP/版本/最新|待升级|离线）+ 逐台/全部 OTA 升级按钮
  - 检测时段设置（等价键盘命令 s ...）
  - 坐姿统计：按设备 + 时间范围（今日/本周/本月/今年/全部），
    KPI 数字块 + 分桶柱形图（检测/不良两系列，SVG 手绘）+ 数据表
  - 运行日志面板（把控制台 print 镜像一份，OTA 进度等直接可见）

与命令行完全同一条路径：网页按钮 → cmd_dispatcher（即接收器的 handle_command），
与键盘输入等价；状态/统计来自 state_provider/stats_provider 快照，定时轮询刷新。
零第三方依赖（标准库 http.server），同局域网内手机也能打开。

图表配色取自 dataviz 参考调色板前两个已验证槽位（蓝 #2a78d6 / 橙 #eb6834，
CVD ΔE 24.7、对白底对比度 ≥3:1，validate_palette.js 全项通过）。
"""

import builtins
import collections
import json
import os
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

WEB_PORT = 20005  # 20000-20004 已用（图像/音频/结果/发现/OTA下载），控制台接 20005

_state_provider = None   # () → dict（设备表/调度/OTA 状态快照，接收器注入）
_cmd_dispatcher = None   # (cmd: str) → None（与控制台 input 完全同一条路径）
_stats_provider = None   # (range_key, device) → dict（坐姿统计快照，接收器注入）


def set_stats_provider(fn):
    global _stats_provider
    _stats_provider = fn


# 控制台镜像：print 钩子写入环形缓冲，/api/state 读出（deque.append 原子，线程安全）
_log = collections.deque(maxlen=300)
_orig_print = builtins.print


def _mirror_print(*args, **kwargs):
    """把 print 输出镜像进环形缓冲供网页显示，原样再打到控制台"""
    try:
        _log.append(" ".join(str(a) for a in args))
    except Exception:
        pass
    _orig_print(*args, **kwargs)


def _install_mirror():
    builtins.print = _mirror_print


_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP 坐姿检测控制台</title>
<style>
  :root {
    --series-1: #2a78d6;  /* 检测次数（dataviz 已验证槽位 1） */
    --series-2: #eb6834;  /* 不良次数（dataviz 已验证槽位 2） */
    --series-3: #1baf7a;  /* 不良率曲线（dataviz 槽位 3；三槽组合 validate_palette 全 PASS，
                            对表面 2.74:1 的 WARN 由数据表/tooltip/右刻度缓解） */
    --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
    --grid: #e1e0d9; --axis: #c3c2b7;
  }
  * { box-sizing: border-box; margin: 0; }
  body { font-family: system-ui, "Microsoft YaHei", sans-serif; background: #f5f6f8; color: #1f2937; }
  .wrap { max-width: 880px; margin: 0 auto; padding: 28px 20px 60px; }
  header { display: flex; align-items: flex-start; justify-content: space-between; margin-bottom: 20px; }
  h1 { font-size: 20px; font-weight: 600; }
  .sub { color: #6b7280; font-size: 13px; margin-top: 4px; max-width: 620px; }
  .badge { font-size: 12px; color: #2563eb; background: #eff6ff; border: 1px solid #bfdbfe;
           border-radius: 99px; padding: 2px 10px; white-space: nowrap; margin-top: 2px; }
  .card { background: #fff; border: 1px solid #e5e7eb; border-radius: 10px; padding: 16px 18px; margin-bottom: 16px; }
  .card h2 { font-size: 14px; font-weight: 600; color: #374151; margin-bottom: 12px;
             display: flex; justify-content: space-between; align-items: center; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th { text-align: left; color: #6b7280; font-weight: 500; padding: 6px 8px; border-bottom: 1px solid #e5e7eb; }
  td { padding: 8px; border-bottom: 1px solid #f3f4f6; }
  tr:last-child td { border-bottom: none; }
  .chip { display: inline-block; padding: 1px 8px; border-radius: 99px; font-size: 12px; }
  .ok  { color: #16a34a; background: #f0fdf4; }
  .up  { color: #d97706; background: #fffbeb; }
  .off { color: #9ca3af; background: #f3f4f6; }
  .upg { color: #2563eb; background: #eff6ff; }
  .prog { display: inline-block; height: 6px; width: 84px; background: #e5e7eb;
          border-radius: 3px; overflow: hidden; flex: none; }
  .prog-fill { display: block; height: 100%; background: #2563eb; border-radius: 3px; transition: width .5s; }
  button { font: inherit; font-size: 13px; border-radius: 8px; border: 1px solid #d1d5db;
           background: #fff; color: #374151; padding: 5px 14px; cursor: pointer; }
  button:hover { background: #f9fafb; }
  button.primary { background: #2563eb; border-color: #2563eb; color: #fff; }
  button.primary:hover { background: #1d4ed8; }
  button.mini { padding: 2px 10px; font-size: 12px; }
  select.cell-sel { font: inherit; font-size: 12px; padding: 2px 4px; border: 1px solid #d1d5db;
                    border-radius: 6px; background: #fff; color: #374151; cursor: pointer; }
  .empty { color: #9ca3af; font-size: 13px; padding: 12px 0; text-align: center; }
  .row { display: flex; gap: 8px; }
  input[type=text] { flex: 1; font: inherit; font-size: 13px; padding: 6px 10px;
                     border: 1px solid #d1d5db; border-radius: 8px; }
  input[type=text]:focus { outline: 2px solid #bfdbfe; border-color: #2563eb; }
  .sched-now { font-size: 13px; color: #374151; margin-bottom: 10px; }
  .sched-now b { font-weight: 600; }
  .sched-hint { font-size: 12px; color: #9ca3af; margin-top: 8px; }
  .dim { color: #9ca3af; }
  .lbl { font-size: 13px; color: #6b7280; }
  input[type=number] { font: inherit; font-size: 13px; padding: 5px 8px;
                       border: 1px solid #d1d5db; border-radius: 8px; width: 64px; }
  pre#log { background: #0f172a; color: #cbd5e1; font-size: 12px; line-height: 1.55;
            padding: 12px 14px; border-radius: 8px; height: 300px; overflow: auto;
            white-space: pre-wrap; word-break: break-all; }
  .ota-banner { font-size: 13px; color: #1d4ed8; background: #eff6ff; border: 1px solid #bfdbfe;
                padding: 6px 12px; border-radius: 8px; margin-bottom: 16px; display: none; }

  /* ---- 坐姿统计 ---- */
  .filter-row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; margin-bottom: 14px; }
  select { font: inherit; font-size: 13px; padding: 5px 8px; border: 1px solid #d1d5db;
           border-radius: 8px; background: #fff; color: #374151; }
  .seg { display: inline-flex; border: 1px solid #d1d5db; border-radius: 8px; overflow: hidden; }
  .seg button { border: none; border-radius: 0; padding: 5px 12px; }
  .seg button + button { border-left: 1px solid #e5e7eb; }
  .seg button.on { background: #eff6ff; color: #1d4ed8; font-weight: 600; }
  .tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(128px, 1fr)); gap: 10px; margin-bottom: 14px; }
  .tile { background: #f9fafb; border: 1px solid #f3f4f6; border-radius: 8px; padding: 10px 12px; }
  .tlabel { font-size: 12px; color: #6b7280; margin-bottom: 4px; }
  .tval { font-size: 20px; font-weight: 600; color: #111827; }
  .legend { display: flex; gap: 16px; font-size: 12px; color: var(--ink-2); margin-bottom: 6px; }
  .legend .sw { display: inline-block; width: 12px; height: 12px; border-radius: 3px;
                margin-right: 5px; vertical-align: -1px; }
  .legend .sw-line { display: inline-block; width: 14px; height: 0; border-top: 2px solid;
                     border-radius: 2px; margin-right: 5px; vertical-align: 1px; }
  #statsBody { transition: opacity .15s; }
  #chartWrap { position: relative; }
  #chartWrap svg { display: block; width: 100%; }
  #chartWrap g.band:hover { filter: brightness(1.1); }   /* 悬停整组提亮 */
  #tip { position: absolute; pointer-events: none; background: #fff; border: 1px solid #e5e7eb;
         border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,.08); padding: 8px 10px;
         font-size: 12px; display: none; min-width: 130px; z-index: 5; }
  .tip-h { color: #6b7280; margin-bottom: 4px; }
  .tip-row { display: flex; align-items: center; gap: 6px; margin-top: 3px; }
  .tip-key { width: 10px; height: 3px; border-radius: 2px; flex: none; }
  .tip-name { color: var(--ink-2); }
  .tip-val { font-weight: 600; color: var(--ink); margin-left: auto; padding-left: 12px; }
  details.tbl { margin-top: 10px; font-size: 13px; }
  details.tbl summary { color: #6b7280; cursor: pointer; font-size: 12px; }
  details.tbl table td, details.tbl table th { font-variant-numeric: tabular-nums; }
  .empty-chart { color: #9ca3af; text-align: center; padding: 34px 0; font-size: 13px; }
  #prevBox { min-height: 60px; text-align: center; }
  #prevBox img { display: block; margin: 0 auto; max-width: min(100%, 640px); height: auto;
                 border-radius: 8px; border: 1px solid #e5e7eb; cursor: zoom-in; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div>
      <h1>ESP 坐姿检测控制台</h1>
      <div class="sub" id="sub">连接中…</div>
    </div>
    <span class="badge" id="fwbadge"></span>
  </header>

  <div class="ota-banner" id="otaBanner"></div>

  <div class="card">
    <h2>设备
      <span>
        <button class="mini primary" onclick="cmd('ota')">升级全部</button>
        <button class="mini" id="alertDefBtn" onclick="toggleAlertDef()">提醒默认: ?</button>
        <button class="mini" id="watchBtn" onclick="toggleWatch()">自动检测: ?</button>
      </span>
    </h2>
    <table>
      <thead><tr><th>#</th><th>设备 ID</th><th>IP</th><th>固件</th><th>检测时段</th><th>时间同步</th><th>提醒</th><th>前倾阈值</th><th>状态</th><th style="text-align:right">操作</th></tr></thead>
      <tbody id="devRows"></tbody>
    </table>
    <div class="empty" id="devEmpty" style="display:none">暂无设备应答（等待 2s 发现周期…）</div>
    <div class="sched-hint">提醒列下拉选择该设备的不良语音模式（连续播/只播一次/关闭——关闭仅停语音播报，检测与上报照常；写入 ESP NVS 断电保持，切换后以 ESP 确认回执为准）；"提醒默认"按钮三档循环统一下发全部设备。前倾阈值列显示 眼/耳 当前判断阈值（修改见下方"前倾比阈值"卡）。旧固件两列显示 —，OTA 升级后可用。</div>
  </div>

  <div class="card">
    <h2>坐姿统计</h2>
    <div class="filter-row">
      <select id="devSel" onchange="onDev()"></select>
      <span class="seg" id="rangeSeg">
        <button data-r="day" class="on">今日</button><button data-r="week">本周</button><button data-r="month">本月</button><button data-r="year">今年</button><button data-r="all">全部</button>
      </span>
    </div>
    <div id="statsBody">
      <div class="tiles" id="tiles"></div>
      <div class="legend">
        <span><i class="sw" style="background:var(--series-1)"></i>检测帧数</span>
        <span><i class="sw" style="background:var(--series-2)"></i>不良帧数</span>
        <span><i class="sw-line" style="border-top-color:var(--series-3)"></i>不良率</span>
      </div>
      <div id="chartWrap">
        <div id="chartBox"></div>
        <div id="tip"></div>
      </div>
      <details class="tbl"><summary>数据表</summary><table id="statsTable"></table></details>
    </div>
  </div>

  <div class="card">
    <h2>检测时段</h2>
    <div class="sched-now" id="schedNow">—</div>
    <div class="filter-row" style="margin-bottom:10px">
      <select id="schedDev"></select>
    </div>
    <div class="row">
      <input type="text" id="slotsInput" placeholder="如 09:00-11:30,14:00-18:00（跨午夜写作 22:00-06:30）">
      <button class="primary" onclick="setSlots()">设置</button>
      <button onclick="clearSlots()">清空</button>
    </div>
    <div class="sched-hint">选"全部设备"= 统一所有设备（各台专属设置会被清除）；选单台 = 只改它，重启接收器后仍会自动恢复</div>
  </div>

  <div class="card">
    <h2>检测阈值（前倾比 + 歪头角度）</h2>
    <div class="sched-now" id="ratioNow">—</div>
    <div class="filter-row" style="margin-bottom:10px">
      <select id="ratioDev" onchange="renderRatio(cur)"></select>
    </div>
    <div class="filter-row">
      <span class="lbl">前倾比</span>
      <span class="lbl">眼</span>
      <input type="number" id="eyeThrInput" min="0.2" max="10" step="0.05">
      <span class="lbl">耳</span>
      <input type="number" id="earThrInput" min="0.2" max="10" step="0.05">
    </div>
    <div class="filter-row">
      <span class="lbl">歪头角</span>
      <span class="lbl">眼</span>
      <input type="number" id="eyeTiltInput" min="10" max="80" step="1">
      <span class="lbl">耳</span>
      <input type="number" id="earTiltInput" min="10" max="80" step="1">
      <button class="mini primary" onclick="applyRatio()">下发设置</button>
      <button class="mini" onclick="fillRatioDefault()">填入默认</button>
    </div>
    <div class="sched-hint">前倾比：眼/耳-肩垂直距离 ÷ 双眼/耳距 &lt; 阈值 → 前倾不良（0.2~10，默认眼 1.2 / 耳 0.7，越小越宽松）；
      歪头角：头-肩相对倾斜角 &gt; 阈值 → 歪头不良（10~80 度，默认眼 20 / 耳 15，越小越严格）。写入 ESP NVS 断电保持。
      选"全部设备"= 统一所有设备（各台专属被清除）；选单台 = 只改它，重启接收器后仍会自动恢复。旧固件显示 —</div>
  </div>

  <div class="card">
    <h2>图像保存
      <span>
        <button class="mini" id="imgSaveBtn" onclick="toggleImgSave()">保存图片: ?</button>
        <button class="mini" id="imgKpBtn" onclick="toggleImgKp()">关键点叠加: ?</button>
        <button class="mini" id="imgPeopleBtn" onclick="toggleImgPeople()">仅有人帧: ?</button>
      </span>
    </h2>
    <div class="filter-row" style="margin-bottom:10px">
      <select id="imgDev" onchange="renderImg(cur)"></select>
    </div>
    <div class="filter-row">
      <span class="lbl">每</span>
      <input type="number" id="imgSaveN" min="0" max="600">
      <span class="lbl">帧保存 1 帧</span>
      <button class="mini primary" onclick="applyImgSaveN()">应用</button>
      <button class="mini" onclick="resetImgDev()">恢复默认</button>
    </div>
    <div class="sched-now" id="imgNow" style="margin:10px 0 0">—</div>
    <div class="sched-hint">选"全部设备"=改默认（该字段各台专属被统一清除）；选单台=只改它（恢复默认=清除专属跟随默认）。
      0=不保存图片；1=每帧都存；"保存图片"开关=0 与 10 快捷切换；叠加含关键点连线/判断文字/时间戳；持久保存（recv_settings.json）</div>
  </div>

  <div class="card">
    <h2>图像预览（最新保存帧）
      <span>
        <select id="prevDev" onchange="onPrevDev()"></select>
        <button class="mini" id="prevPlayBtn" onclick="togglePreview()">暂停刷新</button>
      </span>
    </h2>
    <div id="prevBox"><div class="empty-chart">选择设备后显示该设备最新保存的图片…</div></div>
    <div class="sched-now" id="prevMeta" style="margin:8px 0 0">—</div>
    <div class="sched-hint">显示该设备最近一次保存的 PNG（已含关键点/判断文字/时间戳叠加），默认每 5s 自动刷新。
      看不到图时确认"图像保存"设置：设为 0 不保存、开了"仅有人帧"且当前无人也不存（此时无新图可预览）。</div>
  </div>

  <div class="card">
    <h2>运行日志
      <span>
        <select id="logDev" onchange="renderLog(cur)"></select>
        <button class="mini" onclick="toggleLog(this)">暂停滚动</button>
      </span>
    </h2>
    <pre id="log"></pre>
  </div>
</div>
<script>
/* ---------- 全局状态轮询（设备表/调度/OTA/日志） ---------- */
let cur = { devices: [], watch: true };
let stick = true;  // 日志面板是否吸底滚动（用户上翻查看历史时暂停）

async function poll() {
  try {
    const s = await (await fetch('/api/state')).json();
    cur = s;
    render(s);
  } catch (e) {
    document.getElementById('sub').textContent = '连接失败（接收器未运行？）';
  }
}

async function cmd(c) {
  try {
    await fetch('/api/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: c })
    });
  } catch (e) {}
  setTimeout(poll, 400);  // 稍等命令输出进日志后再刷一次
}

function esc(t) {
  return String(t == null ? '' : t).replace(/[&<>"]/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

/* 设备下拉填充（schedDev/logDev 共用）：选项变化时才重建，保留当前选择 */
function fillDevs(sel) {
  const want = ['all'].concat((cur.devices || []).map(d => d.id));
  if (sel.options.length !== want.length || [...sel.options].some((o, i) => o.value !== want[i])) {
    const old = sel.value;
    sel.textContent = '';
    for (const d of want) {
      const o = document.createElement('option');
      o.value = d;
      o.textContent = d === 'all' ? '全部设备' : d;  // textContent：ID 视为不可信数据
      sel.appendChild(o);
    }
    if (want.includes(old)) sel.value = old;
  }
}

/* ---------- 图像预览（/api/frame：该设备最新保存帧，已含关键点叠加） ---------- */
let prevTimer = null, prevOn = true, prevUrl = null, prevInited = false;

function onPrevDev() { loadPreview(); restartPreview(); }
function togglePreview() {
  prevOn = !prevOn;
  document.getElementById('prevPlayBtn').textContent = prevOn ? '暂停刷新' : '继续刷新';
  if (prevOn) restartPreview(); else stopPreview();
}
function stopPreview() { if (prevTimer) { clearInterval(prevTimer); prevTimer = null; } }
function restartPreview() { stopPreview(); if (prevOn) prevTimer = setInterval(loadPreview, 5000); }

function nameToTime(n) {  // image_20260928_143000_123.png → 2026-09-28 14:30:00
  const m = /image_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/.exec(n || '');
  return m ? `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}:${m[6]}` : '';
}

async function loadPreview() {
  const box = document.getElementById('prevBox'), meta = document.getElementById('prevMeta');
  const dev = document.getElementById('prevDev').value;
  if (!dev) return;
  if (dev === 'all') {  // 图片按设备分目录保存，"全部设备"下只能提示选具体设备
    box.innerHTML = '<div class="empty-chart">请选择具体设备（图片按设备分目录保存）</div>';
    meta.textContent = '—';
    return;
  }
  try {
    const r = await fetch('/api/frame?device=' + encodeURIComponent(dev) + '&t=' + Date.now());
    if (!r.ok) {
      box.innerHTML = '<div class="empty-chart">该设备暂无保存的图片<br>' +
        '（"图像保存"设为 0、或开启"仅有人帧"而当前无人、或尚未到抽稀帧）</div>';
      meta.textContent = '无图片（HTTP ' + r.status + '）';
      return;
    }
    const name = r.headers.get('X-Image-Name') || '';
    const blob = await r.blob();
    if (prevUrl) URL.revokeObjectURL(prevUrl);  // 释放上一张，避免内存堆积
    prevUrl = URL.createObjectURL(blob);
    box.innerHTML = '<img alt="最新保存帧" title="点击查看原图" onclick="if(prevUrl) window.open(prevUrl)">';
    box.firstChild.src = prevUrl;
    meta.textContent = [name, nameToTime(name), (blob.size / 1024).toFixed(0) + ' KB'].filter(Boolean).join(' · ');
  } catch (e) {
    meta.textContent = '加载失败: ' + e;
  }
}

let lastDevRows = '';  // 上次设备表 HTML（无变化不重绘，见 render）

function render(s) {
  const online = (s.devices || []).filter(d => d.online);
  const act = online.filter(d => d.sched_active);
  document.getElementById('sub').textContent = online.length
    ? `${online.length} 台设备在线 · ${act.length} 台检测中` : '暂无设备在线（等待 2s 发现周期…）';
  document.getElementById('fwbadge').textContent = '目标固件 ' + (s.fw_version || '?');
  document.getElementById('watchBtn').textContent = '自动检测: ' + (s.watch ? '开' : '关');
  document.getElementById('alertDefBtn').textContent = '提醒默认: ' + ({1: '连续播', 0: '只播一次', 2: '关闭'}[s.alert_default] || '?');
  const banner = document.getElementById('otaBanner');
  if (s.ota_current) {
    banner.style.display = 'block';
    let txt = 'OTA 进行中：正在升级 ' + s.ota_current;
    if (s.ota_phase === 'download' && s.ota_progress)
      txt += ` — 下载 ${s.ota_progress.pct}%（${Math.round(s.ota_progress.cur / 1024)}/${Math.round(s.ota_progress.tot / 1024)} KB）`;
    else if (s.ota_phase === 'verify') txt += ' — 下载完成，等待重启确认';
    if (s.ota_queue) txt += `（后面还有 ${s.ota_queue} 台）`;
    banner.textContent = txt;
  } else {
    banner.style.display = 'none';
  }
  const tb = document.getElementById('devRows');
  const rowsHtml = (s.devices || []).map((d, i) => {
    let st;
    if (d.id === s.ota_current) {  // 正在升级的设备：进度条 / 阶段提示
      if (s.ota_phase === 'download' && s.ota_progress) {
        st = `<span style="display:inline-flex;align-items:center;gap:6px">` +
             `<span class="prog"><span class="prog-fill" style="width:${s.ota_progress.pct}%"></span></span>` +
             `<span class="dim">${s.ota_progress.pct}%</span></span>`;
      } else if (s.ota_phase === 'verify') st = '<span class="chip upg">重启验证…</span>';
      else st = '<span class="chip upg">已触发…</span>';
    } else if (!d.online) st = '<span class="chip off">离线</span>';
    else if (d.up_to_date) st = '<span class="chip ok">最新</span>';
    else st = '<span class="chip up">待升级</span>';
    if (d.fails >= 3) st += ' <span title="连续 3 次未成功，自动升级已暂停">⚠</span>';
    // 检测时段/时间同步：SCHED_STATE 上报（在线但 90s 未上报 → 灰色"—"）
    const sched = d.sched_desc == null ? '<span class="dim">—</span>' : esc(d.sched_desc);
    let sync;
    if (d.sched_synced == null) sync = '<span class="dim">—</span>';
    else if (d.sched_synced) sync = '<span class="chip ok">已同步</span>';
    else sync = '<span class="chip up">未同步</span>';
    // 不良提醒模式下拉（ACK 上报当前值；选择后等 ESP 回 ALERT_STATE 确认才跳新值；旧固件未上报 → —）
    const alert = d.alert == null ? '<span class="dim">—</span>'
      : `<select class="cell-sel" onchange="setAlert('${esc(d.id)}', this.value)">` +
        `<option value="repeat"${d.alert === 1 ? ' selected' : ''}>连续播</option>` +
        `<option value="once"${d.alert === 0 ? ' selected' : ''}>只播一次</option>` +
        `<option value="off"${d.alert === 2 ? ' selected' : ''}>关闭</option></select>`;
    // 前倾比阈值 眼/耳（ACK 上报当前值，只读展示，修改在"前倾比阈值"卡；旧固件未上报 → —）
    const ratio = (d.eye_thr == null || d.ear_thr == null) ? '<span class="dim">—</span>'
      : esc(d.eye_thr.toFixed(2)) + ' / ' + esc(d.ear_thr.toFixed(2));
    const act2 = (d.online && !d.up_to_date)
      ? `<button class="mini primary" onclick="cmd('ota ${esc(d.id)}')">升级</button>` : '';
    return `<tr><td>${i + 1}</td><td>${esc(d.id)}</td><td>${esc(d.ip || '—')}</td>` +
           `<td>${esc(d.ver || '?')}</td><td>${sched}</td><td>${sync}</td><td>${alert}</td><td>${ratio}</td><td>${st}</td><td style="text-align:right">${act2}</td></tr>`;
  }).join('');
  // 内容无变化不重绘：避免 2s 轮询把用户正在展开的下拉/悬停状态打断
  if (rowsHtml !== lastDevRows) { tb.innerHTML = rowsHtml; lastDevRows = rowsHtml; }
  document.getElementById('devEmpty').style.display = (s.devices || []).length ? 'none' : 'block';

  // 时段卡：各设备当前调度状态（SCHED_STATE，2s 查询周期）
  const sm = s.sched_map || {};
  const ks = Object.keys(sm);
  document.getElementById('schedNow').innerHTML = ks.length
    ? ks.map(k => `<div><b>${esc(k)}</b> ${esc(sm[k])}</div>`).join('') : '—';

  fillDevs(document.getElementById('schedDev'));
  fillDevs(document.getElementById('logDev'));
  const psel = document.getElementById('prevDev');
  fillDevs(psel);
  if (!prevInited && (s.devices || []).length) {  // 首次自动选第一台：停在"全部设备"看不到图
    psel.value = s.devices[0].id;
    prevInited = true;
  }
  if (!prevTimer && prevOn) { loadPreview(); restartPreview(); }  // 首次出图 + 启动 5s 自动刷新
  renderRatio(s);
  renderImg(s);
  renderLog(s);
}

function renderLog(s) {  // 日志面板：按设备过滤（行内含 "[devid]" 或 " devid " 即匹配）
  const lg = document.getElementById('log');
  const atBottom = lg.scrollTop + lg.clientHeight >= lg.scrollHeight - 30;
  let lines = s.log || [];
  const f = document.getElementById('logDev').value;
  if (f && f !== 'all') lines = lines.filter(l => l.includes('[' + f + ']') || l.includes(' ' + f + ' '));
  lg.textContent = lines.join('\\n');
  if (stick && atBottom) lg.scrollTop = lg.scrollHeight;
}

function schedTarget() {  // 'all' → ''（广播）；单台 → '@ID '（单播）
  const d = document.getElementById('schedDev').value;
  return d === 'all' ? '' : '@' + d + ' ';
}
function setSlots() {
  const v = document.getElementById('slotsInput').value.trim();
  if (v) cmd('s ' + schedTarget() + v);
}
function clearSlots() { cmd('s ' + schedTarget() + 'clear'); }

/* ---------- 检测阈值（前倾比 + 歪头角：全部设备默认 + 按设备覆盖，等价 ratio 命令） ---------- */
function ratioTarget() {  // 'all' → ''（广播全部设备）；单台 → '@ID '（单播）
  const d = document.getElementById('ratioDev').value;
  return d && d !== 'all' ? '@' + d + ' ' : '';
}
function applyRatio() {  // 范围本地预校验（与 ESP 一致），不合法不下发
  const e = parseFloat(document.getElementById('eyeThrInput').value);
  const r = parseFloat(document.getElementById('earThrInput').value);
  const t1 = parseFloat(document.getElementById('eyeTiltInput').value);
  const t2 = parseFloat(document.getElementById('earTiltInput').value);
  if (!(e >= 0.2 && e <= 10) || !(r >= 0.2 && r <= 10)) {
    alert('前倾比阈值须在 0.2~10 之间（默认 1.5）'); return;
  }
  if (!(t1 >= 10 && t1 <= 80) || !(t2 >= 10 && t2 <= 80)) {
    alert('歪头倾角阈值须在 10~80 度之间（默认 眼 20 / 耳 15）'); return;
  }
  cmd('ratio ' + ratioTarget() + e + ' ' + r + ' ' + t1 + ' ' + t2);
}
function fillRatioDefault() {  // 把默认值填入输入框（不直接下发，确认后点"下发设置"）
  document.getElementById('eyeThrInput').value = 1.5;
  document.getElementById('earThrInput').value = 1.5;
  document.getElementById('eyeTiltInput').value = 35;
  document.getElementById('earTiltInput').value = 35;
}
function renderRatio(s) {  // 当前生效值 + 输入框预填所选目标的当前阈值（正在输入时不打扰）
  fillDevs(document.getElementById('ratioDev'));
  const t = document.getElementById('ratioDev').value;
  const dflt = s.ratio_default || [1.2, 0.7, 20, 15];
  const dev = (s.devices || []).find(x => x.id === t);
  const has = dev && dev.eye_thr != null && dev.ear_thr != null;
  const ve = has ? dev.eye_thr : +dflt[0];
  const vr = has ? dev.ear_thr : +dflt[1];
  const vt1 = dev && dev.eye_tilt != null ? dev.eye_tilt : +dflt[2];
  const vt2 = dev && dev.ear_tilt != null ? dev.ear_tilt : +dflt[3];
  document.getElementById('ratioNow').innerHTML = has
    ? `当前: ${esc(t)} 眼&lt;${ve.toFixed(2)} 耳&lt;${vr.toFixed(2)} 倾角${vt1.toFixed(0)}°/${vt2.toFixed(0)}°`
    : `当前: 默认 眼&lt;${ve.toFixed(2)} 耳&lt;${vr.toFixed(2)} 倾角${vt1.toFixed(0)}°/${vt2.toFixed(0)}°` +
      (dev ? ' <span class="dim">（旧固件未上报）</span>' : '');
  const f = (id, v) => { const el = document.getElementById(id); if (document.activeElement !== el) el.value = v; };
  f('eyeThrInput', ve); f('earThrInput', vr); f('eyeTiltInput', vt1); f('earTiltInput', vt2);
}

/* ---------- 图像保存（抽稀/叠加：全部设备默认 + 按设备覆盖，等价 img 命令） ---------- */
function imgTarget() {  // 'all' → ''（全部设备）；单台 → '@ID '
  const d = document.getElementById('imgDev').value;
  return d && d !== 'all' ? '@' + d + ' ' : '';
}
function imgSelCfg(s) {  // 下拉当前选中目标的生效配置（单台回退默认）
  const t = document.getElementById('imgDev').value;
  return (t && t !== 'all' ? (s.img?.devices || {})[t] : null) || s.img?.default || {};
}
function toggleImgKp() { cmd('img kp ' + imgTarget() + (imgSelCfg(cur).draw_kp ? 'off' : 'on')); }
function toggleImgPeople() {  // 仅保存有人帧：最近结果连续 50 帧无效（没人/太远/不可信）即停存
  cmd('img people ' + imgTarget() + (imgSelCfg(cur).people ? 'off' : 'on'));
}
function toggleImgSave() {  // 保存快捷开关：当前不保存 → 恢复为 10；保存中 → 关闭（0）
  cmd('img save ' + imgTarget() + (imgSelCfg(cur).save_n > 0 ? '0' : '10'));
}
function applyImgSaveN() {
  const v = parseInt(document.getElementById('imgSaveN').value, 10);
  if (v >= 0) cmd('img save ' + imgTarget() + v);
}
function resetImgDev() {  // 仅对单台有意义：清除其专属，恢复跟随默认
  const d = document.getElementById('imgDev').value;
  if (d && d !== 'all') cmd('img reset @' + d);
}
function imgSaveDesc(n) { return n > 0 ? `每 ${n} 帧存 1 帧` : '不保存'; }
function renderImg(s) {
  fillDevs(document.getElementById('imgDev'));
  const cfg = imgSelCfg(s);
  document.getElementById('imgKpBtn').textContent = '关键点叠加: ' + (cfg.draw_kp ? '开' : '关');
  document.getElementById('imgSaveBtn').textContent = '保存图片: ' + ((cfg.save_n ?? 10) > 0 ? '开' : '关');
  document.getElementById('imgPeopleBtn').textContent = '仅有人帧: ' + (cfg.people ? '开' : '关');
  const inp = document.getElementById('imgSaveN');
  if (document.activeElement !== inp) inp.value = cfg.save_n ?? 10;  // 正在输入时不打扰
  const cust = s.img_custom || [];
  document.getElementById('imgNow').innerHTML =
    Object.entries(s.img?.devices || {}).map(([k, v]) =>
      `<div><b>${esc(k)}</b>${cust.includes(k) ? '（专属）' : ''}: 叠加${v.draw_kp ? '开' : '关'} · ${imgSaveDesc(v.save_n)} · 仅有人${v.people ? '开' : '关'}</div>`
    ).join('') || '—';
}
function toggleWatch() { cmd(cur.watch ? 'ota off' : 'ota on'); }
function toggleAlertDef() {  // 全部设备统一下发（清掉各台专属）：三档循环 连续播→只播一次→关闭→连续播
  const next = {1: 'once', 0: 'off', 2: 'repeat'}[cur.alert_default ?? 1] || 'once';
  cmd('alert ' + next);
}
function setAlert(id, v) { cmd('alert @' + id + ' ' + v); }  // 设备表"提醒"下拉（等价 alert @ID repeat|once）
function toggleLog(btn) {
  stick = !stick;
  btn.textContent = stick ? '暂停滚动' : '恢复滚动';
  if (stick) { const lg = document.getElementById('log'); lg.scrollTop = lg.scrollHeight; }
}

/* ---------- 坐姿统计（/api/stats：设备 × 时间范围） ---------- */
let curRange = 'day', curDev = 'all';

function onDev() {
  curDev = document.getElementById('devSel').value;
  loadStats();
}
document.getElementById('rangeSeg').addEventListener('click', e => {
  const b = e.target.closest('button[data-r]');
  if (!b) return;
  curRange = b.dataset.r;
  document.querySelectorAll('#rangeSeg button').forEach(x => x.classList.toggle('on', x === b));
  loadStats();
});

async function loadStats() {
  const body = document.getElementById('statsBody');
  body.style.opacity = .55;  // 重取期间保留上一帧渲染，只降透明度（不闪骨架屏）
  try {
    const s = await (await fetch(
      `/api/stats?range=${curRange}&device=${encodeURIComponent(curDev)}`)).json();
    renderStats(s);
  } catch (e) {}
  body.style.opacity = 1;
}

function tile(label, value) {
  return `<div class="tile"><div class="tlabel">${label}</div><div class="tval">${value}</div></div>`;
}

function renderStats(s) {
  // 设备下拉（保留当前选择；选项变化时才重建）
  const sel = document.getElementById('devSel');
  const opts = ['all'].concat(s.devices || []);
  if (sel.options.length !== opts.length || [...sel.options].some((o, i) => o.value !== opts[i])) {
    sel.textContent = '';
    for (const d of opts) {
      const o = document.createElement('option');
      o.value = d;
      o.textContent = d === 'all' ? '全部设备' : d;  // textContent：ID 视为不可信数据
      sel.appendChild(o);
    }
    sel.value = opts.includes(curDev) ? curDev : 'all';
    curDev = sel.value;
  }

  const sm = s.summary || {};
  document.getElementById('tiles').innerHTML =
    tile('检测帧数', (sm.total ?? 0).toLocaleString()) +
    tile('有效判断帧', (sm.valid ?? 0).toLocaleString()) +
    tile('不良帧数', (sm.bad ?? 0).toLocaleString()) +
    tile('不良率', sm.rate == null ? '—' : sm.rate + '%') +
    tile('最长不良持续', (sm.longest_min ?? 0) + ' 分钟');

  drawChart(s.buckets || []);
  drawTable(s.buckets || []);
}

/* ---- 分组柱形图（SVG 手绘；规格：柱厚≤24px、顶部 4px 圆角、2px 表面间隙、
        发丝实线网格、文本用墨色 token 不用系列色、悬停整组提亮 + 提示框） ---- */
function niceMax(v) {  // 取 1/2/5×10^k 的整刻度上限
  if (v <= 5) return 5;
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  for (const m of [1, 2, 5, 10]) if (m * p >= v) return m * p;
  return 10 * p;
}

function barPath(x, y, w, h, r) {  // 顶部圆角、底部平方（贴基线）
  r = Math.min(r, w / 2, h);
  return `M${x},${y + h} L${x},${y + r} Q${x},${y} ${x + r},${y}` +
         ` L${x + w - r},${y} Q${x + w},${y} ${x + w},${y + r} L${x + w},${y + h} Z`;
}

function drawChart(buckets) {
  const box = document.getElementById('chartBox');
  if (!buckets.length || !buckets.some(b => b.total)) {
    box.innerHTML = '<div class="empty-chart">所选范围内暂无数据</div>';
    return;
  }
  const W = box.clientWidth || 800, H = 240, m = { l: 44, r: 34, t: 8, b: 26 };
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  const ymax = niceMax(Math.max(...buckets.map(b => b.total)));
  const n = buckets.length;
  const band = pw / n;
  const barW = Math.min(24, Math.max(6, (band - 8) / 2));  // 组内两柱 + 2px 间隙 + 余量换气
  const y = v => m.t + ph - v / ymax * ph;
  // 不良率曲线：固定 0-100% 满量程（顶=100%），不随计数轴缩放——位置语义稳定，右侧独立小刻度
  const yr = p => m.t + ph - Math.min(p, 100) / 100 * ph;
  const hasRate = buckets.some(b => b.rate != null);

  let g = '';
  const ticks = 4;
  for (let i = 0; i <= ticks; i++) {  // 网格 + y 刻度（干净整数，千分位）
    const v = ymax / ticks * i, yy = y(v);
    g += `<line x1="${m.l}" y1="${yy}" x2="${W - m.r}" y2="${yy}" stroke="${i ? 'var(--grid)' : 'var(--axis)'}" stroke-width="1"/>`;
    g += `<text x="${m.l - 8}" y="${yy + 4}" text-anchor="end" font-size="11" fill="var(--muted)">${Math.round(v).toLocaleString()}</text>`;
  }
  if (hasRate) {  // 右侧率刻度（0/50/100%，浅色短刻度线+小字，与主刻度视觉区分）
    for (const p of [0, 50, 100]) {
      const yy = yr(p);
      g += `<line x1="${W - m.r + 3}" y1="${yy}" x2="${W - m.r + 7}" y2="${yy}" stroke="var(--axis)" stroke-width="1"/>`;
      g += `<text x="${W - m.r + 9}" y="${yy + 3.5}" font-size="10" fill="var(--muted)">${p}%</text>`;
    }
  }
  const skip = Math.ceil(n / 12);  // x 标签抽稀，避免挤压
  buckets.forEach((b, i) => {
    const cx = m.l + band * i + band / 2;
    const x1 = cx - barW - 1, x2 = cx + 1;
    const h1 = (b.total / ymax) * ph, h2 = (b.bad / ymax) * ph;
    g += `<g class="band" data-i="${i}">` +
      `<path d="${barPath(x1, y(b.total), barW, Math.max(h1, 0.5), 4)}" fill="var(--series-1)"/>` +
      `<path d="${barPath(x2, y(b.bad), barW, Math.max(h2, 0.5), 4)}" fill="var(--series-2)"/>` +
      `<rect x="${m.l + band * i}" y="${m.t}" width="${band}" height="${ph}" fill="transparent"/>`;  // 悬停命中区=整band
    if (i % skip === 0) {
      let lb = esc(b.label);
      if (n > 8) lb = lb.replace(/^0/, '');  // 挤时去掉前导 0
      g += `<text x="${cx}" y="${H - 8}" text-anchor="middle" font-size="11" fill="var(--muted)">${lb}</text>`;
    }
    g += '</g>';
  });
  if (hasRate) {  // 不良率折线：2px 线 + 小圆点（1px 白描边=表面环）；null 桶（无有效帧）断开分段
    let d = '', pen = false;
    buckets.forEach((b, i) => {
      if (b.rate == null) { pen = false; return; }
      const cx = m.l + band * i + band / 2;
      d += (pen ? ' L' : ' M') + cx.toFixed(1) + ',' + yr(b.rate).toFixed(1);
      pen = true;
    });
    g += `<path d="${d}" fill="none" stroke="var(--series-3)" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>`;
    buckets.forEach((b, i) => {
      if (b.rate == null) return;
      const cx = m.l + band * i + band / 2;
      g += `<circle cx="${cx.toFixed(1)}" cy="${yr(b.rate).toFixed(1)}" r="2.5" fill="var(--series-3)" stroke="#fff" stroke-width="1"/>`;
    });
  }
  box.innerHTML = `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="按时间分组的检测与不良帧数柱形图与不良率曲线">${g}</svg>`;

  // 悬停提示：值为主（粗体）、系列名次之；系列名/桶名经 textContent 写入（不可信数据）
  const tip = document.getElementById('tip');
  box.querySelector('svg').addEventListener('pointermove', e => {
    const bandEl = e.target.closest('g.band');
    if (!bandEl) { tip.style.display = 'none'; return; }
    const b = buckets[+bandEl.dataset.i];
    tip.textContent = '';
    const h = document.createElement('div');
    h.className = 'tip-h';
    h.textContent = b.label;
    tip.appendChild(h);
    for (const [name, val, color, isRate] of [['检测帧数', b.total, 'var(--series-1)'],
                                              ['不良帧数', b.bad, 'var(--series-2)'],
                                              ['不良率', b.rate, 'var(--series-3)', true]]) {
      const r = document.createElement('div');
      r.className = 'tip-row';
      const k = document.createElement('i');
      k.className = 'tip-key'; k.style.background = color;
      if (isRate) k.style.cssText = 'height:2px;width:12px;border-radius:1px';  // 线系列用线形标记
      const nm = document.createElement('span');
      nm.className = 'tip-name'; nm.textContent = name;
      const v = document.createElement('span');
      v.className = 'tip-val'; v.textContent = val == null ? '—' : (isRate ? val + '%' : val.toLocaleString());
      r.append(k, nm, v);
      tip.appendChild(r);
    }
    tip.style.display = 'block';
    const wr = document.getElementById('chartWrap').getBoundingClientRect();
    let tx = e.clientX - wr.left + 14;
    if (tx + 150 > wr.width) tx = e.clientX - wr.left - 160;  // 靠右时翻到左侧
    tip.style.left = tx + 'px';
    tip.style.top = Math.max(0, e.clientY - wr.top - 20) + 'px';
  });
  box.querySelector('svg').addEventListener('pointerleave', () => { tip.style.display = 'none'; });
}

function drawTable(buckets) {  // 图表的表格孪生（免悬停也能读全部数值）
  const t = document.getElementById('statsTable');
  if (!buckets.length) { t.textContent = ''; return; }
  let rows = '<thead><tr><th>时间</th><th>检测帧数</th><th>不良帧数</th><th>不良率</th></tr></thead><tbody>';
  for (const b of buckets) {
    const rate = b.rate == null ? '—' : b.rate + '%';  // valid 分母（与摘要/曲线同口径）
    rows += `<tr><td>${esc(b.label)}</td><td>${b.total.toLocaleString()}</td><td>${b.bad.toLocaleString()}</td><td>${rate}</td></tr>`;
  }
  t.innerHTML = rows + '</tbody>';
}

setInterval(poll, 2000);
poll();
loadStats();
setInterval(loadStats, 60000);  // 统计低频刷新（SQL 全窗扫描，不值得 2s 一次）
</script>
</body>
</html>
"""


# ---------- 图像预览：最新保存帧（received_images/<设备ID>/image_<时间戳>.png）----------

_IMG_ROOT = "received_images"  # 与接收器同 cwd：图片落在启动目录下
_DEV_DIR_RE = re.compile(r"[A-Za-z0-9_.-]{1,64}")  # 设备目录名白名单（device 来自 URL，防目录穿越）


def _latest_image(devid=""):
    """返回最新一张保存帧的路径（文件名 image_YYYYmmdd_HHMMSS_mmm.png，字典序即时间序）。
    devid 为空 = 所有设备目录里最新的；无图/目录不存在 → None"""
    if devid:
        if not _DEV_DIR_RE.fullmatch(devid):
            return None
        dirs = [os.path.join(_IMG_ROOT, devid)]
    else:
        try:
            dirs = [e.path for e in os.scandir(_IMG_ROOT) if e.is_dir()]
        except OSError:
            return None
    best_name, best_path = "", None
    for d in dirs:
        try:
            with os.scandir(d) as it:
                for e in it:
                    if e.name.endswith(".png") and e.name > best_name:
                        best_name, best_path = e.name, e.path
        except OSError:
            continue
    return best_path


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"  # keep-alive（Content-Length 恒有，安全）

    def log_message(self, fmt, *args):  # 静默逐请求访问日志
        pass

    def _send(self, code, body, ctype):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, _PAGE, "text/html; charset=utf-8")
        elif self.path == "/api/state":
            try:
                snap = _state_provider() if _state_provider else {}
            except Exception as e:
                self._send(500, json.dumps({"error": str(e)}), "application/json; charset=utf-8")
                return
            snap["log"] = list(_log)
            self._send(200, json.dumps(snap, ensure_ascii=False), "application/json; charset=utf-8")
        elif self.path.startswith("/api/stats"):
            q = parse_qs(urlparse(self.path).query)
            rng = (q.get("range") or ["day"])[0]
            dev = (q.get("device") or ["all"])[0]
            if rng not in ("day", "week", "month", "year", "all"):
                rng = "day"
            try:
                snap = _stats_provider(rng, dev) if _stats_provider else {}
            except Exception as e:
                snap = {"error": str(e), "buckets": [], "devices": [], "summary": {}}
            self._send(200, json.dumps(snap, ensure_ascii=False), "application/json; charset=utf-8")
        elif self.path.startswith("/api/frame"):
            # 最新保存帧（已含关键点叠加与时间戳）：前端 fetch 成 blob 显示，避免浏览器缓存旧图
            q = parse_qs(urlparse(self.path).query)
            path = _latest_image((q.get("device") or [""])[0])
            if not path:
                self._send(404, "尚无保存的图片", "text/plain; charset=utf-8")
                return
            try:
                with open(path, "rb") as f:
                    data = f.read()
            except OSError as e:
                self._send(500, f"读取失败: {e}", "text/plain; charset=utf-8")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store, must-revalidate")  # 每次都要最新帧
            self.send_header("X-Image-Name", os.path.basename(path))        # 前端据此显示抓拍时间
            self.end_headers()
            self.wfile.write(data)
        else:
            self._send(404, "not found", "text/plain; charset=utf-8")

    def do_POST(self):
        if self.path != "/api/cmd":
            self._send(404, "not found", "text/plain; charset=utf-8")
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            cmd = json.loads(self.rfile.read(n).decode("utf-8")).get("cmd", "")
        except Exception:
            self._send(400, '{"error": "bad request"}', "application/json; charset=utf-8")
            return
        if not cmd or len(cmd) > 200:
            self._send(400, '{"error": "bad cmd"}', "application/json; charset=utf-8")
            return
        print(f"[web] {cmd}")
        if _cmd_dispatcher:
            try:
                _cmd_dispatcher(cmd)
            except Exception as e:
                print(f"[web] 命令执行异常: {e}")
        self._send(200, '{"ok": true}', "application/json; charset=utf-8")


class _QuietWebServer(ThreadingHTTPServer):
    """浏览器刷新/关闭时在途的 /api/state 轮询连接被掐断属正常——
    静默为一行提示，不打整段 traceback（与接收器 OTA 服务的 _QuietHTTPServer 同一套路）"""

    def handle_error(self, request, client_address):
        print(f"[web] HTTP 连接中断（{client_address[0]}，页面刷新/关闭时正常）")


def start(state_provider, cmd_dispatcher, port=WEB_PORT):
    """启动 Web 控制台（在接收器 __main__ 里调用一次）：
    state_provider: () → 状态快照 dict；cmd_dispatcher: (str) → 处理一条命令（与键盘输入同路径）；
    统计源用 set_stats_provider() 注入"""
    global _state_provider, _cmd_dispatcher
    _state_provider = state_provider
    _cmd_dispatcher = cmd_dispatcher
    _install_mirror()  # 先装 print 镜像再启其他线程，启动日志也能进网页
    try:
        httpd = _QuietWebServer(("0.0.0.0", port), _Handler)
    except OSError as e:
        print(f"[web] 端口 {port} 启动失败: {e}（Web 控制台不可用，其余功能不受影响）")
        return
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"[web] 控制台已启动: http://localhost:{port}（同局域网设备用 http://<本机IP>:{port}）")
