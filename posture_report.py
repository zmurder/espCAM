#!/usr/bin/env python3
"""
坐姿检测历史报告 — 检测/不良次数柱形图
================================================
查询 simple_udp_receiver.py 落库的 SQLite（posture_history.db），按窗口出一张柱形图：
每组两根柱 = 检测次数（蓝） + 不良次数（橙），控制台附简要统计。

  窗口      分桶        柱子数
  day/天    按小时      当天 0 时 ~ 当前小时
  week/周   按天        本周一 ~ 今天
  month/月  按天        本月 1 日 ~ 今天
  quarter/季度 按周     本季度首周 ~ 本周
  year/年   按月        本年 1 月 ~ 本月

用法：
  python posture_report.py day            # 今日按小时
  python posture_report.py week|month|quarter|year
  python posture_report.py 天|周|月|季度|年   # 中文同义
  可选：--db 路径（默认 posture_history.db）、--no-show（只存 PNG 不弹窗）
"""

import argparse
import os
import sqlite3
import statistics
import sys
from collections import defaultdict
from datetime import datetime, timedelta

import matplotlib

if os.environ.get("MPLBACKEND") is None and "--no-show" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DB_DEFAULT = "posture_history.db"  # 与 simple_udp_receiver.py 的 HISTORY_DB 一致
BAD = (1, 2)       # 坐姿不良（BAD_NECK / BAD_SHOULDER）
VALID = (0, 1, 2)  # 有效判断帧（不良率分母；NOT_DET/UNRELIABLE/TOO_FAR 不计入）
EVENT_GAP_S = 5    # 连续不良帧间隔超过此值分断为两次"不良事件"

# 配色（dataviz 参考调色板，light 模式）
SURFACE = "#fcfcfb"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, BASELINE = "#e1e0d9", "#c3c2b7"
SERIES_TOTAL = "#2a78d6"  # 检测次数：分类槽 1（蓝）
SERIES_BAD = "#eb6834"    # 不良次数：分类槽 2（橙）
SERIES_RATE = "#e34948"   # 不良率曲线：分类槽 8（红，线形与柱形可区分）

WINDOW_ALIASES = {
    "day": "day", "天": "day", "今日": "day", "今天": "day",
    "week": "week", "周": "week", "本周": "week",
    "month": "month", "月": "month", "本月": "month",
    "quarter": "quarter", "季度": "quarter", "本季度": "quarter",
    "year": "year", "年": "year", "本年": "year",
}


def resolve_window(name):
    """窗口名 → (t0, 桶模式, 中文名)；右端=现在，起点=自然周期（今日/本周一/本月1日/季度首月/1月1日）"""
    now = datetime.now()

    def zero(d):
        return d.replace(hour=0, minute=0, second=0, microsecond=0)

    key = WINDOW_ALIASES.get(name.strip().lower()) or WINDOW_ALIASES.get(name.strip())
    if key == "day":
        return zero(now), now, "hour", "今日"
    if key == "week":
        return zero(now - timedelta(days=now.weekday())), now, "day", "本周"
    if key == "month":
        return zero(now.replace(day=1)), now, "day", "本月"
    if key == "quarter":
        return zero(now.replace(day=1, month=(now.month - 1) // 3 * 3 + 1)), now, "week", "本季度"
    if key == "year":
        return zero(now.replace(month=1, day=1)), now, "month", "本年"
    sys.exit(f"无法解析窗口 '{name}'：应为 day/week/month/quarter/year（或 天/周/月/季度/年）")


def load_rows(db_path, t0, t1):
    """窗口内全部 (ts, result, ratio)，按 ts 升序"""
    if not os.path.exists(db_path):
        sys.exit(f"历史库不存在: {db_path}（先运行 simple_udp_receiver.py 收集数据）")
    db = sqlite3.connect(db_path)
    rows = db.execute(
        "SELECT ts, result, ratio FROM posture_log WHERE ts >= ? AND ts <= ? ORDER BY ts",
        (t0.timestamp(), t1.timestamp())).fetchall()
    db.close()
    return rows


def bucket_key(dt, mode):
    if mode == "hour":
        return dt.hour
    if mode == "day":
        return dt.date()
    if mode == "week":
        return dt.date() - timedelta(days=dt.weekday())  # 所在周的周一
    return (dt.year, dt.month)


def bucket_axis(t0, t1, mode):
    """完整桶序列 + 标签（窗口内无数据的桶计 0，一并画出）"""
    if mode == "hour":
        ks = range(t0.hour, t1.hour + 1)
        return ks, [f"{k:02d}时" for k in ks]
    if mode == "day":
        ks, d = [], t0.date()
        while d <= t1.date():
            ks.append(d)
            d += timedelta(days=1)
        return ks, [f"{d:%m-%d}" for d in ks]
    if mode == "week":
        ks, d = [], t0.date() - timedelta(days=t0.weekday())  # 含 t0 的首周周一（季度起点不一定是周一）
        while d <= t1.date():
            ks.append(d)
            d += timedelta(weeks=1)
        return ks, [f"{d:%m-%d}" for d in ks]
    ks, (y, m) = [], (t0.year, t0.month)
    while (y, m) <= (t1.year, t1.month):
        ks.append((y, m))
        m += 1
        if m > 12:
            y, m = y + 1, 1
    if t0.year == t1.year:
        return ks, [f"{k[1]}月" for k in ks]
    return ks, [f"{k[0] % 100}年{k[1]}月" for k in ks]  # 跨年（如 Q4~Q1）补年份


def detect_events(rows, frame_iv):
    """连续 BAD 帧（间隔 ≤ EVENT_GAP_S）计 1 次不良事件 → [持续时长]"""
    durs, seg_start, last = [], None, None
    for ts, r, _ in rows:
        if r in BAD:
            if seg_start is None or ts - last > EVENT_GAP_S:
                if seg_start is not None:
                    durs.append(max(last - seg_start, frame_iv))
                seg_start = ts
            last = ts
        elif seg_start is not None:
            durs.append(max(last - seg_start, frame_iv))
            seg_start = None
    if seg_start is not None:
        durs.append(max(last - seg_start, frame_iv))
    return durs


def fmt_dur(sec):
    sec = max(0, int(sec))
    if sec < 60:
        return f"{sec}s"
    m, s = divmod(sec, 60)
    return f"{m}m{s:02d}s" if m < 60 else f"{sec // 3600}h{m % 60:02d}m"


def main():
    ap = argparse.ArgumentParser(description="坐姿检测历史报告（检测/不良次数柱形图）")
    ap.add_argument("window", nargs="?", default="day",
                    help="day/week/month/quarter/year（或 天/周/月/季度/年），默认 day")
    ap.add_argument("--db", default=DB_DEFAULT, help=f"历史库路径（默认 {DB_DEFAULT}）")
    ap.add_argument("--no-show", action="store_true", help="只保存 PNG，不弹交互窗口")
    args = ap.parse_args()

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]  # Windows 中文字体
    plt.rcParams["axes.unicode_minus"] = False

    t0, t1, mode, cname = resolve_window(args.window)
    rows = load_rows(args.db, t0, t1)
    if not rows:
        sys.exit(f"{cname}（{t0:%Y-%m-%d %H:%M} ~ 现在）暂无记录（PC 未运行接收器的时段无数据）")

    # ---- 计数 ----
    total, bad = defaultdict(int), defaultdict(int)
    n_valid = n_bad = 0
    for ts, r, _ in rows:
        k = bucket_key(datetime.fromtimestamp(ts), mode)
        total[k] += 1
        bad[k] += r in BAD
        n_valid += r in VALID
        n_bad += r in BAD
    ks, labels = bucket_axis(t0, t1, mode)
    tv = [total.get(k, 0) for k in ks]
    bv = [bad.get(k, 0) for k in ks]

    # ---- 控制台统计 ----
    diffs = [b[0] - a[0] for a, b in zip(rows, rows[1:]) if 0 < b[0] - a[0] < 60]
    frame_iv = statistics.median(diffs) if diffs else 1.5
    events = detect_events(rows, frame_iv)
    ev_desc = f"{len(events)} 次（最长 {fmt_dur(max(events))}）" if events else "0 次"
    print(f"══ 坐姿检测报告 · {cname}（{t0:%Y-%m-%d %H:%M} ~ 现在）══")
    print(f"检测 {len(rows):,} 帧 | 不良 {n_bad:,} 帧 | 不良率 {n_bad / n_valid:.1%}（不良/有效判断帧）" if n_valid
          else f"检测 {len(rows):,} 帧 | 无有效判断帧")
    print(f"不良事件: {ev_desc}（连续不良计 1 次，间隔>{EVENT_GAP_S}s 分段）")

    # ---- 柱形图 + 不良率曲线 ----
    unit = {"hour": "小时", "day": "天", "week": "周", "month": "月"}[mode]
    x = np.arange(len(ks))
    w = 0.38
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.bar(x - w / 2, tv, w, color=SERIES_TOTAL, label="检测次数")
    ax.bar(x + w / 2, bv, w, color=SERIES_BAD, label="不良次数")
    ax.set_title(f"坐姿检测 · {cname}（按{unit}）", color=INK, fontsize=13, loc="left")
    ax.set_ylabel("次数", color=INK2, fontsize=9)
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK2, labelsize=8)
    for s in ax.spines.values():
        s.set_color(BASELINE)
    step = max(1, -(-len(ks) // 16))  # 桶多时隔 N 个取刻度，防标签重叠（向上取整）
    ax.set_xticks(x[::step])
    ax.set_xticklabels(labels[::step], rotation=45 if mode == "day" and len(ks) > 12 else 0, ha="right")
    if max(bv) > 0:  # 单点直标：仅标最高的不良柱
        i = bv.index(max(bv))
        ax.annotate(f"{bv[i]}", xy=(x[i] + w / 2, bv[i]), color=INK2, fontsize=9,
                    xytext=(0, 4), textcoords="offset points", ha="center")

    # 不良率曲线（右轴 0-100%）：不良/检测，与两根柱直接对应；无检测的桶断开不画
    rv = [bv[i] / tv[i] * 100 if tv[i] else np.nan for i in range(len(ks))]
    ax2 = ax.twinx()
    ax2.plot(x, rv, color=SERIES_RATE, linewidth=2, marker="o", markersize=5,
             label="不良率(右轴)")
    ax2.set_ylim(0, 100)
    ax2.set_ylabel("不良率", color=INK2, fontsize=9)
    ax2.yaxis.set_major_formatter(matplotlib.ticker.PercentFormatter())
    ax2.tick_params(colors=INK2, labelsize=8)
    ax2.spines["top"].set_visible(False)
    for s in ("right",):
        ax2.spines[s].set_color(BASELINE)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, frameon=False, loc="upper left", fontsize=9)

    out = f"posture_report_{mode}_{t1:%Y%m%d}.png"
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"图表已保存: {out}")
    if args.no_show:
        return
    plt.show()


if __name__ == "__main__":
    main()
