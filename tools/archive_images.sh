#!/usr/bin/env bash
#
# archive_images.sh — 接收器图片归档：打包 → 上传 → 删源图（树莓派端）
#
# 逐包端到端处理，任何一步失败立即停止，**绝不在上传成功前删源图**：
#   1. 收集 received_images/ 下「修改时间早于 --min-age 分钟」的图片
#      （避开接收器正在写入的半截文件，所以服务开着也能安全归档）
#   2. 按累计大小切包，每个 ≤ --max-size，命名 received_images_<日期>_<序号>.tar.gz
#      包内保留 <设备ID>/<文件名> 相对路径，解压即还原目录结构
#   3. 校验：gzip -t 完整性 + 包内条目数与清单行数一致
#   4. rclone copy 上传该包
#   5. 上传成功后，按清单删除 received_images/ 里已归档的源图（只删文件，保留设备目录）
#
# 用法: tools/archive_images.sh [选项]
#   -s, --src DIR        图片根目录（默认：自动探测——脚本同级 / 上一级 / 当前目录下的 received_images）
#   -o, --out DIR        压缩包输出目录（默认：received_images 同级的 image_archives）
#   -r, --remote PATH    rclone 目标（默认 jianguo:/tmp）
#   -m, --max-size SIZE  单个压缩包上限，支持 400M / 1G / 纯字节（默认 400M）
#   -a, --min-age MIN    只归档修改时间早于 MIN 分钟的文件（默认 2）
#   -l, --log FILE       运行日志路径（默认 received_images 同级的 log/archive.log）
#                        脚本自身的每条记录 + rclone 的传输日志都写在这里
#       --no-log         不写日志文件，只输出到终端
#   -p, --purge-local    上传成功后删除本地压缩包（默认保留在 --out）
#   -u, --no-upload      只打包不上传（同样不删源图）
#   -n, --dry-run        只列出打包计划，不打包、不上传、不删源图、不写日志
#   -h, --help
#
# 注意：只归档图片（*.png / *.jpg）。源目录里的其他文件（如 rclone.log）会被忽略，
#       既不会被打包，也不会被删除——删除严格按打包时的清单进行。
#
# 例：每天 3:10 归档上传（树莓派 crontab -e；日志已由 --log 落盘，无需再重定向）
#   10 3 * * * /home/zyd/code/esp32_cam/tools/archive_images.sh -p
#
set -euo pipefail

# ---------- 默认配置 ----------
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

SRC=""                               # 源图片目录（留空=自动探测，见 detect_src）
OUT=""                               # 压缩包输出目录（留空=与 received_images 同级）
REMOTE="jianguo:/tmp"                # rclone 目标
MAX_SIZE="400M"                      # 单包上限
MIN_AGE=2                            # 只处理 N 分钟前修改的文件
LOG_FILE=""                          # 运行日志（留空=与 received_images 同级的 log/archive.log）
LOG_ENABLED=1
DRY_RUN=0
DO_UPLOAD=1
PURGE_LOCAL=0

# 打印文件头部的注释块（第 2 行起，到 set -euo pipefail 前一行）
usage() { awk 'NR>1 { if ($0 ~ /^set -euo pipefail/) exit; print }' "$0"; }

# 自动探测图片目录：脚本放在项目根或 tools/ 下都能找到，也可放在任意位置用 -s 显式指定
detect_src() {
  local c
  for c in "$SCRIPT_DIR/received_images" "$SCRIPT_DIR/../received_images" "$PWD/received_images"; do
    if [ -d "$c" ]; then (cd -- "$c" && pwd); return 0; fi
  done
  return 1
}

# ---------- 工具函数 ----------
# 所有输出同时进终端和日志文件（日志写入失败不影响主流程）
_log_to_file() { [ "$LOG_ENABLED" = 1 ] && printf '%s\n' "$1" >> "$LOG_FILE" 2>/dev/null || true; }
log() {
  local msg="[$(date '+%F %T')] $*"
  printf '%s\n' "$msg"
  _log_to_file "$msg"
}
die() {
  local msg="[$(date '+%F %T')] 错误: $*"
  printf '%s\n' "$msg" >&2
  _log_to_file "$msg"
  exit 1
}

human() {
  awk -v b="$1" 'BEGIN{ n=split("B KB MB GB TB",u," "); i=1; while (b>=1024 && i<n) { b/=1024; i++ } printf "%.1f%s", b, u[i] }'
}

parse_size() {
  local s="$1" n="${1%[KkMmGg]}"
  case "${s: -1}" in
    K|k) echo $(( n * 1024 )) ;;
    M|m) echo $(( n * 1024 * 1024 )) ;;
    G|g) echo $(( n * 1024 * 1024 * 1024 )) ;;
    *)   echo "$s" ;;
  esac
}

# ---------- 参数解析 ----------
while [ $# -gt 0 ]; do
  case "$1" in
    -s|--src)         SRC="$2"; shift 2 ;;
    -o|--out)         OUT="$2"; shift 2 ;;
    -r|--remote)      REMOTE="$2"; shift 2 ;;
    -m|--max-size)    MAX_SIZE="$2"; shift 2 ;;
    -a|--min-age)     MIN_AGE="$2"; shift 2 ;;
    -l|--log)         LOG_FILE="$2"; LOG_ENABLED=1; shift 2 ;;
    --no-log)         LOG_ENABLED=0; shift ;;
    -p|--purge-local) PURGE_LOCAL=1; shift ;;
    -u|--no-upload)   DO_UPLOAD=0; shift ;;
    -n|--dry-run)     DRY_RUN=1; shift ;;
    -h|--help)        usage; exit 0 ;;
    *) printf '未知参数: %s\n\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

# ---------- 路径解析与前置检查 ----------
# 源目录：-s 优先；否则自动探测（脚本同级 / 上一级 / 当前目录下的 received_images）
if [ -z "$SRC" ]; then
  SRC="$(detect_src)" || die "找不到 received_images 目录（脚本位于 $SCRIPT_DIR），请用 -s 指定"
else
  [ -d "$SRC" ] || die "源目录不存在: $SRC"
fi
SRC="$(cd -- "$SRC" && pwd)"                  # 归一化为绝对路径
BASE_DIR="$(dirname -- "$SRC")"               # 通常是接收器的工作目录

# 输出目录与日志默认跟 received_images 平级，避免依赖脚本放在哪一层
[ -n "$OUT" ] || OUT="$BASE_DIR/image_archives"
[ -n "$LOG_FILE" ] || LOG_FILE="$BASE_DIR/log/archive.log"

# 试运行承诺「不改动任何文件」，所以也不写日志
if [ "$DRY_RUN" = 1 ]; then LOG_ENABLED=0; fi
if [ "$LOG_ENABLED" = 1 ]; then
  LOG_DIR="$(dirname -- "$LOG_FILE")"
  mkdir -p -- "$LOG_DIR" || die "无法创建日志目录: $LOG_DIR"
fi

MAX_BYTES="$(parse_size "$MAX_SIZE")"
case "$MAX_BYTES" in ''|*[!0-9]*) die "无法解析 --max-size: $MAX_SIZE ;;; ('$MAX_BYTES')" ;; esac
[ "$MAX_BYTES" -gt 0 ] || die "--max-size 必须大于 0"

# 实际打包预算留 3% 余量：tar 块填充 + 压缩包首尾开销（PNG 本身压不动，包≈原大小）
BUDGET=$(( MAX_BYTES * 97 / 100 ))

if [ "$DRY_RUN" = 1 ]; then
  TMPD="$(mktemp -d)"
  OUT="$TMPD"
  DO_UPLOAD=0
  trap 'rm -rf "$TMPD"' EXIT
else
  mkdir -p "$OUT"
  if [ "$DO_UPLOAD" = 1 ] && ! command -v rclone >/dev/null 2>&1; then
    die "找不到 rclone（未安装或不在 PATH），可用 -u 只打包"
  fi
  # 防重入（cron 上一轮没跑完时不要叠加；Git Bash 无 flock 时自动跳过）
  if command -v flock >/dev/null 2>&1; then
    exec 9>"$OUT/.archive.lock"
    flock -n 9 || die "另一个归档进程正在运行（$OUT/.archive.lock）"
  fi
fi

# ---------- 收集待归档文件 ----------
DATE="$(date +%Y%m%d)"
LIST="$OUT/.pending.txt"
# 白名单：只收图片。源目录里的其他文件（rclone.log、日志、临时文件等）一律忽略，
# 既不会被打包也不会被删除——删除严格按打包时的清单进行
find_args=( "$SRC" -type f \( -iname '*.png' -o -iname '*.jpg' \) -size +0c )
if [ "$MIN_AGE" -gt 0 ]; then
  find_args+=( -mmin "+$MIN_AGE" )
fi
# 输出目录若嵌在源目录内（-o 指到 received_images 下面），要排除，免得把压缩包自己又打进去
case "$OUT" in "$SRC"/*) find_args+=( -not -path "$OUT/*" ) ;; esac
# %P = 相对源目录的路径，打包时保留 <设备ID>/<文件名> 结构
find "${find_args[@]}" -printf '%s\t%P\n' 2>/dev/null | LC_ALL=C sort -t $'\t' -k2 > "$LIST"

total_count=$(wc -l < "$LIST")
total_bytes=$(awk -F'\t' '{s+=$1} END{print s+0}' "$LIST")

if [ "$total_count" -eq 0 ]; then
  log "没有可归档的图片（$SRC 下没有早于 $MIN_AGE 分钟的文件）"
  exit 0
fi
log "待归档: $total_count 张 / $(human "$total_bytes")，单包上限 $MAX_SIZE"

# 空间预检：输出目录所在分区至少要有 1.1 倍待打包量
if [ "$DRY_RUN" = 0 ]; then
  avail=$(df -PB1 "$OUT" | awk 'NR==2{print $4}')
  need=$(( total_bytes + total_bytes / 10 ))
  if [ "$avail" -lt "$need" ]; then
    die "输出目录可用空间不足：需要约 $(human "$need")，$OUT 可用 $(human "$avail")"
  fi
fi

# ---------- 逐包处理 ----------
idx=0
pkg_count=0
done_count=0
done_bytes=0
cur_bytes=0
cur_count=0
pkg=""
manifest=""

new_pkg() {
  # 序号自增并跳过已存在的包名，保证同日多轮运行不覆盖历史包
  while :; do
    idx=$(( idx + 1 ))
    local tag
    tag="$(printf '%02d' "$idx")"
    pkg="$OUT/received_images_${DATE}_${tag}.tar.gz"
    manifest="$OUT/received_images_${DATE}_${tag}.files"
    if [ ! -e "$pkg" ]; then break; fi
  done
  : > "$manifest"
  cur_bytes=0
  cur_count=0
}

flush_pkg() {
  if [ "$cur_count" -eq 0 ]; then return 0; fi

  log "打包 $(basename "$pkg") —— $cur_count 张 / $(human "$cur_bytes")"
  if [ "$DRY_RUN" = 0 ]; then
    tar -czf "$pkg" -C "$SRC" -T "$manifest" < /dev/null || die "打包失败: $pkg"
    gzip -t "$pkg" < /dev/null || die "压缩包损坏: $pkg"
    local n_pkg n_expect
    n_pkg=$(tar -tzf "$pkg" | wc -l)
    n_expect=$(grep -c . "$manifest" || true)   # 非空行数
    if [ "$n_pkg" -ne "$n_expect" ]; then
      die "包内条目数($n_pkg)与清单($n_expect)不一致: $pkg"
    fi
    log "  校验通过：$(human "$(stat -c %s "$pkg")")"
  fi

  if [ "$DO_UPLOAD" = 1 ]; then
    log "  上传 -> $REMOTE"
    # rclone 的传输记录也写进日志文件（-P 进度条仍显示在终端，不污染日志）
    rclone_log_args=()
    if [ "$LOG_ENABLED" = 1 ]; then
      rclone_log_args=( --log-file "$LOG_FILE" --log-level INFO )
    fi
    rc=0
    rclone copy -v -P "${rclone_log_args[@]}" "$pkg" "$REMOTE" < /dev/null || rc=$?
    if [ "$rc" -ne 0 ]; then
      # rclone 的消息进了日志文件，失败时把末尾几行回显到终端，免得只看到一句"上传失败"
      log "  rclone 退出码: $rc"
      if [ "$LOG_ENABLED" = 1 ] && [ -s "$LOG_FILE" ]; then
        printf '  --- 日志末尾 15 行 ---\n' >&2
        tail -n 15 -- "$LOG_FILE" >&2
        printf '  ---------------------\n' >&2
      fi
      die "rclone 上传失败: $pkg（源图未删除，重跑本脚本即可续传）"
    fi

    # 只有上传成功了才删源图，且严格按清单删（不碰本轮未归档的新图）
    # 循环变量必须 local：否则 EOF 时会清空外层 while read 的 rel，导致后续写出空条目
    local entry del_fail=0
    while IFS= read -r entry; do
      [ -n "$entry" ] || continue
      rm -f -- "$SRC/$entry" || del_fail=1
    done < "$manifest"
    if [ "$del_fail" = 0 ]; then
      log "  已删除 $cur_count 张源图"
    else
      log "  警告：部分源图删除失败（多为权限问题），下次会重新归档"
    fi

    if [ "$PURGE_LOCAL" = 1 ]; then
      rm -f -- "$pkg"
      log "  已删除本地压缩包（-p）"
    fi
  fi

  pkg_count=$(( pkg_count + 1 ))
  done_count=$(( done_count + cur_count ))
  done_bytes=$(( done_bytes + cur_bytes ))

  # 重置计数：下一个文件到来时会开新包（否则会继续往已封的包里累加）
  cur_count=0
  cur_bytes=0
}

while IFS=$'\t' read -r size rel; do
  if [ -z "$rel" ]; then continue; fi

  # 攒够一个包就封包；单个文件超上限时单独成包（正常 PNG 不会触发）
  if [ "$cur_count" -gt 0 ] && [ $(( cur_bytes + size )) -gt "$BUDGET" ]; then
    flush_pkg
  fi
  if [ "$cur_count" -eq 0 ]; then
    if [ "$size" -gt "$MAX_BYTES" ]; then
      log "警告：单文件超过上限 ${MAX_SIZE}: $rel（$(human "$size")），单独成包"
    fi
    new_pkg
  fi

  if [ -z "$rel" ]; then continue; fi   # 双保险：绝不写入空条目（空条目会让包内条目数对不上）
  printf '%s\n' "$rel" >> "$manifest"
  cur_bytes=$(( cur_bytes + size ))
  cur_count=$(( cur_count + 1 ))
done < "$LIST"

flush_pkg

# ---------- 汇总 ----------
if [ "$DRY_RUN" = 1 ]; then
  log "试运行结束：将生成 $pkg_count 个包 / $done_count 张图 / $(human "$done_bytes")（未打包、未上传、未删源图）"
elif [ "$DO_UPLOAD" = 1 ]; then
  log "完成：$pkg_count 个包 / $done_count 张图 / $(human "$done_bytes") 已上传到 $REMOTE"
  if [ "$PURGE_LOCAL" = 0 ]; then
    log "本地压缩包保留在 $OUT（加 -p 可在上传后自动删除）"
  fi
else
  log "完成：$pkg_count 个包 / $done_count 张图 / $(human "$done_bytes")，仅打包未上传（源图未删）"
fi
if [ "$LOG_ENABLED" = 1 ]; then
  log "运行日志: $LOG_FILE"
fi
