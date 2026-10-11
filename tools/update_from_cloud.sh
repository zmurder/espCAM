#!/usr/bin/env bash
#
# update_from_cloud.sh — 从坚果云拉取固件与接收器文件，并重启服务（树莓派端）
#
#   ./update_from_cloud.sh                    # 用默认云端目录
#   ./update_from_cloud.sh 我的网盘:/固件      # 临时换云端目录
#
# 对应的手敲命令：
#   rclone copy -v -P jianguo:/tmp/espCAM_WIFI.bin        /home/zyd/code/esp32_cam/ota/
#   rclone copy -v -P jianguo:/tmp/simple_udp_receiver.py /home/zyd/code/esp32_cam/
#   rclone copy -v -P jianguo:/tmp/web_ui.py              /home/zyd/code/esp32_cam/
#
# 三个文件互相独立：云端没有的那个只跳过、不影响其余文件（所以只想更新其中一个时，
# 另外两个没传上去也没关系）。内容没变的文件 rclone 不会重传，全列着不费流量。
# 结束时汇总本次真正换了内容的文件。
#
set -euo pipefail

REMOTE="${1:-jianguo:/tmp}"
DEST="/home/zyd/code/esp32_cam"   # 树莓派上的项目目录

updated=()                        # 本次真的换了内容的文件（末尾汇总）

sync() {  # sync <云端文件名> <目标目录>
  local f="$1" d="$2" old new
  # 下载前后各算一次校验和：内容真的变了才记进"本次更新"
  # （比时间戳可靠——WebDAV 的时间精度可能和本地对不上）
  # 末尾的 || true 是给 set -e 用的：文件还不存在时 md5sum 会失败，
  # 不兜住的话整个脚本会在这里直接退出（首次部署/新加文件时必踩）
  old="$(md5sum "$d/$f" 2>/dev/null | cut -d' ' -f1 || true)"
  if ! rclone copy -v -P "$REMOTE/$f" "$d/" < /dev/null; then
    echo "  （$f 跳过：云端没有或下载失败）"
    return 0
  fi
  new="$(md5sum "$d/$f" 2>/dev/null | cut -d' ' -f1 || true)"
  [ "$old" = "$new" ] || updated+=("$f")
}

sync espCAM_WIFI.bin        "$DEST/ota"
sync simple_udp_receiver.py "$DEST"
sync web_ui.py              "$DEST"

sudo systemctl restart espcam
echo "本次更新: ${updated[*]:-无（三个文件都是最新的）}"
echo "已重启 espcam"
