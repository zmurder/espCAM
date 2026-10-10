#!/usr/bin/env bash
#
# update_from_cloud.sh — 从坚果云拉取最新固件与接收器脚本，并重启服务（树莓派端）
#
#   ./update_from_cloud.sh                    # 用默认云端目录
#   ./update_from_cloud.sh 我的网盘:/固件      # 临时换云端目录
#
set -euo pipefail

REMOTE="${1:-jianguo:/tmp}"
DEST="/home/zyd/code/esp32_cam"   # 树莓派上的项目目录

rclone copy -v -P "$REMOTE/espCAM_WIFI.bin"        "$DEST/ota/"
rclone copy -v -P "$REMOTE/simple_udp_receiver.py" "$DEST/"

sudo systemctl restart espcam
echo "已更新并重启 espcam"
