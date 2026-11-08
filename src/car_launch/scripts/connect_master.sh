#!/usr/bin/env bash
# =============================================================================
# connect_master.sh —— 让本车(4号车)接入主站车(默认 192.168.8.199)的网络。
#
# 目的：打通到主站 MQTT broker(192.168.8.199:1883) 的网络，之后才能
#       "进入云端系统"、接收云端下发给 robot_4 的目标/路径。
#       （WiFi/网段属于系统网络层，不是 ROS 代码，因此单独用此脚本处理）
#
# 用法：
#   方式A  WiFi 连主站热点（推荐，需要本车有 WiFi/USB 无线网卡）：
#       sudo bash connect_master.sh --wifi <热点SSID> <密码>
#
#   方式B  有线接同一交换机（主站在 192.168.7.0/24，本车加一个 7 号网地址）：
#       sudo bash connect_master.sh --wired <网卡名,如 eth0> 192.168.7.199
#
#   方式C  只检查连通性（不改任何网络配置）：
#       bash connect_master.sh --check
#
# 可调环境变量：
#       MASTER_IP=192.168.8.199   （主站地址）
# =============================================================================
set -u

MASTER_IP="${MASTER_IP:-192.168.8.199}"
MASTER_PORT=1883

check() {
    echo "== 检查到主站 ${MASTER_IP}:${MASTER_PORT} =="
    if ping -c 2 -W 1 "$MASTER_IP" >/dev/null 2>&1; then
        echo "  ping ${MASTER_IP}  -> OK"
    else
        echo "  ping ${MASTER_IP}  -> 不通"
    fi
    if timeout 3 bash -c "cat </dev/null >/dev/tcp/${MASTER_IP}/${MASTER_PORT}" 2>/dev/null; then
        echo "  MQTT ${MASTER_IP}:${MASTER_PORT} -> OPEN（已具备联调条件）"
        return 0
    else
        echo "  MQTT ${MASTER_IP}:${MASTER_PORT} -> 不通"
        return 1
    fi
}

mode="${1:-}"
case "$mode" in
    --check)
        check
        exit $?
        ;;
    --wifi)
        [ $# -ge 3 ] || { echo "用法: sudo bash $0 --wifi <SSID> <密码>"; exit 1; }
        ssid="$2"; pass="$3"
        command -v nmcli >/dev/null 2>&1 || { echo "[错误] 未安装 NetworkManager(nmcli)"; exit 1; }
        if ! nmcli -t -f TYPE dev | grep -q wifi; then
            echo "[错误] 本车没有可用 WiFi 网卡，请插入 USB WiFi 后再试"
            nmcli dev status
            exit 1
        fi
        nmcli radio wifi on || true
        echo "== 连接 WiFi [$ssid] =="
        nmcli dev wifi connect "$ssid" password "$pass" || { echo "[错误] WiFi 连接失败"; exit 1; }
        if check; then
            echo "== 已接入主站网络，可以启动 mqtt/run_all_action =="
        else
            echo "[警告] 已连上 WiFi，但仍到不了主站 ${MASTER_IP}，请确认热点网段是 192.168.7.x"
        fi
        ;;
    --wired)
        [ $# -ge 3 ] || { echo "用法: sudo bash $0 --wired <网卡名> 192.168.7.x"; exit 1; }
        iface="$2"; addr="$3"
        ip link set "$iface" up
        ip addr add "$addr"/24 dev "$iface" 2>/dev/null || echo "(地址 $addr 可能已存在，跳过添加)"
        ip route replace 192.168.7.0/24 dev "$iface" 2>/dev/null || true
        echo "== 已在本机 $iface 配置 ${addr}/24 =="
        if check; then
            echo "== 已接入主站网络，可以启动 mqtt/run_all_action =="
        else
            echo "[警告] 仍到不了主站 ${MASTER_IP}，请核对网线/交换机/网段"
        fi
        ;;
    *)
        echo "用法:"
        echo "  sudo bash $0 --wifi <热点SSID> <密码>       # WiFi 连主站"
        echo "  sudo bash $0 --wired ethX 192.168.7.199     # 有线同网段补地址"
        echo "  bash $0 --check                             # 只检查连通性"
        exit 1
        ;;
esac
