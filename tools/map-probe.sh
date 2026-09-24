#!/bin/sh
# 实测 IBluetoothHci 方法表：逐个事务码空参 oneway 发出去，
# 从 HAL 自己的日志读它命中了哪个同名方法（BluetoothHci::enable/close/...）。
# 前置：安卓蓝牙已关（svc bluetooth disable）、/data/local/tmp/bthci-bridge 已推送。
# 用法：sh tools/map-probe.sh "1 2 3 4 5 6 7"
for c in ${1:-1 2 3 4 5 6 7 8 9 10}; do
    echo "======== code $c $(date +%H:%M:%S)"
    adb -s emulator-5554 shell "su -c '/data/local/tmp/bthci-bridge --map $c --keep 2'" 2>&1 | grep -E "MAP|✗" | sed 's/^/    /'
    sleep 1
    adb -s emulator-5554 shell "su -c 'logcat -d 2>/dev/null | grep -E \"aidl_service|BluetoothHci\" | tail -3'" | sed 's/^/    HAL| /'
done
