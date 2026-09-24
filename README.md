# droid-bluetooth-bridge — 让 DRM 接管桌面直接用上蓝牙（vendor HAL 的 HCI 客户端）

在 Xiaomi Pad 8 Pro 上做纯 DRM/KMS 接管（KWin 持屏、安卓 framework 被 `stop`）时，
蓝牙本该由安卓自己的栈负责，而接管期那套栈是死的。本项目的做法是：

**不去抢芯片，而是去做 vendor 蓝牙 HAL 的客户端。**

```
容器 bluetoothd / Plasma bluedevil（原生 UI）
        │  共享内核里的真 hci0
内核 hci_uart(H4) ← pty ← bthci-bridge ──binder──▶ android.hardware.bluetooth.IBluetoothHci/default
（零新内核模块）                                    │ glink + btpower 上电 + 固件下载（全归 HAL）
                                                   ▼  peach combo 芯片
```

HID 键盘/鼠标经内核 HIDP 直接落成 `/dev/input/eventN`，KWin/libinput 照常读取 —— 不需要
uinput、不需要自造配对 UI、不需要新内核模块。

## 为什么必须这样

- 安卓蓝牙工作时 `/sys/class/bluetooth` 是**空的**：HCI 走 glink `/dev/bt_cp_ctrl`，从不进内核蓝牙栈。
- HAL 进程同时持有 `/dev/bt_cp_ctrl` 与 `/dev/ttyHS0` —— 抢设备节点就是与它抢同一块芯片。
- 接管期 `stop` 杀掉的是 Framework 与 `com.android.bluetooth`，而 HAL 属 `class hal` 不会被杀
  → **HAL 的客户端位空出来了**，我们坐上去，电源/固件/glink 仍由 vendor 代码按设计处理。
- 回滚时我们 `disable()+close()` 退场，安卓 framework 重启后自己重新绑定 → anland 模式无感。

## 目录

| 路径 | 内容 |
|---|---|
| `蓝牙原生方案.md` | 完整设计文档：实测事实基线 F1–F22、已否路线 A–G、架构、里程碑 M0–M4、红线、回归面 |
| `src/bthci-bridge.cpp` | 主体：pty+N_HCI+H4 造 hci0、`libbinder_ndk` 客户端、H4 双向搬运、体面退场 |
| `build.sh` | NDK 交叉编译（aarch64 / API 33 / `-static-libstdc++`） |
| `.github/workflows/build.yml` | 云端构建（本机是 arm64 Linux 且 NDK 只有 x86_64 host 版，故放 CI） |
| `tools/m1-probe.c` | M1 承重假设验证器：只 attach hci0 不接 HAL（已实测通过） |
| `tools/bthci-bridge.c` | 手写 binder 的调试版，**判死保留**（原因见方案 §11），其 attach/H4 部分仍是参考实现 |

## 用法（开发期）

```
sh build.sh                       # 需 NDK；或触发 CI 下载 artifact
adb push out/bthci-bridge /data/local/tmp/bthci-bridge
adb shell su -c 'chmod 755 /data/local/tmp/bthci-bridge'
adb shell su -c 'svc bluetooth disable'          # 让安卓自己释放 HAL 客户端位
adb shell su -c '/data/local/tmp/bthci-bridge --keep 600'
```

另开一个终端确认：`ls /sys/class/bluetooth`（应有 `hci0`）、容器内 `bluetoothctl list`。
结束后安卓侧：`adb shell su -c 'svc bluetooth enable'`。

## 红线（务必）

- **绝不**从容器/本程序手动驱动蓝牙电源 ioctl（`ttyHS0` 上 `0xBFAC–0xBFAF`、`0x54ed/0x54ee`、
  geni grant `0xBFE4`）：peach 是 WiFi+BT combo，电源域由 `btpower.ko`/cnss 协调，越过它 = SSR/
  hangdetect 复位，本项目已因此**强制重启三次**。
- **绝不** `setenforce 0`；**绝不**碰 `/sys/class/rfkill/rfkill0`（那是 HAL 的开关入口）。
- 清理进程用 `pkill -x bthci-bridge`：`pkill -f` 会连 `su -c` 那层 shell 一起杀掉（实测踩过）。
- 不设任何 systemd / KernelSU 自启：生命周期必须挂在 `desk-takeover` / `desk-stop` 上。

## 状态

M0（binder 可达性）与 M1（零模块造出 hci0）已实测通过；M1b（bluedevil 认领 adapter）与
M2（接真 HCI）待做。仅在小米平板 8 Pro（piano / 25091RP04C，HyperOS + KernelSU）上验证过，
不保证适用于其它机型。

## 许可

MIT，见 [LICENSE](LICENSE)。构建产物只链接系统库 `libbinder_ndk` 与内核 UAPI 头，不引入 GPL 代码（与主项目 `droid-drm-takeover` 的 GPL-3.0 相互独立）。
