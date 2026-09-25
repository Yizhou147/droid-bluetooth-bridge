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

HID 键盘/鼠标经内核 uhid/HIDP 落成 `/dev/input/eventN`，KWin/libinput 照常读取 —— 不需要
uinput、不需要自造配对 UI、不需要新内核模块。

## 为什么必须这样

- 安卓蓝牙工作时 `/sys/class/bluetooth` 是**空的**：HCI 走 glink `/dev/bt_cp_ctrl`，从不进内核蓝牙栈。
- HAL 进程同时持有 `/dev/bt_cp_ctrl` 与 `/dev/ttyHS0` —— 抢设备节点就是与它抢同一块芯片。
- 接管期 `stop` 杀掉的是 Framework 与 `com.android.bluetooth`，而 HAL 属 `class hal` 不会被杀
  → **HAL 的客户端位空出来了**，我们坐上去，电源/固件/glink 仍由 vendor 代码按设计处理。
- 上电也**不需要**安卓自己开关蓝牙：桥对冷 HAL 发 `initialize` 就会触发 HAL 自己的
  `initialize_aidl → OpenUart`（实测），所以接管脚本不再碰蓝牙开关。
- 退场 = 杀桥进程：tty 一关，内核**自动**注销 hci0；binder 死亡通知让 HAL 回到无人客户端态，
  安卓 framework 复活后自己重绑 → anland 模式无感。

## 定死的格式事实（都是踩出来的，别再试）

| 事实 | 依据 |
|---|---|
| `IBluetoothHci` 只有 6 个方法：`1 close / 2 initialize(oneway) / 3 sendAclData / 4 sendHciCommand / 5 sendIsoData / 6 sendScoData`，**没有 enable/disable**，上电在闭源 BtTpi（已判死，勿碰） | 反汇编设备 `android.hardware.bluetooth-V1-ndk.so` 的 `Bp*` 码表 |
| 两个方向的 `byte[]` 都**不含 H4 类型字节**；内核那侧要带，桥自己加/减 | 含/不含实测对照 |
| 回调 `2 hciEventReceived / 3 initializationComplete`；Android 15/16 parcel 参数起点 = `16 + align4(名字长度*2+2)`（实测 116），读前必须显式 `setDataPosition` | 实测 |
| HAL 会吞掉它内部已发命令的回包（如它自己发过的 `HCI_Reset`） | logcat `Received event for command sent internally` |

## 目录

| 路径 | 内容 |
|---|---|
| `蓝牙原生方案.md` | 完整设计与全程实测记录：事实基线 F1–F22、已否路线 A–G、里程碑 M0–M4、红线、交接口 |
| `src/bthci-bridge.cpp` | 主体：pty+N_HCI+H4 造 hci0、`libbinder_ndk` 客户端、H4 双向搬运、**自熔断**（每 10s 查 `init.svc.surfaceflinger`，一旦 running 立刻自退——桥与安卓蓝牙栈绝不许同时活着） |
| `build.sh` / `.github/workflows/build.yml` | NDK 交叉编译（本机 arm64 无 NDK，放 GitHub Actions） |
| `artifact/bthci-bridge-aarch64/bthci-bridge` | CI 产物，主项目 `desk-takeover` 直接用这台设备的现役版 |
| `tools/m1-probe.c` | M1 承重假设验证器：只 attach hci0 不接 HAL（已实测通过） |
| `tools/bthci-bridge.c` | 手写 binder 的调试版，**判死保留**（原因见方案 §11），attach/H4 部分仍是参考实现 |
| `device-libs/` | 从设备拉回的 `*-V1-ndk.so`（反汇编定码表用） |
| `ksu-module/` | BtTpi/SELinux 实验期产物，**该路线已判死**（方案 §22–23），仅存档 |

## 用法（已接线后）

主项目 `droid-drm-takeover` 的 `desk-takeover.sh` 5c) 段自动完成：push 产物 → 安卓侧 root
`nohup bthci-bridge --keep 0 &` → 容器 `bluetoothctl list` 见 Controller 即 `BT-NATIVE OK`。
`BT_BRIDGE=0` 可整段关闭；`desk-stop` 主流程、50s 看门狗、rollback 三条路径都会
`pkill -x bthci-bridge`（**必须 `-x`**，`-f` 会连 `su -c` 那层 shell 一起杀）。

手动单跑（调试）：

```
adb -s emulator-5554 shell su -c '/data/local/tmp/bthci-bridge --keep 0'   # 前台
bluetoothctl list            # 容器里应见 Controller（真 BD_ADDR）
pkill -x bthci-bridge        # 退场，hci0 自动注销
```

容器侧桌面可用性三件套（主项目已固化进 desk-takeover）：`/dev/uhid`（c 10:239）建节点、
`/dev/input/*` 批量补号 + seat/uaccess 标签、`kded6` 必须拉起（bluedevil 托盘/配对 agent 的宿主）。

## 红线（务必）

- **桥只能在接管轮内活着**（`surfaceflinger=stopped` 时）：安卓框架活着 + 桥活着 = 两个客户端抢
  HAL/combo 芯片电源协调，实测两次把 WiFi 打进 `is_driver_recovering` 死循环、只能整机重启。
- **绝不**手动驱动蓝牙电源 ioctl（`ttyHS0` 上 `0xBFAC–0xBFAF`、`0x54ed/0x54ee`、geni `0xBFE4`），
  **绝不**写 `/sys/class/rfkill/rfkill0`（bt_power），**绝不**碰 BtTpi——全由 HAL 自己做。
- **绝不** `setenforce 0`。
- 不设任何 systemd / KernelSU 自启：生命周期严格挂在 desk-takeover / desk-stop 上。
- 判"芯片有没有回包"这类计数器判据，先自证计数器会动（本项目曾拿恒为 0 的 `g_cbEvents`
  连否了一整轮实验）。

## 状态

**M0–M3 全部实测通过**：真 BD_ADDR、`power on`、扫描、BLE 鼠标**连上即可用**、ACL 双向
（`RX acl:209 / TX acl:50`）、同开机多轮往返稳定。M4 收尾中：A2DP 出声未实测、安卓 UI 手动
开关蓝牙的交还回归未测、配对密钥不与安卓共享（接管期连外设需重配一次）、桥长跑稳定性观察中。

仅在小米平板 8 Pro（piano / 25091RP04C，HyperOS + KernelSU）上验证过，不保证适用于其它机型。

## 许可

MIT，见 [LICENSE](LICENSE)。构建产物只链接系统库 `libbinder_ndk` 与内核 UAPI 头，不引入 GPL 代码
（与主项目 `droid-drm-takeover` 的 GPL-3.0 相互独立）。
