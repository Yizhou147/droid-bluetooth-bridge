[中文](README.md) | English

# droid-bluetooth-bridge — Native Bluetooth on the DRM takeover desktop (an HCI client for the vendor HAL)

When doing pure DRM/KMS takeover on the Xiaomi Pad 8 Pro (KWin owns the panel, the Android
framework is `stop`ped), Bluetooth is normally served by Android's own stack — which is dead
during takeover. This project's approach:

**Don't fight over the chip; become a client of the vendor Bluetooth HAL instead.**

```
Container bluetoothd / Plasma bluedevil (native UI)
        │  a real hci0 in the shared kernel
kernel hci_uart(H4) ← pty ← bthci-bridge ──binder──▶ android.hardware.bluetooth.IBluetoothHci/default
(zero new kernel modules)                             │ glink + btpower power-up + firmware download (all left to the HAL)
                                                      ▼  peach combo chip
```

HID keyboards/mice land as `/dev/input/eventN` through the kernel's uhid/HIDP — KWin/libinput
just reads them. No uinput, no custom pairing UI, no new kernel modules.

## Why it has to be this way

- While Android Bluetooth is working, `/sys/class/bluetooth` is **empty**: HCI travels over glink
  `/dev/bt_cp_ctrl` and never enters the kernel Bluetooth stack.
- The HAL process holds both `/dev/bt_cp_ctrl` and `/dev/ttyHS0` — grabbing the device nodes
  means fighting it for the same chip.
- During takeover, `stop` kills the Framework and `com.android.bluetooth`, but the HAL belongs to
  `class hal` and is not killed → **the HAL's client slot is free**. We take it; power, firmware
  and glink stay handled by the vendor code as designed.
- Power-up does **not** require Android to have Bluetooth switched on either: the bridge's
  `initialize` on a cold HAL triggers the HAL's own `initialize_aidl → OpenUart` (measured), so
  the takeover scripts never touch the Bluetooth switch.
- Exiting = kill the bridge process: once the tty closes, the kernel **automatically**
  unregisters hci0; the binder death notification returns the HAL to a client-less state, and the
  Android framework rebinds on its own after it revives → anland mode is unaffected.

## Hard format facts (all earned the hard way — don't re-explore)

| Fact | Evidence |
|---|---|
| `IBluetoothHci` has exactly 6 methods: `1 close / 2 initialize(oneway) / 3 sendAclData / 4 sendHciCommand / 5 sendIsoData / 6 sendScoData`, **no enable/disable**; power-up lives in the closed-source BtTpi (judged unreachable, don't touch) | Disassembled the device's `android.hardware.bluetooth-V1-ndk.so` `Bp*` code table |
| `byte[]` payloads in **both** directions do **not** include the H4 type byte; the kernel side needs it, so the bridge adds/strips it | Measured A/B with and without |
| Callbacks `2 hciEventReceived / 3 initializationComplete`; on Android 15/16 the parcel argument offset = `16 + align4(name_length*2+2)` (measured: 116); you must explicitly `setDataPosition` before every read | Measured |
| The HAL swallows replies for commands it sent internally (e.g. its own `HCI_Reset`) | logcat `Received event for command sent internally` |

## Repository layout

| Path | Content |
|---|---|
| `蓝牙原生方案.md` | Full design + complete measurement log (Chinese): fact baseline F1–F22, rejected routes A–G, milestones M0–M4, red lines, handover contract |
| `src/bthci-bridge.cpp` | Main body: pty+N_HCI+H4 to create hci0, `libbinder_ndk` client, H4 shuttle in both directions, **self-fuse** (checks `init.svc.surfaceflinger` every 10s and exits immediately once it is running — the bridge and Android's own Bluetooth stack must never be alive at the same time) |
| `build.sh` / `.github/workflows/build.yml` | NDK cross-compile (no NDK on this arm64 host, so it runs on GitHub Actions) |
| `artifact/bthci-bridge-aarch64/bthci-bridge` | CI artifact; the main project's `desk-takeover` uses this device's current build directly |
| `tools/m1-probe.c` | M1 load-bearing assumption verifier: attaches hci0 only, no HAL connection (measured, passes) |
| `tools/bthci-bridge.c` | Debug build with hand-rolled binder, **kept as a dead-end record** (reasons in design doc §11); its attach/H4 parts remain a reference implementation |
| `device-libs/` | `*-V1-ndk.so` pulled from the device (for disassembling the code table) |
| `ksu-module/` | Artifacts of the BtTpi/SELinux experiments; **that route was ruled out** (design doc §22–23), archive only |

## Usage (once wired up)

The main project's `droid-drm-takeover` `desk-takeover.sh` section 5c) automates everything:
push the artifact → run as Android root `nohup bthci-bridge --keep 0 &` → `bluetoothctl list`
inside the container sees the Controller → `BT-NATIVE OK`. `BT_BRIDGE=0` disables the whole
section; the `desk-stop` main flow, the 50s watchdog and the rollback path all
`pkill -x bthci-bridge` (**must use `-x`**; `-f` would also kill the enclosing `su -c` shell).

Manual one-off run (debugging):

```
adb -s emulator-5554 shell su -c '/data/local/tmp/bthci-bridge --keep 0'   # foreground
bluetoothctl list            # the container should now see a Controller (real BD_ADDR)
pkill -x bthci-bridge        # exit; hci0 is unregistered automatically
```

Container-side desktop readiness trio (already baked into the main project's desk-takeover):
create the `/dev/uhid` node (c 10:239), batch backfill `/dev/input/*` nodes + seat/uaccess tags,
and `kded6` must be running (hosts the bluedevil tray icon and the pairing agent).

## Red lines (strictly)

- **The bridge may only live inside a takeover round** (while `surfaceflinger=stopped`): Android
  framework alive + bridge alive = two clients fighting over the HAL/combo-chip power
  coordination; measured twice to push WiFi into an `is_driver_recovering` death loop, only
  recoverable by a full reboot.
- **Never** drive Bluetooth power ioctls manually (`0xBFAC–0xBFAF`, `0x54ed/0x54ee` on `ttyHS0`,
  geni `0xBFE4`); **never** write `/sys/class/rfkill/rfkill0` (bt_power); **never** touch BtTpi —
  all of it is the HAL's job.
- **Never** `setenforce 0`.
- No systemd / KernelSU autostart of any kind: the lifecycle is strictly bound to
  desk-takeover / desk-stop.
- For any counter-based probe ("did the chip reply"), first prove the counter can move at all
  (this project once spent a whole round of negative results on `g_cbEvents`, which was hard-wired at 0).

## Status

Latest working artifact: [Releases](https://github.com/Yizhou147/droid-bluetooth-bridge/releases)
(current v0.1.0 = self-fuse build, includes install/run/exit usage).

**M0–M3 all verified on real hardware**: real BD_ADDR, `power on`, scanning, a BLE mouse
**usable the moment it connects**, bidirectional ACL (`RX acl:209 / TX acl:50`), stable
multi-round takeovers within the same boot. M4 in progress: A2DP audio output untested, handover
regression after manually toggling Bluetooth in the Android UI untested, pairing keys are not
shared with Android (peripherals must be re-paired during takeover), long-run bridge stability
under observation.

Verified only on the Xiaomi Pad 8 Pro (piano / 25091RP04C, HyperOS + KernelSU); no guarantees for
other models.

## License

MIT, see [LICENSE](LICENSE). Build artifacts link only against the system library
`libbinder_ndk` and kernel UAPI headers — no GPL code is introduced (fully independent of the main
project `droid-drm-takeover`'s GPL-3.0).
