# 刷入方法（KernelSU）

1. 打包：在**平板容器里**执行（四条命令，逐条回车）
   - `cd /home/xieyizhou/Documents/XiaomiPad8Pro-drm-display/droid-bluetooth-bridge/ksu-module`
   - `zip -r ../droid-bt-bridge-sepolicy.zip . `
   - `cd ..`
   - `ls -l droid-bt-bridge-sepolicy.zip`
2. KernelSU Manager → 模块 → 从本地安装 → 选 `droid-bt-bridge-sepolicy.zip` → **重启**。
3. 重启后先别开蓝牙，回我一声，我用 `service call vendor.qti.hardware.bttpi.IBtTpi/default <code>`
   复测：如果不再是 `EX_SECURITY`，说明是 binder 层拒绝，路线立刻打通；
   如果仍是 `EX_SECURITY`，就证明是 BtTpi 用户态自查 SID，那我们改走"让安卓自己上电 + 抢 HAL 客户端位"那条路
   （本轮已把芯片点亮到 HALfd=2、IBS 握手成功，只差 HCI 真正落地的最后一个码位）。

注：规则里没有 permissive、没有 `setenforce 0`，只加 `allow`；随时可在 KernelSU 里停用该模块回滚。
