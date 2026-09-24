// bthci-bridge.cpp — DRM 接管期的蓝牙 HCI 桥（NDK/libbinder_ndk 版）
//
// 一个进程干两件事（在安卓侧以 root 运行）：
//   1) 开一对 pty，在 slave 上挂 N_HCI + HCIUARTSETPROTO(H4) → 在共享内核里注册出真 hci0
//      （已实测：零新内核模块；进程退出 → tty 关闭 → hci_unregister_dev 自动回收）
//   2) 做 vendor HAL 的 binder 客户端：AServiceManager_getService("…IBluetoothHci/default")
//      → initialize(我们的回调) → enable → 把 pty 上的 H4 帧与 HAL 双向搬运
//
// 为什么不自己拼 binder 事务：Android 15/16 的 libbinder 在 Parcel 里加了 'SYST'/'VNDR'
// tuning header（由 Parcel::write 序列化，手写客户端伪造不出来 → servicemanager 直接
// 回 EX_SECURITY）。详见 蓝牙原生方案.md §11。用 AIBinder_prepareTransaction 就彻底交给
// 平台自己写头，我们只负责参数。
//
// 绝对不做：不 open /dev/bt_cp_ctrl、不 open /dev/ttyHS0、不打任何 btpower/geni 电源 ioctl、
// 不碰 /sys/class/rfkill/rfkill0 —— 上电与固件下载是 HAL 的职责，我们越界就是把平板搞重启。
//
// 编译：见 .github/workflows/build-bthci.yml（NDK r27，aarch64，c++_static）
// 运行：adb push bthci-bridge /data/local/tmp/
//       adb shell su -c '/data/local/tmp/bthci-bridge --keep 3600'

#include <android/binder_ibinder.h>
#if __has_include(<android/binder_manager.h>)
#include <android/binder_manager.h>
#else
// 某些 NDK 版本不装 binder_manager.h；符号本身在 libbinder_ndk.so 里（API 31+）
extern "C" AIBinder* AServiceManager_getService(const char* instance);
extern "C" AIBinder* AServiceManager_waitForService(const char* instance);
#endif
#include <android/binder_parcel.h>
#if __has_include(<android/binder_process.h>)
#include <android/binder_process.h>
#else
// 同上：个别 NDK 不装这个头，符号在 libbinder_ndk.so 里（API 29+）
extern "C" void ABinderProcess_setThreadPoolMaxThreadCount(uint32_t numThreads);
extern "C" void ABinderProcess_startThreadPool(void);
#endif
#include <android/binder_status.h>

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <dlfcn.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#ifndef TIOCSETD
#define TIOCSETD 0x541b
#endif
#ifndef N_HCI
#define N_HCI 15
#endif
#ifndef TIOCGPTN
#define TIOCGPTN 0x80045430
#endif
#ifndef TIOCSPTLCK
#define TIOCSPTLCK 0x40045431
#endif
#ifndef HCIUARTSETPROTO
#define HCIUARTSETPROTO 0x400455C8  // _IOW('U', 200, int)
#endif
#ifndef HCIUARTGETDEVICE
#define HCIUARTGETDEVICE 0x800455CA  // _IOR('U', 202, int)
#endif
#define HCI_UART_H4 0

static constexpr const char* kSvcHci = "android.hardware.bluetooth.IBluetoothHci/default";
static constexpr const char* kDescCallbacks = "android.hardware.bluetooth.IBluetoothHciCallbacks";

// IBluetoothHci 方法码 = FIRST_CALL_TRANSACTION(1) + 声明顺序
// 1 initialize 2 enable 3 disable 4 close 5 sendHciCommand 6 sendAclData 7 sendScoData
// IBluetoothHciCallbacks：1 transportReset 2 hciEvent 3 aclDataReceived 4 scoDataReceived
//                        5 latencyInformationChanged 6 flowStatus
enum : uint32_t {
  kInitialize = 1,
  kEnable = 2,
  kDisable = 3,
  kClose = 4,
  kSendCommand = 5,
  kSendAcl = 6,
  kSendSco = 7,
};

// HAL 约定：vec<uint8_t> 含 H4 类型字节（Fluoride 的实现如此）。若实测不对，--no-type-byte 翻一下。
static bool g_include_type = true;
static int g_mfd = -1;  // pty master
static int g_sfd = -1;  // pty slave（挂了 N_HCI）
static std::atomic<bool> g_run{true};
static std::mutex g_ptx;  // 串行化 pty 写（回调可能跑在多个 binder 线程上）
static AIBinder* g_hal = nullptr;

static void log(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "[bthci] ");
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
}

static void on_term(int) { g_run = false; }

// ---------------------------------------------------------------- 回调 binder
static void* cbCreate(void*) { return new std::atomic<uint64_t>{0}; }
static void cbDestroy(void* cookie) { delete static_cast<std::atomic<uint64_t>*>(cookie); }

static bool looksLikeH4Type(uint8_t b) { return b == 0x02 || b == 0x03 || b == 0x04; }

// NDK 的 AParcel_readByteArray(parcel, context, allocator)：由我们提供缓冲
struct HciBuf {
  int8_t data[2048];
  size_t n = 0;
};
static bool allocHci(void* ctx, int numElements, int8_t** out) {
  auto* b = static_cast<HciBuf*>(ctx);
  b->n = 0;
  if (numElements <= 0) {
    *out = nullptr;
    return true;
  }
  if (static_cast<size_t>(numElements) > sizeof(b->data)) return false;
  *out = b->data;
  b->n = static_cast<size_t>(numElements);
  return true;
}

// 把 HAL 给的 HCI 包写进 pty，交给内核蓝牙栈
static void toKernel(uint8_t h4type, const int8_t* data, size_t n) {
  if (n == 0 || g_mfd < 0) return;
  uint8_t frame[2048];
  size_t flen = 0;
  if (looksLikeH4Type(static_cast<uint8_t>(data[0]))) {
    flen = n < sizeof(frame) ? n : sizeof(frame);
    memcpy(frame, data, flen);
  } else {
    frame[0] = h4type;
    flen = (n + 1 < sizeof(frame)) ? n + 1 : sizeof(frame);
    memcpy(frame + 1, data, flen - 1);
  }
  std::lock_guard<std::mutex> lk(g_ptx);
  ssize_t w = write(g_mfd, frame, flen);
  if (w < 0) log("写 pty 失败: %s", strerror(errno));
}

// 手写服务端不会自动 enforceInterface，得自己跳过 token：
// 线格式 = int32(strict-mode) + int32(字符数) + UTF-16 + 0x0000，再补到 4 字节
static void skipToken(const AParcel* in) {
  int32_t v = 0;
  if (AParcel_readInt32(in, &v) != STATUS_OK) return;
  int32_t len = -1;
  if (AParcel_readInt32(in, &len) != STATUS_OK) return;
  if (len <= 0) return;
  size_t pos = 8 + static_cast<size_t>(len) * 2 + 2;
  pos = (pos + 3) & ~static_cast<size_t>(3);
  AParcel_setDataPosition(in, static_cast<int32_t>(pos));
}

static binder_status_t cbOnTransact(AIBinder*, uint32_t code, const AParcel* in, AParcel* out) {
  HciBuf hb;
  binder_status_t st = STATUS_OK;
  skipToken(in);
  if (code == 2 || code == 3 || code == 4) {
    binder_status_t r = AParcel_readByteArray(in, &hb, allocHci);
    if (r != STATUS_OK) st = r;
  }
  if (out) AParcel_writeInt32(out, 0);  // EX_NONE
  switch (code) {
    case 1:
      log("← transportReset");
      break;
    case 2:
      toKernel(0x04, hb.data, hb.n);  // hciEvent
      break;
    case 3:
      toKernel(0x02, hb.data, hb.n);  // aclDataReceived
      break;
    case 4:
      toKernel(0x03, hb.data, hb.n);  // scoDataReceived
      break;
    case 6:
      break;  // flowStatus：H4 无流控语义，忽略
    default:
      log("← 未处理回调 code=%u", code);
      break;
  }
  return st;
}

static const AIBinder_Class* g_cbClass = nullptr;

// ---------------------------------------------------------------- binder 侧
// 一次调用：prepareTransaction 让平台写 token/header，我们只 append 参数
static binder_status_t callVoid(uint32_t code) {
  AParcel* in = nullptr;
  binder_status_t st = AIBinder_prepareTransaction(g_hal, &in);
  if (st != STATUS_OK) return st;
  AParcel* out = nullptr;
  st = AIBinder_transact(g_hal, code, &in, &out, 0 /*sync*/);
  if (out) AParcel_delete(out);
  return st;
}

static binder_status_t callBytes(uint32_t code, const uint8_t* d, size_t n, bool oneway) {
  AParcel* in = nullptr;
  binder_status_t st = AIBinder_prepareTransaction(g_hal, &in);
  if (st != STATUS_OK) return st;
  st = AParcel_writeByteArray(in, reinterpret_cast<const int8_t*>(d), n);
  if (st != STATUS_OK) return st;
  AParcel* out = nullptr;
  st = AIBinder_transact(g_hal, code, &in, &out, oneway ? FLAG_ONEWAY : 0);
  if (out) AParcel_delete(out);
  return st;
}

// ---------------------------------------------------------------- 内核侧
static bool attachHci() {
  int mfd = open("/dev/ptmx", O_RDWR | O_NOCTTY);
  if (mfd < 0) {
    log("open /dev/ptmx 失败: %s", strerror(errno));
    return false;
  }
  int n = 0;
  if (ioctl(mfd, TIOCGPTN, &n) < 0) {
    log("TIOCGPTN 失败: %s", strerror(errno));
    close(mfd);
    return false;
  }
  char sp[64];
  snprintf(sp, sizeof(sp), "/dev/pts/%d", n);
  int lk = 0;
  ioctl(mfd, TIOCSPTLCK, &lk);
  int sfd = open(sp, O_RDWR | O_NOCTTY);
  if (sfd < 0) {
    log("open %s 失败: %s", sp, strerror(errno));
    close(mfd);
    return false;
  }
  g_mfd = mfd;
  g_sfd = sfd;
  int disc = N_HCI;
  if (ioctl(sfd, TIOCSETD, &disc) < 0) {
    log("TIOCSETD N_HCI 失败: %s（需要 root）", strerror(errno));
    return false;
  }
  int proto = HCI_UART_H4;
  ioctl(sfd, HCIUARTSETPROTO, proto);
  ioctl(sfd, HCIUARTSETPROTO, proto);  // 本机 legacy hci_uart 第二次才 register（实测）
  int idx = -1;
  ioctl(sfd, HCIUARTGETDEVICE, &idx);
  log("hci attach: pty=%s GETDEVICE=%d（判据看 /sys/class/bluetooth，不信这个返回值）", sp, idx);
  return true;
}

// pty master → HAL：按 H4 帧切包后逐包转发（内核一次 write 就是一整包，仍做缓冲以防被拆）
static void pumpToHal() {
  static std::vector<uint8_t> acc;
  uint8_t tmp[4096];
  ssize_t got = read(g_mfd, tmp, sizeof(tmp));
  if (got <= 0) return;
  acc.insert(acc.end(), tmp, tmp + got);
  size_t pos = 0;
  while (pos + 2 <= acc.size()) {
    uint8_t type = acc[pos];
    size_t need = 0, tlen = 0;
    if (type == 0x01 && pos + 4 <= acc.size()) {
      tlen = acc[pos + 3];
      need = 4 + tlen;
    } else if (type == 0x02 && pos + 5 <= acc.size()) {
      tlen = static_cast<size_t>(acc[pos + 3] | (acc[pos + 4] << 8));
      need = 5 + tlen;
    } else if (type == 0x03 && pos + 6 <= acc.size()) {
      tlen = acc[pos + 5];
      need = 6 + tlen;
    } else if (type > 0x03) {
      log("未知 H4 类型 0x%02x，丢 1 字节", type);
      ++pos;
      continue;
    } else {
      break;  // 头还没收全
    }
    if (pos + need > acc.size()) break;
    const uint8_t* pkt = acc.data() + pos;
    size_t flen = need;
    uint32_t code = type == 0x01 ? kSendCommand : type == 0x02 ? kSendAcl : kSendSco;
    binder_status_t st =
        callBytes(code, g_include_type ? pkt : pkt + 1, g_include_type ? flen : flen - 1, true);
    if (st != STATUS_OK) log("→ HAL code=%u st=%d", code, st);
    pos += need;
  }
  acc.erase(acc.begin(), acc.begin() + pos);
}

int main(int argc, char** argv) {
  int keep = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--keep") && i + 1 < argc)
      keep = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--no-type-byte"))
      g_include_type = false;
    else {
      fprintf(stderr, "用法: %s [--keep <秒>] [--no-type-byte]\n", argv[0]);
      return 2;
    }
  }
  signal(SIGINT, on_term);
  signal(SIGTERM, on_term);
  signal(SIGPIPE, SIG_IGN);

  // 1) binder 运行时：回调可能随时打进来，先起线程池
  g_cbClass = AIBinder_Class_define(kDescCallbacks, cbCreate, cbDestroy, cbOnTransact);
  if (!g_cbClass) {
    log("✗ AIBinder_Class_define 失败");
    return 1;
  }
  AIBinder* cb = AIBinder_new(g_cbClass, nullptr);
  if (!cb) {
    log("✗ AIBinder_new 失败");
    return 1;
  }
  // platform-only 符号：能 dlsym 到就强制 vendor-stable，拿不到就算了
  // （多数 HAL 只检查 binder 是否 stable，不检查分区归属）
  if (void* f = dlsym(RTLD_DEFAULT, "AIBinder_forceDowngradeToVendorStability"))
    reinterpret_cast<void (*)(AIBinder*)>(f)(cb);

  ABinderProcess_setThreadPoolMaxThreadCount(2);
  ABinderProcess_startThreadPool();

  g_hal = AServiceManager_getService(kSvcHci);
  if (!g_hal) {
    log("✗ 拿不到 %s —— HAL 活着吗？getprop init.svc.vendor.bluetooth-aidl-qti", kSvcHci);
    return 1;
  }
  log("✓ 拿到 %s", kSvcHci);

  // 2) 内核侧 hci0
  if (!attachHci()) return 1;

  // 3) 坐 HAL 的客户端位
  {
    AParcel* in = nullptr;
    binder_status_t st = AIBinder_prepareTransaction(g_hal, &in);
    if (st == STATUS_OK) st = AParcel_writeStrongBinder(in, cb);
    if (st == STATUS_OK) {
      AParcel* out = nullptr;
      st = AIBinder_transact(g_hal, kInitialize, &in, &out, 0);
      if (out) AParcel_delete(out);
    }
    log("initialize → %d", st);
    st = callVoid(kEnable);
    log("enable → %d", st);
  }

  // 4) 搬运主循环
  auto until = keep > 0 ? std::chrono::steady_clock::now() + std::chrono::seconds(keep)
                        : std::chrono::steady_clock::time_point::max();
  while (g_run && std::chrono::steady_clock::now() < until) {
    pollfd pf{g_mfd, POLLIN, 0};
    int s = poll(&pf, 1, 1000);
    if (s > 0 && (pf.revents & POLLIN)) pumpToHal();
    if (s < 0 && errno != EINTR) break;
  }

  // 5) 体面退场：disable+close 把 HAL 交还给安卓 framework
  log("退场：disable + close");
  callVoid(kDisable);
  callVoid(kClose);
  AIBinder_decStrong(cb);
  int back = N_TTY;
  ioctl(g_sfd, TIOCSETD, &back);
  close(g_sfd);
  close(g_mfd);
  return 0;
}
