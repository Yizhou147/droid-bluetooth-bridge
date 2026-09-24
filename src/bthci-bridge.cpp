// bthci-bridge.cpp — DRM 接管期的蓝牙 HCI 桥（vendor IBluetoothHci 的 binder 客户端）
//
// 一个进程干两件事（在安卓侧以 root 运行）：
//   1) 开一对 pty，在 slave 上挂 N_HCI + HCIUARTSETPROTO(H4) → 在共享内核里注册出真 hci0
//      （已实测：零新内核模块；进程退出 → tty 关闭 → hci_unregister_dev 自动回收）
//   2) 做 vendor 蓝牙 HAL 的 binder 客户端：initialize(我们的回调) → enable →
//      把 pty 上的 H4 帧与 HAL 双向搬运。上电/固件下载/glink 全部仍由 HAL 负责。
//
// 为什么用 dlopen 而不是 -lbinder_ndk：GitHub runner 上那份 NDK 没有 binder_manager.h /
// binder_process.h，其 libbinder_ndk.so stub 也不导出 ABinderProcess_*/AServiceManager_*
// （CI 实测 undefined symbol）。设备上的 /system/lib64/libbinder_ndk.so 是全的 →
// 运行时 dlopen + 自己声明原型，彻底不依赖 NDK 头与 stub，头文件漂移这一整类失败消失。
//
// 绝对不做：不 open /dev/bt_cp_ctrl、不 open /dev/ttyHS0、不打任何 btpower/geni 电源 ioctl、
// 不写 /sys/class/rfkill/rfkill0、不 setenforce 0。
//
// 编译：sh build.sh（NDK；本机 arm64 无 NDK → 由 .github/workflows/build.yml 云端构建）
// 运行：adb push bthci-bridge /data/local/tmp/
//       adb shell su -c '/data/local/tmp/bthci-bridge --keep 3600'

#include <cerrno>
#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <dlfcn.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

#ifndef TIOCSETD
#define TIOCSETD 0x541b
#endif
#ifndef N_HCI
#define N_HCI 15
#endif
#ifndef N_TTY
#define N_TTY 0
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

// ------------------------------------------------------------ libbinder_ndk 的最小声明
struct AIBinder;
struct AParcel;
struct AIBinder_Class;
typedef int32_t binder_status_t;
typedef uint32_t transaction_code_t;
typedef void* (*fn_onCreate)(void*);
typedef void (*fn_onDestroy)(void*);
typedef binder_status_t (*fn_onTransact)(AIBinder*, transaction_code_t, const AParcel*, AParcel*);
typedef bool (*fn_byteArrayAllocator)(void* ctx, int numElements, int8_t** out);

static void log(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

static constexpr uint32_t FLAG_ONEWAY = 1;
static constexpr binder_status_t ST_OK = 0;

static struct {
  const AIBinder_Class* (*Class_define)(const char*, fn_onCreate, fn_onDestroy, fn_onTransact);
  AIBinder* (*New)(const AIBinder_Class*, void*);
  void (*IncStrong)(AIBinder*);
  void (*DecStrong)(AIBinder*);
  binder_status_t (*Prepare)(AIBinder*, AParcel**);
  binder_status_t (*Transact)(AIBinder*, transaction_code_t, AParcel**, AParcel**, uint32_t);
  AIBinder* (*SM_getService)(const char*);
  AIBinder* (*SM_waitForService)(const char*);
  binder_status_t (*Parcel_writeInt32)(AParcel*, int32_t);
  binder_status_t (*Parcel_readInt32)(const AParcel*, int32_t*);
  binder_status_t (*Parcel_writeByteArray)(AParcel*, const int8_t*, size_t);
  binder_status_t (*Parcel_readByteArray)(const AParcel*, void*, fn_byteArrayAllocator);
  binder_status_t (*Parcel_writeStrongBinder)(AParcel*, AIBinder*);
  void (*Parcel_setDataPosition)(const AParcel*, int32_t);
  void (*Parcel_delete)(AParcel*);
  void (*Proc_setThreadPoolMaxThreadCount)(uint32_t);
  void (*Proc_startThreadPool)(void);
  void (*ForceDowngradeToVendorStability)(AIBinder*);  // platform-only，可能没有
} ndk{};

static bool loadNdk() {
  void* h = dlopen("libbinder_ndk.so", RTLD_NOW | RTLD_GLOBAL);
  if (!h) {
    log("dlopen libbinder_ndk.so 失败: %s", dlerror());
    return false;
  }
  auto get = [&](const char* n) { return dlsym(h, n); };
  ndk.Class_define = (decltype(ndk.Class_define))get("AIBinder_Class_define");
  ndk.New = (decltype(ndk.New))get("AIBinder_new");
  ndk.IncStrong = (decltype(ndk.IncStrong))get("AIBinder_incStrong");
  ndk.DecStrong = (decltype(ndk.DecStrong))get("AIBinder_decStrong");
  ndk.Prepare = (decltype(ndk.Prepare))get("AIBinder_prepareTransaction");
  ndk.Transact = (decltype(ndk.Transact))get("AIBinder_transact");
  ndk.SM_getService = (decltype(ndk.SM_getService))get("AServiceManager_getService");
  ndk.SM_waitForService = (decltype(ndk.SM_waitForService))get("AServiceManager_waitForService");
  ndk.Parcel_writeInt32 = (decltype(ndk.Parcel_writeInt32))get("AParcel_writeInt32");
  ndk.Parcel_readInt32 = (decltype(ndk.Parcel_readInt32))get("AParcel_readInt32");
  ndk.Parcel_writeByteArray = (decltype(ndk.Parcel_writeByteArray))get("AParcel_writeByteArray");
  ndk.Parcel_readByteArray = (decltype(ndk.Parcel_readByteArray))get("AParcel_readByteArray");
  ndk.Parcel_writeStrongBinder = (decltype(ndk.Parcel_writeStrongBinder))get("AParcel_writeStrongBinder");
  ndk.Parcel_setDataPosition = (decltype(ndk.Parcel_setDataPosition))get("AParcel_setDataPosition");
  ndk.Parcel_delete = (decltype(ndk.Parcel_delete))get("AParcel_delete");
  ndk.Proc_setThreadPoolMaxThreadCount =
      (decltype(ndk.Proc_setThreadPoolMaxThreadCount))get("ABinderProcess_setThreadPoolMaxThreadCount");
  ndk.Proc_startThreadPool = (decltype(ndk.Proc_startThreadPool))get("ABinderProcess_startThreadPool");
  ndk.ForceDowngradeToVendorStability =
      (decltype(ndk.ForceDowngradeToVendorStability))get("AIBinder_forceDowngradeToVendorStability");

  const char* missing = nullptr;
  if (!ndk.Class_define) missing = "AIBinder_Class_define";
  else if (!ndk.New) missing = "AIBinder_new";
  else if (!ndk.Prepare) missing = "AIBinder_prepareTransaction";
  else if (!ndk.Transact) missing = "AIBinder_transact";
  else if (!ndk.SM_getService) missing = "AServiceManager_getService";
  else if (!ndk.Parcel_writeByteArray) missing = "AParcel_writeByteArray";
  else if (!ndk.Parcel_readByteArray) missing = "AParcel_readByteArray";
  else if (!ndk.Parcel_writeStrongBinder) missing = "AParcel_writeStrongBinder";
  else if (!ndk.Proc_startThreadPool) missing = "ABinderProcess_startThreadPool";
  if (missing) {
    log("✗ libbinder_ndk 里缺 %s", missing);
    return false;
  }
  return true;
}

// ------------------------------------------------------------ 本程序状态
static const char* kSvcHci = "android.hardware.bluetooth.IBluetoothHci/default";
static const char* kDescCallbacks = "android.hardware.bluetooth.IBluetoothHciCallbacks";
static const char* kDescHci = "android.hardware.bluetooth.IBluetoothHci";

// AIBinder_Class_define 的副作用就是把 descriptor 在本进程"声明"（Stability::declare）。
// 不声明的话 AIBinder_prepareTransaction 直接返回 -38(INVALID_OPERATION)：
// libbinder 的 Stability::checkDeclared(descriptor) 不过。AIDL 生成的桩本来会替你做这件事，
// 我们手写就得自己补一次——给目标接口 define 个永不服务的哑类即可（实测 prepare=-38 定位）。
static void* noopCreate(void*) { return new int(0); }
static void noopDestroy(void* c) { delete static_cast<int*>(c); }
static binder_status_t noopTransact(AIBinder*, transaction_code_t, const AParcel*, AParcel*) {
  return ST_OK;
}

// IBluetoothHci 方法码 = 声明顺序（1 initialize 2 enable 3 disable 4 close
// 5 sendHciCommand 6 sendAclData 7 sendScoData）
enum : uint32_t { kInitialize = 1, kEnable = 2, kDisable = 3, kClose = 4, kSendCommand = 5, kSendAcl = 6, kSendSco = 7 };
// 回调方法码（1 transportReset 2 hciEvent 3 aclDataReceived 4 scoDataReceived 6 flowStatus）
enum : uint32_t { kCbTransportReset = 1, kCbHciEvent = 2, kCbAcl = 3, kCbSco = 4, kCbFlowStatus = 6 };

// HAL 约定：vec<uint8_t> 含 H4 类型字节（Fluoride 的实现如此）。不对就 --no-type-byte 翻一下。
static bool g_include_type = true;
static int g_mfd = -1;  // pty master
static int g_sfd = -1;  // pty slave（挂了 N_HCI）
static volatile sig_atomic_t g_run = 1;
static std::mutex g_ptx;  // 回调可能在多个 binder 线程上 → 串行化 pty 写
static AIBinder* g_hal = nullptr;

static void log(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  fprintf(stderr, "[bthci] ");
  vfprintf(stderr, fmt, ap);
  va_end(ap);
  fprintf(stderr, "\n");
  fflush(stderr);
}

static void on_term(int) { g_run = 0; }

// ------------------------------------------------------------ 内核侧
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

// pty master → HAL：按 H4 切帧后逐包转发
static void pumpToHal() {
  static std::vector<uint8_t> acc;
  uint8_t tmp[4096];
  ssize_t got = read(g_mfd, tmp, sizeof(tmp));
  if (got <= 0) return;
  acc.insert(acc.end(), tmp, tmp + got);
  size_t pos = 0;
  while (pos + 2 <= acc.size()) {
    uint8_t type = acc[pos];
    size_t need = 0;
    if (type == 0x01 && pos + 4 <= acc.size())
      need = 4 + acc[pos + 3];
    else if (type == 0x02 && pos + 5 <= acc.size())
      need = 5 + static_cast<size_t>(acc[pos + 3] | (acc[pos + 4] << 8));
    else if (type == 0x03 && pos + 6 <= acc.size())
      need = 6 + acc[pos + 5];
    else if (type > 0x03) {
      log("未知 H4 类型 0x%02x，丢 1 字节", type);
      ++pos;
      continue;
    } else {
      break;
    }
    if (pos + need > acc.size()) break;
    const uint8_t* pkt = acc.data() + pos;
    size_t flen = need;
    uint32_t code = type == 0x01 ? kSendCommand : type == 0x02 ? kSendAcl : kSendSco;
    AParcel* in = nullptr;
    if (ndk.Prepare(g_hal, &in) == ST_OK) {
      const uint8_t* payload = g_include_type ? pkt : pkt + 1;
      size_t n = g_include_type ? flen : flen - 1;
      binder_status_t st = ndk.Parcel_writeByteArray(in, reinterpret_cast<const int8_t*>(payload), n);
      if (st == ST_OK) {
        AParcel* out = nullptr;
        st = ndk.Transact(g_hal, code, &in, &out, FLAG_ONEWAY);
        if (out) ndk.Parcel_delete(out);
      }
      if (st != ST_OK) log("→ HAL code=%u st=%d", code, st);
    }
    pos += need;
  }
  acc.erase(acc.begin(), acc.begin() + pos);
}

// ------------------------------------------------------------ HAL → 内核
static bool looksLikeH4Type(uint8_t b) { return b == 0x02 || b == 0x03 || b == 0x04; }

struct HciBuf {
  int8_t data[2048];
  size_t n = 0;
};

static bool allocHci(void* ctx, int numElements, int8_t** out) {
  auto* b = static_cast<HciBuf*>(ctx);
  b->n = 0;
  *out = nullptr;
  if (numElements <= 0) return true;
  if (static_cast<size_t>(numElements) > sizeof(b->data)) return false;
  *out = b->data;
  b->n = static_cast<size_t>(numElements);
  return true;
}

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
  if (write(g_mfd, frame, flen) < 0) log("写 pty 失败: %s", strerror(errno));
}

// 手写服务端不会自动 enforceInterface，得自己跳过 token：
// 线格式 = int32(strict-mode) + int32(字符数) + UTF-16 + 0x0000，再补到 4 字节
static void skipToken(const AParcel* in) {
  if (!ndk.Parcel_readInt32 || !ndk.Parcel_setDataPosition) return;
  int32_t v = 0;
  if (ndk.Parcel_readInt32(in, &v) != ST_OK) return;
  int32_t len = -1;
  if (ndk.Parcel_readInt32(in, &len) != ST_OK) return;
  if (len <= 0) return;
  size_t pos = 8 + static_cast<size_t>(len) * 2 + 2;
  pos = (pos + 3) & ~static_cast<size_t>(3);
  ndk.Parcel_setDataPosition(in, static_cast<int32_t>(pos));
}

static void* cbCreate(void*) { return new int(0); }
static void cbDestroy(void* cookie) { delete static_cast<int*>(cookie); }

static binder_status_t cbOnTransact(AIBinder*, transaction_code_t code, const AParcel* in, AParcel* out) {
  HciBuf hb;
  skipToken(in);
  if (code == kCbHciEvent || code == kCbAcl || code == kCbSco) {
    binder_status_t r = ndk.Parcel_readByteArray(in, &hb, allocHci);
    if (r != ST_OK) log("← 读 byte[] 失败 code=%u st=%d", code, r);
  }
  if (out && ndk.Parcel_writeInt32) ndk.Parcel_writeInt32(out, 0);  // EX_NONE
  switch (code) {
    case kCbTransportReset:
      log("← transportReset");
      break;
    case kCbHciEvent:
      toKernel(0x04, hb.data, hb.n);
      break;
    case kCbAcl:
      toKernel(0x02, hb.data, hb.n);
      break;
    case kCbSco:
      toKernel(0x03, hb.data, hb.n);
      break;
    case kCbFlowStatus:
      break;  // H4 无流控语义
    default:
      log("← 未处理回调 code=%u", code);
      break;
  }
  return ST_OK;
}

static binder_status_t callVoid(uint32_t code, bool oneway) {
  AParcel* in = nullptr;
  binder_status_t st = ndk.Prepare(g_hal, &in);
  if (st != ST_OK) return st;
  AParcel* out = nullptr;
  st = ndk.Transact(g_hal, code, &in, &out, oneway ? FLAG_ONEWAY : 0);
  if (out) ndk.Parcel_delete(out);
  return st;
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

  if (!loadNdk()) return 1;

  // 先声明目标接口（否则 prepareTransaction 直接 -38）
  const AIBinder_Class* hciCls = ndk.Class_define(kDescHci, noopCreate, noopDestroy, noopTransact);
  if (!hciCls) {
    log("✗ 无法声明 %s（后续 prepareTransaction 会一直 -38）", kDescHci);
    return 1;
  }
  log("✓ 已在本进程声明接口 %s", kDescHci);

  const AIBinder_Class* cls = ndk.Class_define(kDescCallbacks, cbCreate, cbDestroy, cbOnTransact);
  if (!cls) {
    log("✗ AIBinder_Class_define 失败");
    return 1;
  }
  AIBinder* cb = ndk.New(cls, nullptr);
  if (!cb) {
    log("✗ AIBinder_new 失败");
    return 1;
  }
  // vendor HAL 可能要求回调是 vendor-stable；有这个 platform 符号就用
  if (ndk.ForceDowngradeToVendorStability) ndk.ForceDowngradeToVendorStability(cb);

  ndk.Proc_setThreadPoolMaxThreadCount(2);
  ndk.Proc_startThreadPool();

  g_hal = ndk.SM_getService(kSvcHci);
  if (!g_hal && ndk.SM_waitForService) g_hal = ndk.SM_waitForService(kSvcHci);
  if (!g_hal) {
    log("✗ 拿不到 %s —— HAL 活着吗？getprop init.svc.vendor.bluetooth-aidl-qti", kSvcHci);
    return 1;
  }
  log("✓ 拿到 %s", kSvcHci);

  if (!attachHci()) return 1;

  // 坐 HAL 的客户端位。
  // -38 = libbinder 的 INVALID_OPERATION，来自 Stability::checkDeclared()：
  // 从 system 进程拿 vendor 侧 AIDL HAL，必须先对句柄做 forceDowngrade，
  // 否则 prepareTransaction 直接拒。三种降级依次试，谁通用谁。
  {
    static const char* kDg[] = {"", "vendor", "system", "local"};
    int okIdx = -1;
    for (int i = 0; i < 4 && okIdx < 0; i++) {
      if (i > 0) {
        // 重新取句柄，避免上一次的 stability 标记残留
        g_hal = ndk.SM_getService(kSvcHci);
        if (!g_hal) { log("✗ 第%d次取句柄失败", i); break; }
        void* sym = dlsym(RTLD_DEFAULT, (std::string("AIBinder_forceDowngradeTo") + kDg[i] + "Stability").c_str());
        if (!sym) { log("  · 无 %s 符号，跳过", kDg[i]); continue; }
        reinterpret_cast<void (*)(AIBinder*)>(sym)(g_hal);
        log("  · 已对 HAL 句柄施加 %s-stability", kDg[i]);
      }
      AParcel* in = nullptr;
      binder_status_t s1 = ndk.Prepare(g_hal, &in);
      binder_status_t s2 = (s1 == ST_OK) ? ndk.Parcel_writeStrongBinder(in, cb) : s1;
      binder_status_t s3 = (s2 == ST_OK) ? [&] {
        AParcel* out = nullptr;
        binder_status_t r = ndk.Transact(g_hal, kInitialize, &in, &out, 0);
        if (out) ndk.Parcel_delete(out);
        return r;
      }() : s2;
      log("initialize[%s]: prepare=%d writeBinder=%d transact=%d", kDg[i], s1, s2, s3);
      if (s3 == ST_OK) okIdx = i;
    }
    if (okIdx < 0) {
      log("✗ initialize 全部失败（-38=stability 未过 / -ENOSYS=事务码不对）");
    } else {
      log("✓ initialize 通过（stability=%s）", kDg[okIdx]);
      log("enable → %d", callVoid(kEnable, false));
    }
  }

  auto until = keep > 0 ? std::chrono::steady_clock::now() + std::chrono::seconds(keep)
                        : std::chrono::steady_clock::time_point::max();
  while (g_run && std::chrono::steady_clock::now() < until) {
    struct pollfd pf{g_mfd, POLLIN, 0};
    int s = poll(&pf, 1, 1000);
    if (s > 0 && (pf.revents & POLLIN)) pumpToHal();
    if (s < 0 && errno != EINTR) break;
  }

  log("退场：disable + close（把 HAL 交还给安卓 framework）");
  callVoid(kDisable, false);
  callVoid(kClose, true);
  if (ndk.DecStrong) ndk.DecStrong(cb);
  int back = N_TTY;
  ioctl(g_sfd, TIOCSETD, &back);
  close(g_sfd);
  close(g_mfd);
  return 0;
}
