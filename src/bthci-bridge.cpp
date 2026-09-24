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
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <thread>

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

// 自己触发内核打开 hci0（等价 hciconfig hci0 up）：enable 之后 HAL 做 IBS DeviceWakeUp，
// 1~2s 等不到第一帧 HCI 就超时并被 SIGKILL（实测 pid 2198→17985），所以必须立刻让内核开口。
#ifndef AF_BLUETOOTH
#define AF_BLUETOOTH 31
#endif
#ifndef BTPROTO_HCI
#define BTPROTO_HCI 1
#endif
#define HCIDEVUP_E 0x400448C9u    // _IOW('H', 201, int)
#define HCIDEVDOWN_E 0x400448CAu  // _IOW('H', 202, int)

// 让内核打开 hci0（它会立刻发 HCI_Reset）。用容器里的 hciconfig：
// 实测"未绑定控制 socket + HCIDEVUP"会 ENODEV，而 nsenter 进容器 ns 跑 hciconfig 成功过。
// hciconfig 会阻塞等 init-complete（超时也照常返回非 0，但命令已发出），所以放后台。
static int kickHci(int up) {
  char cmd[256];
  if (up)
    snprintf(cmd, sizeof(cmd),
             "P=$(ps -A -o PID,NAME | grep -w bluetoothd | head -1 | cut -d' ' -f1); "
             "[ -n \"$P\" ] || exit 3; nsenter -t $P -m -p -- /usr/bin/hciconfig hci0 up "
             ">/dev/null 2>&1 & exit 0");
  else
    snprintf(cmd, sizeof(cmd),
             "P=$(ps -A -o PID,NAME | grep -w bluetoothd | head -1 | cut -d' ' -f1); "
             "[ -n \"$P\" ] || exit 3; nsenter -t $P -m -p -- /usr/bin/hciconfig hci0 down "
             ">/dev/null 2>&1");
  int rc = system(cmd);
  return rc;
}

// 自动扫出来的 sendHciCommand / sendAclData 码位（初值是猜的）
static uint32_t g_cmdCode = 5, g_aclCode = 6;

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
  void* handle;
  void (*AssociateClass)(AIBinder*, const AIBinder_Class*);
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
  bool (*Binder_isRemote)(AIBinder*);
  bool (*Binder_isNative)(AIBinder*);
  int32_t (*Binder_getVendor)(AIBinder*);
  void (*ForceDowngradeToVendorStability)(AIBinder*);  // platform 声明，但设备 .so 里有符号
  void (*ForceDowngradeToSystemStability)(AIBinder*);
} ndk{};

static bool loadNdk() {
  void* h = dlopen("libbinder_ndk.so", RTLD_NOW | RTLD_GLOBAL);
  if (!h) {
    log("dlopen libbinder_ndk.so 失败: %s", dlerror());
    return false;
  }
  auto get = [&](const char* n) { return dlsym(h, n); };
  ndk.handle = h;
  ndk.AssociateClass = (decltype(ndk.AssociateClass))get("AIBinder_associateClass");
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
  ndk.Binder_isRemote = (decltype(ndk.Binder_isRemote))get("AIBinder_isRemote");
  ndk.Binder_isNative = (decltype(ndk.Binder_isNative))get("AIBinder_isNative");
  ndk.Binder_getVendor = (decltype(ndk.Binder_getVendor))get("AIBinder_getVendor");
  ndk.Proc_setThreadPoolMaxThreadCount =
      (decltype(ndk.Proc_setThreadPoolMaxThreadCount))get("ABinderProcess_setThreadPoolMaxThreadCount");
  ndk.Proc_startThreadPool = (decltype(ndk.Proc_startThreadPool))get("ABinderProcess_startThreadPool");
  ndk.ForceDowngradeToVendorStability =
      (decltype(ndk.ForceDowngradeToVendorStability))get("AIBinder_forceDowngradeToVendorStability");
  ndk.ForceDowngradeToSystemStability =
      (decltype(ndk.ForceDowngradeToSystemStability))get("AIBinder_forceDowngradeToSystemStability");

  const char* missing = nullptr;
  if (!ndk.Class_define) missing = "AIBinder_Class_define";
  else if (!ndk.AssociateClass) missing = "AIBinder_associateClass";
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
// 探测用的候选服务名（真机可能注册在 HIDL 兼容名或 QTI 专有名下）
static const char* kSvcVariants[] = {
    "android.hardware.bluetooth.IBluetoothHci/default",
    "android.hardware.bluetooth@1.0::IBluetoothHci/default",
    "android.hardware.bluetooth@aidl-service-qti",
    "vendor.qti.hardware.bluetooth.IBluetoothHci/default",
};
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

// IBluetoothHci 方法码 —— **本机实测得出**，与 AOSP 早期顺序不同：
//   发 code 2 的那一刻 HAL 打了 BluetoothHci::close() → 2=close
//   无参调 2 还报 BAD_TYPE（close 不收参数，多余的 int 被 enforceNoDataAvail 拒）
// 故本机的声明顺序是：initialize, close, enable(EnableReason), disable(DisableReason),
//                      sendHciCommand, sendAclData, sendScoData
enum : uint32_t {
  // 蓝牙关闭的干净状态下 --sweep 实测：code=4 + int32 让 HAL 打开 bt_cp_ctrl/ttyHS
  // （fd 0→2）⇒ enable = 4。code 2/3 全无副作用，说明本机 AIDL 顺序不是 AOSP 老的那套；
  // disable / sendHciCommand / sendAclData 的真实码位待下一轮用
  // 「发出去后 IBS 超时是否停止、有没有 hciEvent 回来」当判据再扫。
  kInitialize = 1,
  kEnable = 4,
};
enum : uint32_t { kMaybeClose = 2, kMaybe3 = 3, kMaybeDisable = 5, kMaybeSendCmd = 6,
                  kMaybeSendAcl = 7, kMaybeSendSco = 8 };
// 过渡别名（这几位还是猜的，下一轮用副作用判据实定）：
enum : uint32_t { kClose = kMaybeClose, kDisable = kMaybeDisable, kSendCommand = kMaybeSendCmd,
                  kSendAcl = kMaybeSendAcl, kSendSco = kMaybeSendSco };
// EnableReason / DisableReason 枚举值（AIDL）：0=OTHER/UNKNOWN，1/2/3 为其它原因；
// 先按 0 发，不行再挨个试。
// 实测 code=3 + int32 才会让 HAL 打开 bt_cp_ctrl/ttyHS；reason 具体取值待观察，
// 按 1,0,2,3 顺序试（1 已被实测证明有效）
static const int32_t kEnableReasons[] = {1, 0, 2, 3};
// 回调方法码（1 transportReset 2 hciEvent 3 aclDataReceived 4 scoDataReceived 6 flowStatus）
enum : uint32_t { kCbTransportReset = 1, kCbHciEvent = 2, kCbAcl = 3, kCbSco = 4, kCbFlowStatus = 6 };

// HAL 约定：vec<uint8_t> 含 H4 类型字节（Fluoride 的实现如此）。不对就 --no-type-byte 翻一下。
static bool g_include_type = true;
static int g_mfd = -1;  // pty master
static int g_sfd = -1;  // pty slave（挂了 N_HCI）
static volatile sig_atomic_t g_run = 1;
static std::mutex g_ptx;  // 回调可能在多个 binder 线程上 → 串行化 pty 写
static AIBinder* g_hal = nullptr;
static volatile unsigned long long g_toHal = 0, g_toKernel = 0;

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
    ++g_toHal;
    uint32_t code = type == 0x01 ? g_cmdCode : type == 0x02 ? g_aclCode : kSendSco;
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
  ++g_toKernel;
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

static int halPid() {
  FILE* f = popen("pidof android.hardware.bluetooth@aidl-service-qti 2>/dev/null | awk '{print $1}'", "r");
  if (!f) return -1;
  int pid = -1;
  if (fscanf(f, "%d", &pid) != 1) pid = -2;
  pclose(f);
  return pid;
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
  bool probe4 = false, probe5 = false, probeEnable = false, sweep = false, autotune = false;
  int mapCode = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--probe4")) {
      probe4 = true;
      continue;
    }
    if (!strcmp(argv[i], "--probe5")) {
      probe5 = true;
      continue;
    }
    if (!strcmp(argv[i], "--map") && i + 1 < argc) {
      mapCode = atoi(argv[++i]);
      continue;
    }
    if (!strcmp(argv[i], "--auto")) {
      autotune = true;
      continue;
    }
    if (!strcmp(argv[i], "--sweep")) {
      sweep = true;
      continue;
    }
    if (!strcmp(argv[i], "--probe-enable")) {
      probeEnable = true;
      continue;
    }
    if (!strcmp(argv[i], "--keep") && i + 1 < argc)
      keep = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--no-type-byte"))
      g_include_type = false;
    else {
      fprintf(stderr, "用法: %s [--keep <秒>] [--no-type-byte] [--probe4]\n", argv[0]);
      return 2;
    }
  }
  signal(SIGINT, on_term);
  signal(SIGTERM, on_term);
  signal(SIGPIPE, SIG_IGN);

  if (!loadNdk()) return 1;

  // 给目标接口 define 一个类：既完成 Stability 声明，也当"parcel 工厂"用
  // （见下面 Path A 的注释）
  const AIBinder_Class* hciCls = ndk.Class_define(kDescHci, noopCreate, noopDestroy, noopTransact);
  if (!hciCls) {
    log("✗ 无法声明 %s", kDescHci);
    return 1;
  }
  AIBinder* hciLocal = ndk.New(hciCls, nullptr);  // descriptor 与 vendor 句柄完全一致的本地 binder
  log("✓ 已在本进程声明接口 %s（本地替身 %p）", kDescHci, (void*)hciLocal);

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
  // 关键一步：AIBinder_prepareTransaction 的 interface token 是从 binder 绑定的 class
  // (+0x48) 取的，而 AServiceManager_getService 返回的是没 class 的裸句柄 → 一律 -38。
  // 用 AIBinder_associateClass 把我们 define 的 hciCls 挂上去。
  if (ndk.AssociateClass && hciCls) {
    ndk.AssociateClass(g_hal, hciCls);
    log("✓ 已 associateClass（descriptor=%s）", kDescHci);
  }
  // 注意：forceDowngrade* 只能作用于**本地** binder（对远端调用会
  // LOG_ALWAYS_FATAL("Can only downgrade local binder")，实测把进程直接打死）。
  // 远端句柄的 stability 由 associateClass + 对端声明决定，这里不做任何降级。

  if (!attachHci()) return 1;

  // ── 坐 HAL 的客户端位 ────────────────────────────────────────────────
  // 实测：prepareTransaction(g_hal) 直接 -38(INVALID_OPERATION) —— NDK 这层会拦
  // "从 servicemanager 拿来的 vendor 稳定 binder"（C++ 的 service call 不受此限，
  // 它走 BpBinder::transact）。绕法 Path A：用同 descriptor 的**本地** binder 当
  // parcel 工厂（Prepare 会写进正确的 interface token），再把 parcel 发给 vendor 句柄。
  // withInt>=0 时额外写一个 int32（新版 AIDL 的 enable(EnableReason)/disable(DisableReason)）
  auto callWith = [&](uint32_t code, bool withCb, uint32_t flags, int32_t* firstReply,
                      int withInt = -1) -> binder_status_t {
    AParcel* in = nullptr;
    binder_status_t st = ndk.Prepare(g_hal, &in);
    if (st == ST_OK && withCb) st = ndk.Parcel_writeStrongBinder(in, cb);
    if (st == ST_OK && withInt >= 0 && ndk.Parcel_writeInt32)
      st = ndk.Parcel_writeInt32(in, withInt);
    AParcel* out = nullptr;
    if (st == ST_OK) {
      st = ndk.Transact(g_hal, code, &in, &out, flags);
      if (out) {
        if (firstReply && ndk.Parcel_readInt32 && ndk.Parcel_setDataPosition) {
          ndk.Parcel_setDataPosition(out, 0);
          ndk.Parcel_readInt32(out, firstReply);
        }
        ndk.Parcel_delete(out);
      }
    }
    return st;
  };

  // 只读判据：HAL 有没有把 glink/UART 打开（=它真的开始上电了）
  auto halOpenTransport = []() -> int {
    char cmd[160];
    snprintf(cmd, sizeof(cmd), "ls -l /proc/%d/fd 2>/dev/null | grep -cE 'bt_cp_ctrl|ttyHS'", halPid());
    FILE* f = popen(cmd, "r");
    if (!f) return -1;
    int n = -1;
    if (fscanf(f, "%d", &n) != 1) n = -2;
    pclose(f);
    return n;
  };
  auto rfkillSoft = []() -> int {
    FILE* f = fopen("/sys/class/rfkill/rfkill0/soft", "r");
    if (!f) return -1;
    int v = -2;
    if (fscanf(f, "%d", &v) != 1) v = -3;
    fclose(f);
    return v;
  };

  auto halTransportFds = []() -> int {
    char cmd[160];
    snprintf(cmd, sizeof(cmd), "ls -l /proc/%d/fd 2>/dev/null | grep -cE 'bt_cp_ctrl|ttyHS'", halPid());
    FILE* g = popen(cmd, "r");
    int fds = -1;
    if (g) { if (fscanf(g, "%d", &fds) != 1) fds = -2; pclose(g); }
    return fds;
  };
  auto rfSoft = []() -> int {
    FILE* f = fopen("/sys/class/rfkill/rfkill0/soft", "r");
    if (!f) return -1;
    int v = -2;
    if (fscanf(f, "%d", &v) != 1) v = -3;
    fclose(f);
    return v;
  };

  // 一次完整的占位尝试（HAL 被杀后要重来）
  auto acquire = [&]() -> bool {
    g_hal = ndk.SM_getService(kSvcHci);
    if (!g_hal) { log("acquire: getService 失败"); return false; }
    if (ndk.AssociateClass) ndk.AssociateClass(g_hal, hciCls);
    int32_t f = 0x7abc;
    binder_status_t st = callWith(kInitialize, true, 0, &f);
    log("acquire: initialize=%d 回包首int32=%d", st, f);
    return st == ST_OK;
  };

  int initialized = 0;
  if (autotune) {
    static const uint32_t cands[] = {5, 6, 7, 3, 8, 2};
    for (uint32_t c : cands) {
      g_cmdCode = c;
      g_aclCode = c + 1;
      g_toKernel = 0;
      g_toHal = 0;
      kickHci(0);  // 先确保是 down 的，open 时内核才会重新发 HCI_Reset
      if (!acquire()) { log("AUTO cmdCode=%u 占位失败，换下一个", c); continue; }
      binder_status_t est = callWith(kEnable, false, FLAG_ONEWAY, nullptr, 1);
      int kr = kickHci(1);
      int got = 0;
      for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        struct pollfd pf{g_mfd, POLLIN, 0};
        if (poll(&pf, 1, 50) > 0 && (pf.revents & POLLIN)) pumpToHal();
        if (g_toKernel > 0) { got = 1; break; }
      }
      log("AUTO cmdCode=%u enable=%d kick=%d → 转发=%llu 收回=%llu %s", c, est, kr,
          (unsigned long long)g_toHal, (unsigned long long)g_toKernel,
          got ? "★★★ HAL 回应了：cmdCode 就是它" : "无回应");
      if (got) { initialized = 1; break; }
      kickHci(0);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!initialized) { log("✗ AUTO 没找到会让 HAL 回应的 cmdCode"); return 1; }
    log("✓ 定板 sendHciCommand=%u sendAclData=%u，继续搬运 %d 秒供观察", g_cmdCode, g_aclCode, keep);
    auto until2 = std::chrono::steady_clock::now() + std::chrono::seconds(keep > 0 ? keep : 30);
    while (g_run && std::chrono::steady_clock::now() < until2) {
      struct pollfd pf{g_mfd, POLLIN, 0};
      int s2 = poll(&pf, 1, 500);
      if (s2 > 0 && (pf.revents & POLLIN)) pumpToHal();
    }
    {
      char cmd[160];
      snprintf(cmd, sizeof(cmd),
               "nsenter -t $(ps -A -o PID,NAME | grep -w bluetoothd | head -1 | cut -d' ' -f1) -m -p "
               "-- /usr/bin/hciconfig -a 2>/dev/null | head -4");
      FILE* g = popen(cmd, "r");
      if (g) {
        char ln[256];
        while (fgets(ln, sizeof(ln), g)) fprintf(stderr, "[bthci] hciconfig| %s", ln);
        pclose(g);
      }
    }
    return 0;
  }
  if (sweep) {
    // 让服务器自己报出每个事务码期望的参数形状：无参 / int32 / byte[] / binder
    // 判据 = HAL 日志里那句 status（NOT_ENOUGH_DATA / BAD_TYPE / ReadAndValidateArraySize…）
    //        加上"HAL 是否重新打开了 bt_cp_ctrl/ttyHS"（=真的执行了 enable）
    int32_t first = 0x7abc;
    log("SWEEP initialize=%d", callWith(kInitialize, true, 0, &first));
    auto stamp = []() {
      timespec ts{};
      clock_gettime(CLOCK_REALTIME, &ts);
      tm lt{};
      localtime_r(&ts.tv_sec, &lt);
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", lt.tm_hour, lt.tm_min, lt.tm_sec,
               (int)(ts.tv_nsec / 1000000));
      return std::string(buf);
    };
    for (uint32_t code = 2; code <= 8; code++) {
      for (int shape = 0; shape < 4; shape++) {
        AParcel* in = nullptr;
        binder_status_t st = ndk.Prepare(g_hal, &in);
        if (st == ST_OK) {
          if (shape == 1) st = ndk.Parcel_writeInt32(in, 1);
          else if (shape == 2) {
            const int8_t one[1] = {0};
            st = ndk.Parcel_writeByteArray(in, one, 1);
          } else if (shape == 3) st = ndk.Parcel_writeStrongBinder(in, cb);
        }
        AParcel* out = nullptr;
        if (st == ST_OK) st = ndk.Transact(g_hal, code, &in, &out, FLAG_ONEWAY);
        if (out) ndk.Parcel_delete(out);
        static const char* sn[] = {"none", "int32", "byte[1]", "binder"};
        log("SWEEP t=%s code=%u shape=%-7s 投递=%d", stamp().c_str(), code, sn[shape], st);
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        if (shape == 1 || shape == 2) {
          char cmd[160];
          snprintf(cmd, sizeof(cmd), "ls -l /proc/%d/fd 2>/dev/null | grep -cE 'bt_cp_ctrl|ttyHS'", halPid());
          FILE* g = popen(cmd, "r");
          int fds = -1;
          if (g) { if (fscanf(g, "%d", &fds) != 1) fds = -2; pclose(g); }
          if (fds > 0) log("   ★★★ code=%u shape=%s 让 HAL 打开了传输 fd=%d", code, sn[shape], fds);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int back = N_TTY;
    ioctl(g_sfd, TIOCSETD, &back);
    close(g_sfd);
    close(g_mfd);
    return 0;
  }
  if (mapCode > 0) {
    // 方法表实测：只 initialize（code 1 已验证正确），然后空参 oneway 发 mapCode，
    // 由 HAL 自己的日志（BluetoothHci::xxx()）告诉我们它是哪个方法。
    int32_t first = 0x7abc;
    binder_status_t ist = callWith(kInitialize, true, 0, &first);
    log("MAP code=%d initialize=%d", mapCode, ist);
    if (ist == ST_OK) {
      binder_status_t st = callWith((uint32_t)mapCode, false, FLAG_ONEWAY, nullptr);
      log("MAP code=%d → 投递 st=%d（看 HAL 日志里的 BluetoothHci:: 行）", mapCode, st);
    }
    // 不发 close（会被误认成方法表的一行）；直接退场让 binder 死亡通知去清理
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int back = N_TTY;
    ioctl(g_sfd, TIOCSETD, &back);
    close(g_sfd);
    close(g_mfd);
    return 0;
  }
  if (probeEnable) {
    // initialize 必须已经成功才有意义
    int32_t first = 0x7abc;
    binder_status_t ist = callWith(kInitialize, true, 0, &first);
    log("probe-enable: initialize → %d 回包首int32=%d", ist, first);
    if (ist != ST_OK) return 1;
    struct Att { const char* name; int reason; };
    static const Att atts[] = {
        {"无参", -1}, {"reason=0", 0}, {"reason=1", 1}, {"reason=2", 2}, {"reason=3", 3},
    };
    for (const auto& a : atts) {
      // 每轮先重新确认句柄：逐个候选服务名试，谁的 initialize 通就用谁
      for (const char* nm : kSvcVariants) {
        AIBinder* b = ndk.SM_getService(nm);
        if (!b) continue;
        if (ndk.AssociateClass) ndk.AssociateClass(b, hciCls);
        int32_t f2 = 0x7abc;
        AParcel* in = nullptr;
        binder_status_t ps = ndk.Prepare(b, &in);
        binder_status_t ws = (ps == ST_OK) ? ndk.Parcel_writeStrongBinder(in, cb) : ps;
        AParcel* out = nullptr;
        binder_status_t ts2 = (ws == ST_OK) ? ndk.Transact(b, kInitialize, &in, &out, 0) : ws;
        if (out) ndk.Parcel_delete(out);  // in 由 transact 消费，不能再删
        log("   · 候选 %-52s prepare=%d init=%d", nm, ps, ts2);
        if (ts2 == ST_OK) { g_hal = b; break; }
      }
      binder_status_t st = callWith(kEnable, false, FLAG_ONEWAY, nullptr, a.reason);
      std::this_thread::sleep_for(std::chrono::seconds(4));
      int soft = rfkillSoft();
      int fds = halOpenTransport();
      log("probe-enable enable/%-9s oneway st=%d → rfkill0.soft=%d HALfd=%d %s", a.name, st, soft,
          fds, soft == 0 ? "★ 已上电" : "");
      if (soft == 0) break;
    }
    initialized = 1;
  } else if (probe4) {
    struct Case { const char* name; uint32_t code; bool withCb; uint32_t flags; };
    static const Case cases[] = {
        {"enable/sync/无参", kEnable, false, 0},
        {"enable/oneway/无参", kEnable, false, FLAG_ONEWAY},
        {"initialize/sync/带回调", kInitialize, true, 0},
        {"initialize/oneway/带回调", kInitialize, true, FLAG_ONEWAY},
        {"initialize/sync/不带回调", kInitialize, false, 0},
    };
    for (const auto& c : cases) {
      int32_t first = 0x7abc;
      binder_status_t st = callWith(c.code, c.withCb, c.flags, &first);
      log("PROBE %-26s code=%u flags=%u → st=%d 回包首int32=%d", c.name, c.code, c.flags, st, first);
    }
    initialized = 1;  // 探测模式不再走正式流程
  } else if (probe5) {
    // 判定 -38 是不是"vendor flavor"闸：对照 system 侧服务与 vendor HAL
    struct Target { const char* name; const char* desc; bool variant; };
    static const Target ts[] = {
        {"activity", "android.app.IActivityManager", false},
        {"android.hardware.power.IPower/default", "android.hardware.power.IPower", false},
        {"蓝牙 HAL（多服务名轮询）", kDescHci, true},
    };
    for (const auto& t : ts) {
      AIBinder* b = nullptr;
      if (t.variant) {
        for (const char* nm : kSvcVariants) {
          b = ndk.SM_getService(nm);
          if (b) { log("     · 命中服务名 %s", nm); break; }
        }
      } else {
        b = ndk.SM_getService(t.name);
      }
      if (!b) { log("PROBE5 %-42s getService=NULL", t.name); continue; }
      if (ndk.Binder_isRemote || ndk.Binder_isNative || ndk.Binder_getVendor)
        log("PROBE5 %-42s isRemote=%d isNative=%d vendor=%d", t.name,
            ndk.Binder_isRemote ? (int)ndk.Binder_isRemote(b) : -1,
            ndk.Binder_isNative ? (int)ndk.Binder_isNative(b) : -1,
            ndk.Binder_getVendor ? ndk.Binder_getVendor(b) : -1);
      AParcel* in = nullptr;
      binder_status_t st = ndk.Prepare(b, &in);
      log("     → Prepare=%d %s%s", st, st == ST_OK ? "(可发!)" : "",
          st == -38 ? " (-38=INVALID_OPERATION)" : "");
      if (in) ndk.Parcel_delete(in);
    }
    initialized = 1;
  } else {
    int32_t first = 0x7abc;
    binder_status_t st = callWith(kInitialize, true, 0, &first);
    log("initialize(A/sync/带回调) → st=%d 回包首int32=%d", st, first);
    if (st != ST_OK) {
      st = callWith(kInitialize, true, FLAG_ONEWAY, &first);
      log("initialize(A/oneway/带回调) → st=%d 回包首int32=%d", st, first);
    }
    if (st != ST_OK) {
      AParcel* in2 = nullptr;
      AParcel* out2 = nullptr;
      st = ndk.Transact(g_hal, kInitialize, &in2, &out2, 0);
      if (out2) ndk.Parcel_delete(out2);
      log("initialize(B/in=nullptr) → st=%d", st);
    }
    if (st != ST_OK) {
      log("✗ initialize 三种打法都不通（enable 未试）");
    } else {
      initialized = 1;
      // enable 的确切形状还没定死：sweep 里 fd=2 出现在 code3 无参之后、code3+int32 之前，
      // 而正式流程带 int32 时始终没开传输 → 逐个候选试，判据=HAL 是否打开传输 fd /
      // rfkill 是否解除阻塞（oneway 的投递 0 不代表执行）。
      {
        struct Try { uint32_t code; int arg; const char* label; };  // arg<0 = 不带参数
        static const Try tries[] = {
            {4, 1, "code4/int=1"},
            {4, 0, "code4/int=0"},
            {4, 2, "code4/int=2"},
            {4, 3, "code4/int=3"},
        };
        bool up = false;
        for (const auto& t : tries) {
          binder_status_t est = callWith(t.code, false, FLAG_ONEWAY, nullptr, t.arg);
          for (int i = 0; i < 6 && !up; i++) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            int soft = rfSoft(), fds = halTransportFds();
            if (soft == 0 || fds > 0) {
              log("★ %s 生效（投递=%d）→ rfkill.soft=%d HAL传输fd=%d", t.label, est, soft, fds);
              up = true;
            }
          }
          if (!up) log("  · %s 无反应（投递=%d）", t.label, est);
          else break;
        }
        if (!up) log("✗ 所有 enable 候选都没让 HAL 开传输");
      }
    }
  }

  if (!initialized) {
    log("未拿到 HAL 客户端位，直接退场");
    callVoid(kDisable, true);
    callVoid(kClose, true);
    int back = N_TTY;
    ioctl(g_sfd, TIOCSETD, &back);
    close(g_sfd);
    close(g_mfd);
    return 1;
  }

  auto until = keep > 0 ? std::chrono::steady_clock::now() + std::chrono::seconds(keep)
                        : std::chrono::steady_clock::time_point::max();
  int tick = 0;
  while (g_run && std::chrono::steady_clock::now() < until) {
    struct pollfd pf{g_mfd, POLLIN, 0};
    int s = poll(&pf, 1, 1000);
    if (s > 0 && (pf.revents & POLLIN)) pumpToHal();
    if (s < 0 && errno != EINTR) break;
    if (++tick % 5 == 0) {
      // 状态快照：内核侧是否已 up、HAL 传输是否开着、芯片是否解除阻塞
      char cmd[160];
      snprintf(cmd, sizeof(cmd), "cat /sys/class/bluetooth/hci0/flags 2>/dev/null || echo NODEV");
      FILE* g = popen(cmd, "r");
      char fl[64] = "?";
      if (g) { if (!fgets(fl, sizeof(fl), g)) snprintf(fl, sizeof(fl), "ERR"); pclose(g); }
      log("状态 hci0.flags=%s HALfd=%d rfkill.soft=%d 已转发包=%llu 收包=%llu", fl, halTransportFds(),
          rfSoft(), (unsigned long long)g_toHal, (unsigned long long)g_toKernel);
    }
  }

  log("退场：disable + close（把 HAL 交还给安卓 framework）");
  callVoid(kDisable, true);
  callVoid(kClose, true);
  if (ndk.DecStrong) ndk.DecStrong(cb);
  int back = N_TTY;
  ioctl(g_sfd, TIOCSETD, &back);
  close(g_sfd);
  close(g_mfd);
  return 0;
}
