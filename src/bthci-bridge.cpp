// bthci-bridge.cpp — DRM 接管期的蓝牙 HCI 桥（IBluetoothHci 的 binder 客户端）
//
// 一个进程干两件事（在安卓侧以 root 运行）：
//   1) 开一对 pty，在 slave 上挂 N_HCI + HCIUARTSETPROTO(H4) → 在共享内核里注册出真 hci0
//      （已实测：零新内核模块；进程退出 → tty 关闭 → hci_unregister_dev 自动回收）
//   2) 做蓝牙 HAL 的 binder 客户端：initialize(回调) → 把 pty 上的 H4 帧与 HAL 双向搬运。
//      上电、固件下载、glink/IBS 全部仍由 HAL 负责，我们只是在它上面多挂一个客户端。
//
// 事务码表**不靠猜**：从本机 /vendor/lib64/android.hardware.bluetooth-V1-ndk.so 反汇编得出
// （每个 Bp 方法在 `bl AIBinder_transact` 前都有一条 `mov w1, #code`），见下面 enum。
//
// 为什么用 dlopen 而不是 -lbinder_ndk：GitHub runner 上那份 NDK 没有 binder_manager.h /
// binder_process.h，其 libbinder_ndk.so stub 也不导出 ABinderProcess_*/AServiceManager_*
// （CI 实测 undefined symbol）。设备上的 /system/lib64/libbinder_ndk.so 是全的 →
// 运行时 dlopen + 自己声明原型，彻底不依赖 NDK 头与 stub，头文件漂移这一整类失败消失。
//
// 绝对不做：不 open /dev/bt_cp_ctrl、不 open /dev/ttyHS0、不打任何 btpower/geni 电源 ioctl、
// 不写 /sys/class/rfkill/rfkill0、不 setenforce 0、不 ctl.stop HAL。
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

// hciconfig hci0 up：实测"自己开 AF_BLUETOOTH socket 打 HCIDEVUP"会 ENODEV，
// 而 nsenter 进容器 ns 跑 hciconfig 成功过。它会阻塞等 init-complete，所以放后台。
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
  return system(cmd);
}

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
  size_t (*Parcel_getDataSize)(const AParcel*);
  void (*Parcel_delete)(AParcel*);
  void (*Proc_setThreadPoolMaxThreadCount)(uint32_t);
  void (*Proc_startThreadPool)(void);
  bool (*Binder_isRemote)(AIBinder*);
  bool (*Binder_isNative)(AIBinder*);
  int32_t (*Binder_getVendor)(AIBinder*);
  void (*ForceDowngradeToVendorStability)(AIBinder*);
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
  ndk.Parcel_writeStrongBinder =
      (decltype(ndk.Parcel_writeStrongBinder))get("AParcel_writeStrongBinder");
  ndk.Parcel_setDataPosition =
      (decltype(ndk.Parcel_setDataPosition))get("AParcel_setDataPosition");
  ndk.Parcel_getDataSize = (decltype(ndk.Parcel_getDataSize))get("AParcel_getDataSize");
  ndk.Parcel_delete = (decltype(ndk.Parcel_delete))get("AParcel_delete");
  ndk.Binder_isRemote = (decltype(ndk.Binder_isRemote))get("AIBinder_isRemote");
  ndk.Binder_isNative = (decltype(ndk.Binder_isNative))get("AIBinder_isNative");
  ndk.Binder_getVendor = (decltype(ndk.Binder_getVendor))get("AIBinder_getVendor");
  ndk.Proc_setThreadPoolMaxThreadCount =
      (decltype(ndk.Proc_setThreadPoolMaxThreadCount))get("ABinderProcess_setThreadPoolMaxThreadCount");
  ndk.Proc_startThreadPool =
      (decltype(ndk.Proc_startThreadPool))get("ABinderProcess_startThreadPool");
  ndk.ForceDowngradeToVendorStability =
      (decltype(ndk.ForceDowngradeToVendorStability))get("AIBinder_forceDowngradeToVendorStability");

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

// ------------------------------------------------------------ 接口与码表
static const char* kSvcHci = "android.hardware.bluetooth.IBluetoothHci/default";
static const char* kDescCallbacks = "android.hardware.bluetooth.IBluetoothHciCallbacks";
static const char* kDescHci = "android.hardware.bluetooth.IBluetoothHci";

// 反汇编本机 android.hardware.bluetooth-V1-ndk.so 得到的事务码：
//   IBluetoothHci           1=close 2=initialize(cb) 3=sendAclData 4=sendHciCommand
//                           5=sendIsoData 6=sendScoData
//   IBluetoothHciCallbacks  1=aclDataReceived 2=hciEventReceived 3=initializationComplete(Status)
//                           4=isoDataReceived 5=scoDataReceived
// initialize 是 **oneway**（同步调它会被拒 -2147483647 且不投递 → 之前"code 2 开传输"的
// 结论其实是安卓自己开的传输）；三个 send* 是同步 void，用 oneway 发等于什么都没送。
enum : uint32_t {
  kClose = 1,
  kInitialize = 2,
  kSendAcl = 3,
  kSendCommand = 4,
  kSendIso = 5,
  kSendSco = 6,
};
enum : uint32_t {
  kCbAcl = 1,
  kCbHciEvent = 2,
  kCbInitComplete = 3,
  kCbIso = 4,
  kCbSco = 5,
};

// AIBinder_Class_define 的副作用就是把 descriptor 在本进程"声明"（Stability::declare）。
// 不声明的话 AIBinder_prepareTransaction 直接返回 -38(INVALID_OPERATION)：
// libbinder 的 Stability::checkDeclared(descriptor) 不过。AIDL 生成的桩本来会替你做这件事。
static void* noopCreate(void*) { return new int(0); }
static void noopDestroy(void* c) { delete static_cast<int*>(c); }
static binder_status_t noopTransact(AIBinder*, transaction_code_t, const AParcel*, AParcel*) {
  return ST_OK;
}

// ------------------------------------------------------------ 进程状态
// 实测（09-24 23:1x，--cmd 011000 = HCI_Read_Local_Version 不带类型字节 → 芯片回 event；
// 带类型字节的 01011000 不回）：QTI 这个 HAL 的 byte[] **不含 H4 类型字节**，两个方向都是。
// 内核 hci_uart(H4) 那侧必须带类型字节，所以桥自己负责加/减。
static bool g_include_type = false;  // 内核→HAL：写进 byte[] 的负载带不带 H4 类型字节
static bool g_cb_has_type = false;   // HAL→内核：回调 byte[] 里带不带 H4 类型字节
static int g_mfd = -1;              // pty master
static int g_sfd = -1;              // pty slave（挂了 N_HCI）
static volatile sig_atomic_t g_run = 1;
static std::mutex g_ptx;            // 回调可能在多个 binder 线程上 → 串行化 pty 写
static AIBinder* g_hal = nullptr;
// 计数一律由**实际发生**的回调/包累加。旧版的 g_cbEvents 从来没有 ++，
// 于是每一轮扫描打印的"event=0"都是在读常量 0 —— 靠它下的结论全部作废。
static volatile unsigned long long g_toHal = 0, g_toKernel = 0, g_cbEvents = 0, g_cbAny = 0;
static volatile unsigned long long g_cbInitSeen = 0;
static volatile int g_cbInitStatus = -1000;  // -1000 = 还没收到 initializationComplete

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

static void hexdump(const char* tag, const int8_t* d, size_t n) {
  char buf[3 * 48 + 1];
  size_t show = n < 47 ? n : 47;
  int p = 0;
  for (size_t i = 0; i < show; i++)
    p += snprintf(buf + p, sizeof(buf) - p, "%02x ", (uint8_t)d[i]);
  buf[p] = 0;
  log("%s (%zu 字节): %s%s", tag, n, buf, n > show ? "…" : "");
}

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

// HAL → 内核：补上 H4 类型字节写进 pty master。
// HAL 给的数组是否自带类型字节**不靠猜**（事件码 0x01~0x05 和类型字节取值范围重叠，
// 首字节启发式一定会读错），用 --cb-with-type 显式切换，实测哪边通定哪边。
static void toKernel(uint8_t h4type, const int8_t* data, size_t n) {
  if (n == 0 || g_mfd < 0) return;
  uint8_t frame[2048];
  size_t flen = 0;
  if (g_cb_has_type) {
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

// 手写服务端不会自动 enforceInterface，得自己跳过 token 才能读到参数。
// 本机实测（dumpParcel 打出的 120 字节 initializationComplete）线格式：
//   [0]=0x80000000 [1]=0xffffffff [2]='SYST'/'VNDR'(Android 15+ 的 tuning 头)
//   [3]=descriptor 字符数(不含 NUL) 之后 UTF-16 + 0x0000，补到 4 字节 → 参数从这里开始
// 前面那几个 flag 字到底是 2 个还是 3 个版本相关，所以不写死偏移：**扫**出那个
// "后面紧跟 UTF-16 ASCII"的长度字，从它算参数起点。少算 4 字节会把参数读歪一位
// （实测把 Status 读成 6422574 = UTF-16 的 "re"）。
// **返回值 = 参数起点**，调用方必须自己 setDataPosition 回去再读：中途任何别的位置操作
// （dumpParcel / 试错扫描）都会把游标挪走 —— 实测因此把 49 字节的 descriptor 当成事件数组读。
static int32_t skipToken(const AParcel* in) {
  if (!ndk.Parcel_readInt32 || !ndk.Parcel_setDataPosition || !ndk.Parcel_getDataSize) return 0;
  size_t sz = ndk.Parcel_getDataSize(in);
  int words = static_cast<int>(sz / 4);
  for (int i = 0; i + 1 < words; i++) {
    ndk.Parcel_setDataPosition(in, i * 4);
    int32_t len = 0;
    if (ndk.Parcel_readInt32(in, &len) != ST_OK) return 0;
    if (len < 8 || len > 120) continue;
    int32_t w = 0;
    if (ndk.Parcel_readInt32(in, &w) != ST_OK) return 0;
    uint8_t b0 = w & 0xff, b1 = (w >> 8) & 0xff, b2 = (w >> 16) & 0xff, b3 = (w >> 24) & 0xff;
    if (b1 != 0 || b3 != 0) continue;  // UTF-16LE 的 ASCII：奇数字节必须是 0
    if (b0 < 0x20 || b0 > 0x7e || b2 < 0x20 || b2 > 0x7e) continue;
    size_t str = (static_cast<size_t>(len) * 2 + 2 + 3) & ~static_cast<size_t>(3);
    size_t pos = static_cast<size_t>(i + 1) * 4 + str;
    if (pos <= sz) return static_cast<int32_t>(pos);
  }
  return 0;
}

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

static void* cbCreate(void*) { return new int(0); }
static void cbDestroy(void* cookie) { delete static_cast<int*>(cookie); }

// 把回调 parcel 从位置 0 起按 int32 全部打印出来。目的：一次性量清 AIDL 线格式
// （Android 15/16 的 Parcel 有 'SYST'/'VNDR' tuning 头 + strict-mode + token 长度 + UTF-16，
// 手算偏移老错，直接看比推可靠）。
static void dumpParcel(const char* tag, const AParcel* in) {
  if (!ndk.Parcel_readInt32 || !ndk.Parcel_setDataPosition) return;
  size_t sz = ndk.Parcel_getDataSize ? ndk.Parcel_getDataSize(in) : 0;
  char buf[9 * 12 + 1];
  int p = 0;
  ndk.Parcel_setDataPosition(in, 0);
  int i = 0;
  for (; i < 12; i++) {
    int32_t w = 0;
    if (ndk.Parcel_readInt32(in, &w) != ST_OK) break;
    p += snprintf(buf + p, sizeof(buf) - p, "%08x ", (uint32_t)w);
  }
  buf[p] = 0;
  log("%s parcel: %d 字节 / 读到 %d 个 int32 = %s", tag, (int)sz, i, buf);
}

static binder_status_t cbOnTransact(AIBinder*, transaction_code_t code, const AParcel* in,
                                    AParcel* out) {
  ++g_cbAny;
  HciBuf hb;
  // 参数起点只认 skipToken 的返回值，并且**每次读之前都显式 setDataPosition**：
  // 中途任何别的位置操作（dumpParcel、试错扫描）都会把游标挪走。
  const int32_t p0 = skipToken(in);
  if (code == kCbInitComplete) {
    int32_t st = 0;
    binder_status_t r = ST_OK;
    if (ndk.Parcel_setDataPosition && ndk.Parcel_readInt32) {
      ndk.Parcel_setDataPosition(in, p0);
      r = ndk.Parcel_readInt32(in, &st);
    }
    g_cbInitStatus = (r == ST_OK) ? st : -1;
    ++g_cbInitSeen;
    // Status 枚举（AIDL IBluetoothHciCallbacks）：0=SUCCESS 1=HARDWARE_FAILURE
    // 2=UNABLE_TO_INIT_ALGO 3=FIRMWARE_PATCH_NOT_SUPPORTED 4=UNKNOWN
    static const char* sn[] = {"SUCCESS", "HARDWARE_FAILURE", "UNABLE_TO_INIT_ALGO",
                               "FIRMWARE_PATCH_NOT_SUPPORTED", "UNKNOWN"};
    log("← initializationComplete(status=%d %s)@%d %s", g_cbInitStatus,
        (g_cbInitStatus >= 0 && g_cbInitStatus <= 4) ? sn[g_cbInitStatus] : "?", p0,
        g_cbInitStatus == 0 ? "★★★ HAL 认了我们这个客户端" : "");
    dumpParcel("initComplete", in);
    if (out && ndk.Parcel_writeInt32) ndk.Parcel_writeInt32(out, 0);
    return ST_OK;
  }
  binder_status_t r = ST_OK;
  int rpos = -1;
  if (ndk.Parcel_setDataPosition && ndk.Parcel_readByteArray) {
    ndk.Parcel_setDataPosition(in, p0);
    r = ndk.Parcel_readByteArray(in, &hb, allocHci);
    if (r == ST_OK && hb.n > 0 && hb.n <= 1024) rpos = p0;
  }
  if (rpos < 0) {
    log("← 按 token 终点(%d)读 byte[] 失败 code=%u st=%d → dump 出来看布局", p0, code, r);
    dumpParcel("byte[]?", in);
  }
  if (out && ndk.Parcel_writeInt32) ndk.Parcel_writeInt32(out, 0);  // EX_NONE
  switch (code) {
    case kCbHciEvent: {
      ++g_cbEvents;
      char t[48];
      snprintf(t, sizeof(t), "← hciEventReceived(数组@%d)", rpos);
      hexdump(t, hb.data, hb.n);
      toKernel(0x04, hb.data, hb.n);
      break;
    }
    case kCbAcl:
      toKernel(0x02, hb.data, hb.n);
      break;
    case kCbSco:
      toKernel(0x03, hb.data, hb.n);
      break;
    case kCbIso:
      toKernel(0x05, hb.data, hb.n);
      break;
    default:
      log("← 未处理的回调 code=%u（%zu 字节）", code, hb.n);
      break;
  }
  return ST_OK;
}

// ------------------------------------------------------------ HAL 侧
static int halPid() {
  FILE* f = popen(
      "pidof android.hardware.bluetooth@aidl-service-qti 2>/dev/null | awk '{print $1}'", "r");
  if (!f) return -1;
  int pid = -1;
  if (fscanf(f, "%d", &pid) != 1) pid = -2;
  pclose(f);
  return pid;
}

// HAL 有没有把 glink/UART 打开（=它真的在驱动芯片）
static int halTransportFds() {
  char cmd[160];
  snprintf(cmd, sizeof(cmd),
           "ls -l /proc/%d/fd 2>/dev/null | grep -cE 'bt_cp_ctrl|ttyHS'", halPid());
  FILE* g = popen(cmd, "r");
  int fds = -1;
  if (g) {
    if (fscanf(g, "%d", &fds) != 1) fds = -2;
    pclose(g);
  }
  return fds;
}

static int rfSoft() {
  FILE* f = fopen("/sys/class/rfkill/rfkill0/soft", "r");
  if (!f) return -1;
  int v = -2;
  if (fscanf(f, "%d", &v) != 1) v = -3;
  fclose(f);
  return v;
}

// HAL 自己很啰嗦，它抱怨的那一行往往就是答案（固件下载失败/IBS 超时/权限）。
// 安卓 framework 死了 logd 照样在（class core），所以接管期也能读。
static void dumpHalLog(int n) {
  char cmd[224];
  snprintf(cmd, sizeof(cmd),
           "logcat -d -t %d 2>/dev/null | grep -iE 'bluetooth|btpower|ibs_|vendor.qti|bt_vendor|"
           "hal_bluetooth' | tail -18",
           n);
  FILE* g = popen(cmd, "r");
  if (!g) return;
  char ln[512];
  while (fgets(ln, sizeof(ln), g)) {
    size_t e = strlen(ln);
    while (e && (ln[e - 1] == '\n' || ln[e - 1] == '\r')) ln[--e] = 0;
    fprintf(stderr, "[bthci]  HAL| %s\n", ln);
  }
  pclose(g);
}

// 一包发给 HAL。payload 是否含 H4 类型字节由 g_include_type 决定；
// 三个 send* 是**同步**方法，flags 必须为 0（oneway 发过去等于没送）。
static binder_status_t sendToHal(uint32_t code, const int8_t* payload, size_t n, uint32_t flags) {
  AParcel* in = nullptr;
  binder_status_t st = ndk.Prepare(g_hal, &in);
  if (st != ST_OK) return st;
  st = ndk.Parcel_writeByteArray(in, payload, n);
  if (st != ST_OK) {
    ndk.Parcel_delete(in);
    return st;
  }
  AParcel* out = nullptr;
  st = ndk.Transact(g_hal, code, &in, &out, flags);
  if (out) ndk.Parcel_delete(out);
  return st;
}

static binder_status_t callInitialize(AIBinder* cb) {
  AParcel* in = nullptr;
  binder_status_t st = ndk.Prepare(g_hal, &in);
  if (st == ST_OK) st = ndk.Parcel_writeStrongBinder(in, cb);
  AParcel* out = nullptr;
  if (st == ST_OK) st = ndk.Transact(g_hal, kInitialize, &in, &out, FLAG_ONEWAY);
  if (out) ndk.Parcel_delete(out);
  return st;
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
    else if ((type == 0x02 || type == 0x04) && pos + 5 <= acc.size())
      need = 5 + static_cast<size_t>(acc[pos + 3] | (acc[pos + 4] << 8));
    else if (type == 0x03 && pos + 6 <= acc.size())
      need = 6 + acc[pos + 5];
    else if (type > 0x04) {
      log("未知 H4 类型 0x%02x，丢 1 字节", type);
      ++pos;
      continue;
    } else {
      break;  // 帧还没收全
    }
    if (pos + need > acc.size()) break;
    const uint8_t* pkt = acc.data() + pos;
    ++g_toHal;
    uint32_t code = type == 0x01 ? kSendCommand : type == 0x02 ? kSendAcl
                                             : type == 0x03    ? kSendSco
                                                               : kSendIso;
    const int8_t* payload = g_include_type ? (const int8_t*)pkt : (const int8_t*)pkt + 1;
    size_t n = g_include_type ? need : need - 1;
    binder_status_t st = sendToHal(code, payload, n, 0);
    if (st != ST_OK) log("→ HAL code=%u st=%d（这包发失败了）", code, st);
    pos += need;
  }
  acc.erase(acc.begin(), acc.begin() + pos);
}

static int hex2bytes(const char* s, uint8_t* out, int maxn) {
  auto hv = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  int n = 0;
  while (*s && n < maxn) {
    while (*s == ' ' || *s == ',') s++;
    if (!s[0] || !s[1]) break;
    int hi = hv(s[0]), lo = hv(s[1]);
    if (hi < 0 || lo < 0) return -1;
    out[n++] = static_cast<uint8_t>((hi << 4) | lo);
    s += 2;
  }
  return n;
}

int main(int argc, char** argv) {
  int keep = 60;
  bool kick = true, cmdMode = false;
  const char* cmdHex = nullptr;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--keep") && i + 1 < argc)
      keep = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--with-type-byte"))
      g_include_type = true;
    else if (!strcmp(argv[i], "--cb-with-type"))
      g_cb_has_type = true;
    else if (!strcmp(argv[i], "--no-kick"))
      kick = false;
    else if (!strcmp(argv[i], "--cmd") && i + 1 < argc) {
      cmdMode = true;
      cmdHex = argv[++i];
    } else if (!strcmp(argv[i], "--codes")) {
      printf("IBluetoothHci:          1=close 2=initialize(cb,oneway) 3=sendAclData "
             "4=sendHciCommand 5=sendIsoData 6=sendScoData\n");
      printf("IBluetoothHciCallbacks: 1=aclDataReceived 2=hciEventReceived "
             "3=initializationComplete(Status) 4=isoDataReceived 5=scoDataReceived\n");
      printf("来源：反汇编本机 /vendor/lib64/android.hardware.bluetooth-V1-ndk.so 里\n"
             "      每个 Bp 方法 bl AIBinder_transact 前的 mov w1, #code\n");
      printf("HAL pid=%d 传输fd数=%d rfkill0.soft=%d\n", halPid(), halTransportFds(), rfSoft());
      return 0;
    } else {
      fprintf(stderr,
              "用法: %s [--keep <秒>] [--no-kick] [--with-type-byte] [--cb-with-type] "
              "[--cmd <hex>] [--codes]\n"
              "  默认：注册 hci0 + initialize + 双向搬运（keep 秒）\n"
              "  --cmd 011000：不经内核，直接用 sendHciCommand 发一条命令（**不带** H4 类型字节），"
              "只看回调\n",
              argv[0]);
      return 2;
    }
  }
  signal(SIGINT, on_term);
  signal(SIGTERM, on_term);
  signal(SIGPIPE, SIG_IGN);

  if (!loadNdk()) return 1;

  // 目标接口的类：只为声明 descriptor（Stability）+ 让 prepareTransaction 写对 interface token
  const AIBinder_Class* hciCls = ndk.Class_define(kDescHci, noopCreate, noopDestroy, noopTransact);
  if (!hciCls) {
    log("✗ 无法声明 %s", kDescHci);
    return 1;
  }
  const AIBinder_Class* cls = ndk.Class_define(kDescCallbacks, cbCreate, cbDestroy, cbOnTransact);
  if (!cls) {
    log("✗ AIBinder_Class_define(%s) 失败", kDescCallbacks);
    return 1;
  }
  AIBinder* cb = ndk.New(cls, nullptr);
  if (!cb) {
    log("✗ AIBinder_new 失败");
    return 1;
  }
  // vendor HAL 可能要求回调是 vendor-stable；有这个符号就用（对远端句柄不能调，会 FATAL）
  if (ndk.ForceDowngradeToVendorStability) ndk.ForceDowngradeToVendorStability(cb);
  ndk.Proc_setThreadPoolMaxThreadCount(4);
  ndk.Proc_startThreadPool();

  g_hal = ndk.SM_getService(kSvcHci);
  if (!g_hal && ndk.SM_waitForService) g_hal = ndk.SM_waitForService(kSvcHci);
  if (!g_hal) {
    log("✗ 拿不到 %s —— HAL 活着吗？", kSvcHci);
    return 1;
  }
  // getService 返回的是没 class 的裸句柄，不 associateClass 一律 -38
  ndk.AssociateClass(g_hal, hciCls);
  log("✓ 拿到 %s 并已 associateClass", kSvcHci);
  if (ndk.Binder_isRemote) log("  isRemote=%d", (int)ndk.Binder_isRemote(g_hal));

  int rc = 1;
  if (!cmdMode && !attachHci()) goto out;

  g_cbAny = 0;
  g_cbEvents = 0;
  g_cbInitSeen = 0;
  g_cbInitStatus = -1000;
  {
    binder_status_t ist = callInitialize(cb);
    log("initialize(oneway, 带回调) 投递=%d", ist);
    int waited = 0;
    while (g_run && g_cbInitSeen == 0 && waited < 24) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      waited++;
    }
    log("回调数=%llu initSeen=%llu status=%d HAL传输fd=%d rfkill.soft=%d",
        (unsigned long long)g_cbAny, (unsigned long long)g_cbInitSeen, g_cbInitStatus,
        halTransportFds(), rfSoft());
    if (g_cbInitSeen == 0) {
      log("✗ HAL 一个回调都没发 → 客户端位没坐上（查回调 binder 的稳定性/送达）");
      dumpHalLog(120);
      goto out;
    }
    if (g_cbInitStatus != 0) {
      log("· status=%d（非 0）→ HAL 自己怎么说的：", g_cbInitStatus);
      dumpHalLog(120);
    }
  }

  if (cmdMode) {
    // 剔除"内核 hci0"这个变量：自己发一条 HCI 命令，只看 HAL 有没有把 event 送回回调。
    uint8_t buf[128];
    int n = hex2bytes(cmdHex, buf, sizeof(buf));
    if (n <= 0) {
      log("✗ --cmd 的十六进制解析失败");
      goto out;
    }
    hexdump("→ sendHciCommand", (int8_t*)buf, n);
    binder_status_t st = sendToHal(kSendCommand, (int8_t*)buf, n, 0);
    log("sendHciCommand 返回=%d", st);
    unsigned long long before = g_cbEvents;
    for (int i = 0; i < 16 && g_run; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (g_cbEvents > before) break;
    }
    log("结果：新收到 hciEvent=%llu %s", (unsigned long long)(g_cbEvents - before),
        g_cbEvents > before ? "★★★ binder↔芯片 链路通了" : "✗ 芯片没回 event");
    dumpHalLog(80);
    rc = g_cbEvents > before ? 0 : 1;
    goto out;
  }

  if (kick) {
    int kr = kickHci(1);
    log("hciconfig hci0 up 已后台发出 rc=%d（内核会立刻发 HCI_Reset）", kr);
  }
  {
    auto until = keep > 0 ? std::chrono::steady_clock::now() + std::chrono::seconds(keep)
                          : std::chrono::steady_clock::time_point::max();
    int tick = 0;
    while (g_run && std::chrono::steady_clock::now() < until) {
      struct pollfd pf{g_mfd, POLLIN, 0};
      int s = poll(&pf, 1, 1000);
      if (s > 0 && (pf.revents & POLLIN)) pumpToHal();
      if (s < 0 && errno != EINTR) break;
      if (++tick % 10 == 0) {
        char fl[64] = "?";
        FILE* g = popen("cat /sys/class/bluetooth/hci0/flags 2>/dev/null || echo NODEV", "r");
        if (g) {
          if (!fgets(fl, sizeof(fl), g)) snprintf(fl, sizeof(fl), "ERR");
          pclose(g);
        }
        log("状态 hci0.flags=%s 转发=%llu 收回=%llu 回调=%llu event=%llu HALfd=%d rfkill.soft=%d",
            fl, (unsigned long long)g_toHal, (unsigned long long)g_toKernel,
            (unsigned long long)g_cbAny, (unsigned long long)g_cbEvents, halTransportFds(),
            rfSoft());
      }
    }
    rc = 0;
  }

out:
  // 退场：不 disable、不 close —— 进程一退 binder 死亡通知会让 HAL 自己回收这个客户端；
  // pty 一关内核就注销 hci0。手打电源路径是本项目的红线（三次强启换来的教训）。
  if (ndk.DecStrong) ndk.DecStrong(cb);
  if (g_sfd >= 0) {
    int back = N_TTY;
    ioctl(g_sfd, TIOCSETD, &back);
    close(g_sfd);
  }
  if (g_mfd >= 0) close(g_mfd);
  return rc;
}
