/* bthci-bridge.c — DRM 接管期的蓝牙 HCI 桥：内核 hci0 ⇄ vendor IBluetoothHci HAL
 *
 * 一个进程干完所有活（跑在安卓侧 root，见 方案.md §4.2）：
 *   1) 开一对 pty，在 slave 上挂 N_HCI + HCIUARTSETPROTO(H4) → 共享内核里注册出 hci0
 *      （m1-probe 实测过：零新内核模块；进程退出→tty 关闭→hci_unregister_dev 自动回收）
 *   2) 作为 binder 客户端连 android.hardware.bluetooth.IBluetoothHci/default
 *      （HAL 自己负责 btpower 上电 / glink / 固件下载——本程序绝不碰那些 ioctl）
 *   3) pty master 上的 H4 帧 ⇄ binder 方法，双向搬运
 *
 * 手写 binder（不依赖 NDK/libbinder：本机没有 NDK，且 glibc 进程无法 dlopen bionic 的
 * libbinder_ndk）。只用 UAPI 头 linux/android/binder.h 的权威结构体定义。
 *
 * 构建：sh build.sh            （产出静态 /tmp/bthci-bridge）
 * 用法：bthci-bridge --probe [binder-node]   只验 getService/initialize/enable + 打回调
 *       bthci-bridge [--node /dev/vndbinder] [--keep <秒>]  正式搬运
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/android/binder.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef TIOCSETD
#define TIOCSETD 0x541b
#endif
#ifndef N_HCI
#define N_HCI 15
#endif
#ifndef HCIUARTSETPROTO
#define HCIUARTSETPROTO 0x400455C8 /* _IOW('U', 200, int) */
#endif
#ifndef HCIUARTGETDEVICE
#define HCIUARTGETDEVICE 0x800455CA /* _IOR('U', 202, int) */
#endif
#define HCI_UART_H4 0

#define BINDER_MAP_SIZE (1024 * 1024)

/* libbinder 保留事务码（B_PACK_CHARS('_','P','I','N') 一类的 fourcc） */
#define TC_PING 0x5f504e47u /* '_PNG' → PING_TRANSACTION，stable AIDL 回 descriptor */
#define TC_INTERFACE 0x5f544e49u /* '_TNI' → INTERFACE_TRANSACTION（旧式） */


#define DESC_SM "android.os.IServiceManager"
#define DESC_HCI "android.hardware.bluetooth.IBluetoothHci"
#define DESC_CB "android.hardware.bluetooth.IBluetoothHciCallbacks"
#define SVC_HCI "android.hardware.bluetooth.IBluetoothHci/default"

/* IBluetoothHci 方法事务码（AIDL：FIRST_CALL_TRANSACTION + 声明顺序）
 * 依据 AOSP hardware/interfaces/bluetooth/aidl/android/hardware/bluetooth/IBluetoothHci.aidl
 * 1 initialize 2 enable 3 disable 4 close 5 sendHciCommand 6 sendAclData 7 sendScoData
 * 8 configure 9 downConfigure
 * IBluetoothHciCallbacks：1 transportReset 2 hciEvent 3 aclDataReceived
 *                        4 scoDataReceived 5 latencyInformationChanged 6 flowStatus */
enum {
	TC_INITIALIZE = 1,
	TC_ENABLE = 2,
	TC_DISABLE = 3,
	TC_CLOSE = 4,
	TC_SEND_CMD = 5,
	TC_SEND_ACL = 6,
	TC_SEND_SCO = 7,
};

/* HAL 约定：vec<uint8_t> 里是**含 H4 类型字节**的完整 HCI 包（Fluoride 就这么传）。
 * 若 M2 实测 HAL 不认，改成 0 再试一次即可（两种都是合法解读）。 */
static int g_include_type = 1;

static void *g_bmap;      /* binder mmap 基址（回包 payload 在这里面） */
static size_t wr_probe_len;   /* 最近一次 ioctl 的 read_consumed，供 --known 扫特征 */
static int g_bfd = -1;      /* binder fd */
static uint8_t *g_rbuf;     /* binder 读缓冲（内核把 incoming 数据拷进来） */
static size_t g_rcap;
static uint32_t g_hal = 0;  /* IBluetoothHci 的远程 handle */
static int g_mfd = -1;      /* pty master */
static int g_sfd = -1;      /* pty slave（挂了 N_HCI） */
static volatile sig_atomic_t g_run = 1;
static int g_verbose = 1;

static void say(const char *fmt, ...)
{
	va_list ap;
	if (!g_verbose)
		return;
	va_start(ap, fmt);
	fprintf(stderr, "[bthci] ");
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
}

static void on_term(int sig)
{
	(void)sig;
	g_run = 0;
}

/* ============================ Parcel（写侧） ============================ */
typedef struct {
	uint8_t *b;
	size_t len, cap;
	binder_size_t *o;
	size_t no, ocap;
} Parcel;

static void dump(const char *tag, const Parcel *p)
{
	if (!p->b) {
		say("%s: <空回包>", tag);
		return;
	}
	fprintf(stderr, "[bthci] %s: %zu 字节\n  hex: ", tag, p->len);
	for (size_t i = 0; i < p->len && i < 96; i++)
		fprintf(stderr, "%02x", p->b[i]);
	fprintf(stderr, "\n  txt: ");
	for (size_t i = 0; i < p->len && i < 96; i++) {
		uint8_t c = p->b[i];
		fputc(c >= 0x20 && c < 0x7f ? c : '.', stderr);
	}
	fprintf(stderr, "\n");
	fflush(stderr);
}


static void p_free(Parcel *p)
{
	free(p->b);
	free(p->o);
	p->b = NULL;
	p->o = NULL;
	p->len = p->cap = p->no = p->ocap = 0;
}

static void p_need(Parcel *p, size_t n)
{
	if (p->len + n <= p->cap)
		return;
	size_t c = (p->cap ? p->cap : 256);
	while (c < p->len + n)
		c *= 2;
	p->b = realloc(p->b, c);
	if (!p->b) {
		say("OOM parcel");
		exit(1);
	}
	memset(p->b + p->cap, 0, c - p->cap);
	p->cap = c;
}

static void p_align(Parcel *p, size_t a)
{
	p_need(p, a);
	while (p->len % a)
		p->b[p->len++] = 0;
}

static void p_u32(Parcel *p, uint32_t v)
{
	p_need(p, 4);
	memcpy(p->b + p->len, &v, 4);
	p->len += 4;
}

static void p_off(Parcel *p, size_t at)
{
	if (p->no == p->ocap) {
		p->ocap = p->ocap ? p->ocap * 2 : 8;
		p->o = realloc(p->o, p->ocap * sizeof(binder_size_t));
	}
	p->o[p->no++] = at; /* 低位 3 bit 保留给对象类型，经典 flat object 填 0 */
}

/* AIDL String16：int32 字符数（不含结尾 0）+ UTF-16LE + uint16 0，再补到 4 字节对齐 */
static void p_string16(Parcel *p, const char *s)
{
	size_t n = strlen(s);
	p_u32(p, (uint32_t)n);
	p_need(p, n * 2 + 2);
	for (size_t i = 0; i < n; i++) {
		p->b[p->len] = (uint8_t)s[i];
		p->b[p->len + 1] = 0;
		p->len += 2;
	}
	p->b[p->len++] = 0;
	p->b[p->len++] = 0;
	p_align(p, 4);
}

/* AIDL stable 的 interface token：int32 0（strict-mode/flags）+ descriptor */
static void p_token(Parcel *p, const char *desc)
{
	p_u32(p, 0);
	p_string16(p, desc);
}

/* AIDL byte[]：int32 长度 + 原始字节，补到 4 字节对齐 */
static void p_bytes(Parcel *p, const uint8_t *d, size_t n)
{
	p_u32(p, (uint32_t)n);
	p_need(p, n + 4);
	memcpy(p->b + p->len, d, n);
	p->len += n;
	p_align(p, 4);
}

/* 本地 binder 对象（我们当被调方）：BINDER_TYPE_BINDER + cookie 指回我们自己 */
static void p_local_binder(Parcel *p)
{
	p_align(p, 8);
	struct flat_binder_object *fbo = (struct flat_binder_object *)(p->b + p->len);
	fbo->hdr.type = BINDER_TYPE_BINDER;
	fbo->flags = FLAT_BINDER_FLAG_ACCEPTS_FDS;
	fbo->binder = (binder_uintptr_t)(uintptr_t)&g_hal; /* cookie 值随便，回来时原样给我们 */
	fbo->cookie = (binder_uintptr_t)(uintptr_t)&g_hal;
	p->len += sizeof(*fbo);
	p_off(p, (size_t)((uint8_t *)fbo - p->b));
}

/* ============================ Parcel（读侧） ============================ */
typedef struct {
	uint8_t *b;
	size_t len, pos;
} RParcel;

static uint32_t r_u32(RParcel *r)
{
	uint32_t v = 0;
	if (r->pos + 4 <= r->len) {
		memcpy(&v, r->b + r->pos, 4);
		r->pos += 4;
	}
	return v;
}

static void r_skip_string16(RParcel *r)
{
	uint32_t n = r_u32(r);
	size_t bytes = (size_t)n * 2 + 2;
	r->pos += bytes;
	r->pos = (r->pos + 3) & ~(size_t)3;
}

static size_t r_bytes(RParcel *r, uint8_t *out, size_t cap)
{
	uint32_t n = r_u32(r);
	if (n == 0xffffffffu)
		return 0;
	if (r->pos + n > r->len)
		n = (uint32_t)(r->len - r->pos);
	if (n > cap)
		n = (uint32_t)cap;
	if (n)
		memcpy(out, r->b + r->pos, n);
	r->pos += n;
	r->pos = (r->pos + 3) & ~(size_t)3;
	return n;
}

/* ============================ binder 核心 ============================ */
static void handle_incoming(struct binder_transaction_data *td);
static int binder_consume(size_t total, Parcel *reply);
static int binder_write_only(uint8_t *wbuf, size_t wlen);
static int binder_wait_reply(Parcel *rep);
static int binder_call(uint32_t handle, uint32_t code, Parcel *in, Parcel *rep, int flags);
static int binder_open(const char *node)
{
	int fd = open(node, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		say("open %s 失败: %s", node, strerror(errno));
		return -1;
	}
	size_t max_threads = 1;
	ioctl(fd, BINDER_SET_MAX_THREADS, &max_threads);
	/* 必须 mmap：内核把回包/incoming 的数据分配在进程这块映射区里，
	 * 没映射就 "binder_alloc_buf, no vma" → BR_DEAD_REPLY（实测踩过） */
	void *map = mmap(nullptr, BINDER_MAP_SIZE, PROT_READ, MAP_PRIVATE, fd, 0);
	if (map == MAP_FAILED) {
		say("mmap(%s, %dKB) 失败: %s", node, (int)(BINDER_MAP_SIZE / 1024), strerror(errno));
		close(fd);
		return -1;
	}
	g_bmap = map;
	say("mmap ok %p %zuKB", map, BINDER_MAP_SIZE / 1024);
	g_rcap = 64 * 1024;
	g_rbuf = malloc(g_rcap);
	g_bfd = fd;
	say("binder 节点 %s fd=%d", node, fd);
	return fd;
}

/* Android 16 libbinder 在每个事务 parcel 开头放 12 字节 header，fourcc 在偏移 8：
 * vndbinder 侧对端期望 'VNDR'(0x564e4452)，/dev/binder 侧对端期望 'SYST'(0x53595354)。
 * 缺了它 servicemanager 会把 getService 解析成 null（实测）。前 8 字节的含义待测，故试几种排布。 */
#define FOURCC_VNDR 0x564e4452u
#define FOURCC_SYST 0x53595354u
static void p_header(Parcel *p, int variant, uint32_t fourcc)
{
	switch (variant) {
	case 0: p_u32(p, 0); p_u32(p, 0); p_u32(p, fourcc); break;
	case 1: p_u32(p, 12); p_u32(p, 0); p_u32(p, fourcc); break;
	case 2: p_u32(p, fourcc); p_u32(p, 0); p_u32(p, 0); break;
	case 3: p_u32(p, 0); p_u32(p, fourcc); p_u32(p, 0); break;
	case 4: p_u32(p, 12); p_u32(p, 1); p_u32(p, fourcc); break;
	case 5: p_u32(p, 8); p_u32(p, 0); p_u32(p, fourcc); break;
	case 6: p_u32(p, 24); p_u32(p, 8); p_u32(p, fourcc); break;
	case 7: p_u32(p, fourcc); p_u32(p, 12); p_u32(p, 0); break;
	case 8: p_u32(p, fourcc); p_u32(p, 0); p_u32(p, 12); break;
	default: break;
	}
}

static int try_getservice(uint32_t code, int variant, uint32_t fourcc, uint32_t *out_handle)
{
	Parcel in;
	memset(&in, 0, sizeof(in));
	if (variant >= 20) {
		/* token 的第一个 int32 直接放 fourcc（不加 12 字节前缀） */
		p_u32(&in, fourcc);
		p_string16(&in, DESC_SM);
	} else if (variant >= 10) {
		/* 完全不写 token */
	} else if (variant >= 0) {
		p_header(&in, variant - 10 < 0 ? variant : variant, fourcc);
		p_token(&in, DESC_SM);
	}
	p_string16(&in, SVC_HCI);
	Parcel rep;
	memset(&rep, 0, sizeof(rep));
	int rc = binder_call(0, code, &in, &rep, 0);
	int found = -1;
	if (rc == 0 && rep.b) {
		for (size_t off = 0; off + 8 <= rep.len; off += 4) {
			uint32_t t;
			memcpy(&t, rep.b + off, 4);
			if (t == BINDER_TYPE_HANDLE) {
				uint32_t h;
				memcpy(&h, rep.b + off + 8, 4);
				found = (int)h;
				*out_handle = h;
				break;
			}
		}
	}
	say("  variant=%d code=%u → rc=%d 回包=%zu HANDLE=%d%s", variant, code, rc, rep.len, found,
	    rep.b && rep.len >= 4 && *(uint32_t *)rep.b == 0x80000001u ? " [EX_SECURITY]" : "");
	p_free(&in);
	p_free(&rep);
	return found >= 0 ? 0 : -1;
}

/* 我们是被调方：HAL 的 IBluetoothHciCallbacks 打进来 → 解出 HCI 包 → 写进 pty master */
static void handle_incoming(struct binder_transaction_data *td)
{
	RParcel in;
	/* payload 由内核放进进程的 binder mmap 区，buffer 字段本身就是可用用户指针 */
	in.b = (uint8_t *)(uintptr_t)td->data.ptr.buffer;
	in.len = (size_t)td->data_size;
	in.pos = 0;

	uint32_t code = td->code;
	r_u32(&in);			/* interface token 的 flags（来者不拒，不校验） */
	r_skip_string16(&in);		/* descriptor */

	uint8_t payload[1024];
	size_t n = 0;
	int is_data = 0;
	uint8_t h4type = 0;

	switch (code) {
	case 1: /* transportReset */
		say("← transportReset");
		break;
	case 2: /* hciEvent */
		n = r_bytes(&in, payload, sizeof(payload));
		h4type = 0x04;
		is_data = 1;
		break;
	case 3: /* aclDataReceived */
		n = r_bytes(&in, payload, sizeof(payload));
		h4type = 0x02;
		is_data = 1;
		break;
	case 4: /* scoDataReceived */
		n = r_bytes(&in, payload, sizeof(payload));
		h4type = 0x03;
		is_data = 1;
		break;
	case 6: /* flowStatus */
		say("← flowStatus（忽略）");
		break;
	default:
		say("← 未处理方法 code=%u", code);
		break;
	}

	if (is_data && n && g_mfd >= 0) {
		uint8_t frame[1030];
		size_t flen;
		/* HAL 约定 vec 里第 0 字节就是 H4 类型；不带就自己补 */
		if (payload[0] == 0x02 || payload[0] == 0x03 || payload[0] == 0x04) {
			memcpy(frame, payload, n);
			flen = n;
		} else {
			frame[0] = h4type;
			memcpy(frame + 1, payload, n);
			flen = n + 1;
		}
		if (write(g_mfd, frame, flen) < 0)
			say("写 pty 失败: %s", strerror(errno));
	}

	/* 回包：EX_NONE(0)。BC_FREE_BUFFER 释放内核塞进来的 data */
	Parcel out;
	memset(&out, 0, sizeof(out));
	p_u32(&out, 0);
	uint8_t wbuf[128];
	size_t wl = 0;
	uint32_t bcf = BC_FREE_BUFFER;
	uint64_t fp = td->data.ptr.buffer;
	memcpy(wbuf + wl, &bcf, 4);
	wl += 4;
	memcpy(wbuf + wl, &fp, 8);
	wl += 8;
	uint32_t bcr = BC_REPLY;
	memcpy(wbuf + wl, &bcr, 4);
	wl += 4;
	struct binder_transaction_data rep;
	memset(&rep, 0, sizeof(rep));
	rep.target.handle = td->target.handle;
	rep.cookie = td->cookie;
	rep.data_size = out.len;
	rep.data.ptr.buffer = (binder_uintptr_t)out.b;
	memcpy(wbuf + wl, &rep, sizeof(rep));
	wl += sizeof(rep);
	binder_write_only(wbuf, wl);   /* 注意：上面 out.b 必须在 ioctl 期间存活 */
	free(out.b);
	free(out.o);
}

/* 主循环里被 binder_call 用来等迟到回包 */
static int binder_wait_reply(Parcel *rep)
{
	struct pollfd pf = { .fd = g_bfd, .events = POLLIN };
	for (int i = 0; i < 20; i++) {
		if (poll(&pf, 1, 250) <= 0)
			continue;
		struct binder_write_read wr;
		memset(&wr, 0, sizeof(wr));
		wr.read_size = g_rcap;
		wr.read_buffer = (uintptr_t)g_rbuf;
		if (ioctl(g_bfd, BINDER_WRITE_READ, &wr) < 0)
			return -1;
		int rc = binder_consume(wr.read_consumed, rep);
		if (rc == 0 || rc == -2 || rc == -3)
			return rc;
	}
	return -1;
}

/* 只做 write（BC_FREE_BUFFER / BC_REPLY 这类不需要回包的路径）。
 * 注意：read_size=0 时内核会把事务当"异步、不要回包"，所以同步调用绝不能走这里。 */
static int binder_write_only(uint8_t *wbuf, size_t wlen)
{
	struct binder_write_read wr;
	memset(&wr, 0, sizeof(wr));
	wr.write_size = wlen;
	wr.write_buffer = (uintptr_t)wbuf;
	wr.read_size = 0;
	wr.read_buffer = (uintptr_t)g_rbuf;
	if (ioctl(g_bfd, BINDER_WRITE_READ, &wr) < 0) {
		say("BINDER_WRITE_READ(write_only) 失败: %s", strerror(errno));
		return -1;
	}
	return 0;
}

/* 消费内核放进 g_rbuf 的命令流。
 * 返回：0=取到回包（已填 reply）/ -2=死或失败 / -1=没有回包（可能处理了 incoming） */
static int binder_consume(size_t total, Parcel *reply)
{
	size_t consumed = 0;
	while (consumed < total) {
		uint8_t *p = g_rbuf + consumed;
		uint32_t cmd = *(uint32_t *)p;
		consumed += 4;
		switch (cmd) {
		case BR_NOOP:
			break;
		case BR_TRANSACTION_COMPLETE: /* 异步事务已完成 */
			break;
		case BR_TRANSACTION_SEC_CTX:
		case BR_TRANSACTION: {
			/* p 指着命令字，负载结构体在它后面 4 字节 */
			struct binder_transaction_data *td = (struct binder_transaction_data *)(p + 4);
			consumed += (cmd == BR_TRANSACTION_SEC_CTX)
						    ? sizeof(struct binder_transaction_data_secctx)
						    : sizeof(*td);
			handle_incoming(td);
			break;
		}
		case BR_REPLY: {
			struct binder_transaction_data *td = (struct binder_transaction_data *)(p + 4);
			{
				const uint8_t *q = (const uint8_t *)td;
				for (int k = 0; k < 8; k++) {
					uint64_t v;
					memcpy(&v, q + k * 8, 8);
					fprintf(stderr, "[bthci]     +%2zu = 0x%016llx\n", k * 8, (unsigned long long)v);
				}
				say("  · sizeof=%zu offsetof data_size=%zu offsets_size=%zu data=%zu",
				    sizeof(*td), offsetof(struct binder_transaction_data, data_size),
				    offsetof(struct binder_transaction_data, offsets_size),
				    offsetof(struct binder_transaction_data, data));
				fprintf(stderr, "[bthci] MMAP 前 64 字节:");
				for (int k = 0; k < 64; k++)
					fprintf(stderr, " %02x", ((uint8_t *)g_bmap)[k]);
				fprintf(stderr, "\n");
			}
			consumed += sizeof(*td);
			if (reply && td->data_size) {
				uint8_t *dp = (uint8_t *)(uintptr_t)td->data.ptr.buffer;
				reply->b = malloc(td->data_size + 8);
				memcpy(reply->b, dp, td->data_size);
				reply->len = td->data_size;
				reply->cap = td->data_size + 8;
			}
			uint8_t wbuf[64];
			size_t wl = 0;
			uint32_t bcf = BC_FREE_BUFFER;
			uint64_t fp = td->data.ptr.buffer;
			memcpy(wbuf + wl, &bcf, 4);
			wl += 4;
			memcpy(wbuf + wl, &fp, 8);
			wl += 8;
			binder_write_only(wbuf, wl);
			return 0;
		}
		case BR_DEAD_REPLY:
			say("BR_DEAD_REPLY（目标已死）");
			return -2;
		case BR_FAILED_REPLY:
			say("BR_FAILED_REPLY（handle 无效/权限被拒/对方拒绝）");
			return -3;
		case BR_DEAD_BINDER:
			say("BR_DEAD_BINDER");
			consumed += 8;
			return -2;
		case BR_ERROR: {
			int32_t e = 0;
			memcpy(&e, p + 4, 4);
			consumed += 4;
			say("BR_ERROR %d", e);
			return -2;
		}
		case BR_FINISHED:
			return -1;
		default:
			say("未识别 BR 0x%x，停在此处", cmd);
			return -1;
		}
	}
	return -1;
}

/* 一次同步调用：写 BC_TRANSACTION + 同一次 ioctl 里读回包（read_size>0 是关键） */
static int binder_call(uint32_t handle, uint32_t code, Parcel *in, Parcel *rep, int flags)
{
	size_t wl = 0;
	uint8_t *wbuf = malloc(8 + sizeof(struct binder_transaction_data));
	uint32_t bc = BC_TRANSACTION;
	memcpy(wbuf + wl, &bc, 4);
	wl += 4;
	struct binder_transaction_data tr;
	memset(&tr, 0, sizeof(tr));
	tr.target.handle = handle;
	tr.code = code;
	tr.flags = flags;
	tr.data_size = in ? in->len : 0;
	tr.offsets_size = in ? in->no * sizeof(binder_size_t) : 0;
	tr.data.ptr.buffer = (in && in->len) ? (binder_uintptr_t)in->b : 0;
	tr.data.ptr.offsets = (in && in->no) ? (binder_uintptr_t)in->o : 0;
	memcpy(wbuf + wl, &tr, sizeof(tr));
	wl += sizeof(tr);

	struct binder_write_read wr;
	memset(&wr, 0, sizeof(wr));
	wr.write_size = wl;
	wr.write_buffer = (uintptr_t)wbuf;
	/* 内核用 !!read_size 决定这是不是"要回包"的同步事务 → 同步调用必须给非零 */
	wr.read_size = (flags & TF_ONE_WAY) ? 0 : g_rcap;
	wr.read_buffer = (uintptr_t)g_rbuf;
	int rc;
	say("  · BC_TRANSACTION handle=%u code=%u data=%zu offs=%zu read_size=%zu", handle, code, (size_t)tr.data_size, (size_t)tr.offsets_size, (size_t)wr.read_size);
	if (ioctl(g_bfd, BINDER_WRITE_READ, &wr) < 0) {
		say("BINDER_WRITE_READ(txn code=%u) 失败: %s", code, strerror(errno));
		rc = -1;
	} else {
		wr_probe_len = wr.read_consumed;
		say("  · ioctl 完成 read_consumed=%zu", (size_t)wr.read_consumed);
		fprintf(stderr, "[bthci] RAW:");
		for (size_t i = 0; i < wr.read_consumed; i++)
			fprintf(stderr, " %02x", g_rbuf[i]);
		fprintf(stderr, "\n");
		rc = binder_consume(wr.read_consumed, rep);
		if (rc == -1 && !(flags & TF_ONE_WAY))
			rc = binder_wait_reply(rep); /* 回包可能排在下一轮 */
	}
	free(wbuf);
	if (flags & TF_ONE_WAY)
		return 0;
	return rc;
}

/* 等一次 incoming（主循环用；写侧空，只读） */
static int binder_poll(int timeout_ms)
{
	struct pollfd pf = { .fd = g_bfd, .events = POLLIN };
	if (poll(&pf, 1, timeout_ms) <= 0)
		return -1;
	struct binder_write_read wr;
	memset(&wr, 0, sizeof(wr));
	wr.write_size = 0;
	wr.read_size = g_rcap;
	wr.read_buffer = (uintptr_t)g_rbuf;
	if (ioctl(g_bfd, BINDER_WRITE_READ, &wr) < 0)
		return -1;
	return binder_consume(wr.read_consumed, NULL);
}

static int getservice(const char *name, uint32_t *handle)
{
	Parcel in;
	memset(&in, 0, sizeof(in));
	p_token(&in, DESC_SM);
	p_string16(&in, name);
	Parcel out;
	memset(&out, 0, sizeof(out));
	int rc = binder_call(0 /* context manager */, 2 /* getService=code */, &in, &out, 0);
	p_free(&in);
	if (rc < 0 || !out.b) {
		say("getService(%s) 失败 rc=%d", name, rc);
		return -1;
	}
	if (out.len < sizeof(struct flat_binder_object)) {
		say("getService 回包太短(%zu)，多半是没拿到 binder 对象", out.len);
		p_free(&out);
		return -1;
	}
	struct flat_binder_object *fbo = (struct flat_binder_object *)(out.b);
	if (fbo->hdr.type != BINDER_TYPE_HANDLE) {
		say("getService 回包 type=0x%x（不是 HANDLE）", fbo->hdr.type);
		p_free(&out);
		return -1;
	}
	*handle = fbo->handle;
	p_free(&out);
	say("getService(%s) → handle=%u", name, *handle);
	return 0;
}

/* ============================ 内核侧：pty + hci_uart(H4) ============================ */
static int attach_hci(void)
{
	int mfd = open("/dev/ptmx", O_RDWR | O_NOCTTY);
	if (mfd < 0)
		return -1;
	int n = 0;
	if (ioctl(mfd, TIOCGPTN, &n) < 0) {
		close(mfd);
		return -1;
	}
	char sp[64];
	snprintf(sp, sizeof(sp), "/dev/pts/%d", n);
	int lk = 0;
	ioctl(mfd, TIOCSPTLCK, &lk);
	int sfd = open(sp, O_RDWR | O_NOCTTY);
	if (sfd < 0) {
		close(mfd);
		return -1;
	}
	g_mfd = mfd;
	g_sfd = sfd;
	int disc = N_HCI;
	if (ioctl(sfd, TIOCSETD, &disc) < 0) {
		say("TIOCSETD N_HCI 失败: %s", strerror(errno));
		return -1;
	}
	int proto = HCI_UART_H4;
	if (ioctl(sfd, HCIUARTSETPROTO, proto) < 0)
		say("HCIUARTSETPROTO 失败: %s", strerror(errno));
	/* 本内核 legacy(tty 型) hci_uart 需要第二次 SETPROTO 才走 hci_uart_register_dev() */
	ioctl(sfd, HCIUARTSETPROTO, proto);
	int idx = -1;
	ioctl(sfd, HCIUARTGETDEVICE, &idx);
	say("hci 已 attach：index=%d pty=%s", idx, sp);
	return 0;
}

/* master 上是 H4 字节流，按 [type][依类型定长] 切帧 */
static void pump_from_kernel(void)
{
	static uint8_t acc[4096];
	static size_t acc_len = 0;
	ssize_t got = read(g_mfd, acc + acc_len, sizeof(acc) - acc_len);
	if (got <= 0)
		return;
	acc_len += (size_t)got;
	size_t pos = 0;
	while (pos + 1 < acc_len + 1 && pos < acc_len) {
		uint8_t type = acc[pos];
		size_t need;
		if (type == 0x01)
			need = pos + 4 + (pos + 3 < acc_len ? acc[pos + 3] : 0); /* cmd: type+op2+len+payload */
		else if (type == 0x02)
			need = pos + 5 + ((pos + 4 < acc_len) ? (acc[pos + 3] | (acc[pos + 4] << 8)) : 0);
		else if (type == 0x03)
			need = pos + 6 + ((pos + 5 < acc_len) ? acc[pos + 5] : 0);
		else {
			say("未知 H4 类型 0x%02x，丢字节", type);
			pos++;
			continue;
		}
		if (need > acc_len)
			break; /* 还没收全 */
		size_t flen = need - pos;
		uint8_t *pkt = acc + pos;
		pos = need;

		Parcel in;
		memset(&in, 0, sizeof(in));
		p_token(&in, DESC_HCI);
		if (g_include_type)
			p_bytes(&in, pkt, flen);
		else
			p_bytes(&in, pkt + 1, flen - 1);
		uint32_t code = (pkt[0] == 0x01) ? TC_SEND_CMD : (pkt[0] == 0x02) ? TC_SEND_ACL
							 : TC_SEND_SCO;
		binder_call(g_hal, code, &in, NULL, TF_ONE_WAY /* 这些在 AIDL 里是 oneway */);
		p_free(&in);
	}
	if (pos) {
		memmove(acc, acc + pos, acc_len - pos);
		acc_len -= pos;
	}
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"用法: %s [--ping|--probe] [--node <binderfs节点>] [--keep <秒>] [--no-type-byte]\n"
		"  --probe  只验 getService/initialize/enable 并打印回调\n",
		argv0);
}

int main(int argc, char **argv)
{
	int probe = 0, keep = 0, ping_only = 0, find_code = 0, find_hdr = 0, known = 0, sweep = 0;
	const char *node = "/dev/vndbinder";
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--probe"))
			probe = 1;
		else if (!strcmp(argv[i], "--ping"))
			ping_only = 1;
		else if (!strcmp(argv[i], "--find-code"))
			find_code = 1;
		else if (!strcmp(argv[i], "--find-header"))
			find_hdr = 1;
		else if (!strcmp(argv[i], "--known"))
			known = 1;
		else if (!strcmp(argv[i], "--sweep"))
			sweep = 1;
		else if (!strcmp(argv[i], "--node") && i + 1 < argc)
			node = argv[++i];
		else if (!strcmp(argv[i], "--keep") && i + 1 < argc)
			keep = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--no-type-byte"))
			g_include_type = 0;
		else {
			usage(argv[0]);
			return 2;
		}
	}
	signal(SIGINT, on_term);
	signal(SIGTERM, on_term);
	signal(SIGPIPE, SIG_IGN);

	/* 1) binder：先试 vndbinder，找不到服务再试 binder */
	uint32_t hal = 0;
	for (int t = 0; t < 2; t++) {
		const char *cand = t == 0 ? node : "/dev/binder";
		if (binder_open(cand) < 0)
			continue;
		if (sweep) {
			for (uint32_t code = 1; code <= 8; code++) {
				for (int v = -1; v <= 1; v++) {
					uint32_t h = 0;
					int rc = try_getservice(code, v, strstr(node, "vnd") ? FOURCC_VNDR : FOURCC_SYST, &h);
					(void)rc;
				}
			}
			return 0;
		}
		if (known) {
			static const char *names[] = { "bluetooth_manager", "activity", "package",
							     "android.hardware.bluetooth.IBluetoothHci/default",
							     "iservice_manager_selftest_nonexistent" };
			for (size_t k = 0; k < sizeof(names) / sizeof(names[0]); k++) {
				Parcel in;
				memset(&in, 0, sizeof(in));
				p_token(&in, DESC_SM);
				p_string16(&in, names[k]);
				Parcel rep;
				memset(&rep, 0, sizeof(rep));
				int rc = binder_call(0, 2, &in, &rep, 0);
				say("◆ node=%s name=%s rc=%d 回包=%zu", node, names[k], rc, rep.len);
				fprintf(stderr, "[bthci]   回包hex:");
				for (size_t i = 0; i < rep.len && i < 48; i++)
					fprintf(stderr, " %02x", rep.b[i]);
				fprintf(stderr, "\n");
				/* 在整条读流里找 flat_binder_object 的 type 特征 */
				for (size_t off = 0; off + 12 <= wr_probe_len; off += 4) {
					uint32_t t;
					memcpy(&t, g_rbuf + off, 4);
					if (t == BINDER_TYPE_HANDLE || t == BINDER_TYPE_BINDER || t == BINDER_TYPE_WEAK_HANDLE)
						say("   · 读流偏移 %zu 处有对象 type=0x%x", off, t);
				}
				p_free(&in);
				p_free(&rep);
			}
			return 0;
		}
		if (find_hdr) {
			uint32_t fh = 0;
			uint32_t fourcc = strstr(node, "vnd") ? FOURCC_VNDR : FOURCC_SYST;
			say("节点 %s → fourcc 候选 0x%08x", node, fourcc);
			for (uint32_t code = 2; code <= 2 && !fh; code++)
				for (int v = -1; v <= 21 && !fh; v++)
					if (try_getservice(code, v, fourcc, &fh) == 0)
						say("  ★★ 成功：code=%u variant=%d handle=%u", code, v, fh);
			if (!fh) {
				say("本节点不行，换下一个 binder 节点再试");
				close(g_bfd);
				g_bfd = -1;
				node = strstr(node, "vnd") ? "/dev/binder" : "/dev/vndbinder";
				if (binder_open(node) == 0) {
					fourcc = strstr(node, "vnd") ? FOURCC_VNDR : FOURCC_SYST;
					for (uint32_t code = 1; code <= 5 && !fh; code++)
						for (int v = -1; v <= 4 && !fh; v++)
							if (try_getservice(code, v, fourcc, &fh) == 0)
								say("  ★★ 成功：node=%s code=%u variant=%d handle=%u", node, code, v, fh);
				}
			}
			printf("RESULT_HANDLE=%u\n", fh);
			return fh ? 0 : 1;
		}
		if (find_code) {
			g_hal = 0;
			goto run_find;
		}
		if (getservice(SVC_HCI, &hal) == 0) {
			g_hal = hal;
			break;
		}
		close(g_bfd);
		g_bfd = -1;
		if (t == 0 && node && strcmp(node, "/dev/binder") == 0)
			break;
	}
	if (g_bfd < 0 || !g_hal) {
		say("✗ 拿不到 %s —— 确认 HAL 活着：getprop init.svc.vendor.bluetooth-aidl-qti", SVC_HCI);
		return 1;
	}

	if (ping_only)
		goto skip_attach;
run_find:
	if (attach_hci() < 0) {
		say("✗ hci attach 失败（需要 root？看 /sys/class/bluetooth）");
		return 1;
	}

skip_attach:
	/* 2) 做 HAL 的客户端：initialize(我们的回调 binder) → enable() */
	Parcel in;
	memset(&in, 0, sizeof(in));
	p_token(&in, DESC_CB);
	p_local_binder(&in);
	int rc = binder_call(g_hal, TC_INITIALIZE, &in, NULL, 0);
	p_free(&in);
	say("initialize → rc=%d", rc);

	Parcel e;
	memset(&e, 0, sizeof(e));
	p_token(&e, DESC_HCI);
	rc = binder_call(g_hal, TC_ENABLE, &e, NULL, 0);
	p_free(&e);
	say("enable → rc=%d", rc);

	if (find_code) {
		/* IServiceManager 的方法顺序各版本不同：逐个试，看哪个返回 binder 对象 */
		for (uint32_t code = 1; code <= 8; code++) {
			Parcel in;
			memset(&in, 0, sizeof(in));
			p_token(&in, DESC_SM);
			p_string16(&in, SVC_HCI);
			Parcel rep;
			memset(&rep, 0, sizeof(rep));
			int rc = binder_call(0, code, &in, &rep, 0);
			int obj = -1;
			if (rc == 0 && rep.b && rep.len >= 8) {
				struct flat_binder_object *f = (struct flat_binder_object *)(rep.b + (rep.len >= 24 ? 8 : 0));
				obj = (int)f->hdr.type;
			}
			say("code=%u rc=%d 回包=%zu 字节 首槽type=0x%x", code, rc, rep.len, obj);
			if (rc == 0 && rep.len >= 32) {
				for (size_t off = 0; off + 8 <= rep.len; off += 4) {
					struct flat_binder_object *f = (struct flat_binder_object *)(rep.b + off);
					if (f->hdr.type == BINDER_TYPE_HANDLE) {
						say("  ★ code=%u 在偏移 %zu 拿到 HANDLE=%u → getService 码就是 %u",
						    code, off, f->handle, code);
						g_hal = f->handle;
					}
				}
			}
			p_free(&in);
			p_free(&rep);
			if (g_hal)
				break;
		}
		if (!g_hal)
			say("✗ 1..8 号码都没拿到 binder 对象");
		close(g_bfd);
		return g_hal ? 0 : 1;
	}

	if (ping_only) {
		Parcel rep;
		memset(&rep, 0, sizeof(rep));
		int rc = binder_call(g_hal, TC_PING, NULL, &rep, 0);
		say("PING_TRANSACTION rc=%d", rc);
		dump("PING 回包", &rep);
		p_free(&rep);
		memset(&rep, 0, sizeof(rep));
		rc = binder_call(g_hal, TC_INTERFACE, NULL, &rep, 0);
		say("INTERFACE_TRANSACTION rc=%d", rc);
		dump("INTERFACE 回包", &rep);
		p_free(&rep);
		close(g_bfd);
		return 0;
	}

	if (probe) {
		say("probe 模式：跑 8 秒看回调/pump，然后 close+退场");
		keep = 8;
	}

	/* 3) 搬运循环：binder 与 pty master 一起 poll */
	time_t until = keep ? time(NULL) + keep : (time_t)0;
	while (g_run && (!until || time(NULL) < until)) {
		struct pollfd pf[2] = { { g_bfd, POLLIN, 0 }, { g_mfd, POLLIN, 0 } };
		int s = poll(pf, 2, 1000);
		if (s < 0) {
			if (errno == EINTR)
				break;
			break;
		}
		if (pf[0].revents & POLLIN) {
			while (binder_poll(0) >= 0)
				; /* 把已排好的回调读干净 */
		}
		if (pf[1].revents & POLLIN)
			pump_from_kernel();
	}

	/* 4) 体面退场：让 HAL 回到"无客户端"状态，安卓 framework 起来能直接重新绑定 */
	Parcel d;
	memset(&d, 0, sizeof(d));
	p_token(&d, DESC_HCI);
	say("disable → rc=%d", binder_call(g_hal, TC_DISABLE, &d, NULL, 0));
	Parcel c;
	memset(&c, 0, sizeof(c));
	p_token(&c, DESC_HCI);
	say("close → rc=%d", binder_call(g_hal, TC_CLOSE, &c, NULL, TF_ONE_WAY));
	p_free(&d);
	p_free(&c);
	int back = N_TTY;
	ioctl(g_sfd, TIOCSETD, &back);
	close(g_sfd);
	close(g_mfd);
	close(g_bfd);
	say("退场完成（hci 设备已随 tty 关闭自动注销）");
	return 0;
}
