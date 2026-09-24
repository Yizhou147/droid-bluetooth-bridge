/* m1-probe.c — M1 承重假设验证：不装任何内核模块，仅用已加载的 hci_uart(H4)
 * 经一对 pty 在共享内核里注册出真 hci0。
 *
 * 做 hciattach 的活：open ptmx → 拿 slave → TIOCSETD N_HCI → HCIUARTSETPROTO H4
 * → 之后把 master fd 当 HCI 字节流收发（本探针只回 HCI_Reset 的 Command_Complete，
 *   够让内核把设备注册出来并进入 down/up 流程）。
 *
 * 退出（Ctrl-C / SIGTERM / --timeout 到点）→ close(slave) → ldisc 析构 → hci_unregister_dev。
 *
 * 编译：gcc -static -O2 -o m1-probe m1-probe.c   （跑在安卓侧，需 root）
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/select.h>
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
#ifndef TIOCGPTN
#define TIOCGPTN 0x80045430
#endif
#ifndef TIOCSPTLCK
#define TIOCSPTLCK 0x40045431
#endif
#ifndef HCIUARTSETPROTO
#define HCIUARTSETPROTO 0x400455C8 /* _IOW('U', 200, int) */
#endif
#ifndef HCIUARTGETPROTO
#define HCIUARTGETPROTO 0x800455C9 /* _IOR('U', 201, int) */
#endif
#ifndef HCIUARTGETDEVICE
#define HCIUARTGETDEVICE 0x800455CA /* _IOR('U', 202, int) */
#endif
#define HCI_UART_H4 0

static volatile sig_atomic_t g_run = 1;
static void on_term(int sig) { (void)sig; g_run = 0; }

static void say(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	fprintf(stderr, "[m1-probe] ");
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	fflush(stderr);
}

/* master fd 上收到内核发来的 H4 帧，回一个最小合法应答（只覆盖 HCI_Reset 等常见命令） */
static void answer(int mfd, unsigned char *pkt, int len)
{
	/* H4: [type=0x01][opcode_lo][opcode_hi][plen][params...] */
	if (len < 4 || pkt[0] != 0x01)
		return;
	unsigned op = pkt[1] | (pkt[2] << 8);
	unsigned char cc[276];
	int n = 0;
	cc[n++] = 0x0e; /* HCI packet type: Command Complete */
	cc[n++] = 1;   /* parameter total length placeholder, fixed below */
	cc[n++] = op & 0xff;
	cc[n++] = op >> 8;
	cc[n++] = 0x00; /* status: success */
	int body = 0;

	switch (op) {
	case 0x1001: /* Read_Local_Version_Information */
		/* ver=0x0b(5.1) rev sub manuf lmp_ver lmp_sub  → 9 bytes 之后按内核容忍度给零 */
		memset(cc + n, 0, 9);
		n += 9;
		body = 9;
		break;
	case 0x1003: /* Read_Local_Extended_Features */
	case 0x1009: /* Read_BD_ADDR */
		memset(cc + n, 0, 6);
		n += 6;
		body = 6;
		break;
	case 0x1005: /* Read_Buffer_Size */
		cc[n++] = 0xfc; cc[n++] = 0x00; /* pkt_type */
		cc[n++] = 40;   cc[n++] = 0;      /* ACL MTU 40（够 HID） */
		cc[n++] = 0;    cc[n++] = 0;      /* SCO MTU */
		cc[n++] = 16;   cc[n++] = 1;      /* max ACL / SCO pkts */
		body = 8;
		break;
	default:
		body = 0; /* HCI_Reset 等：只回 status，1 字节参数 */
		cc[1] = 1;
		if (op != 0x0c03)
			say("cmd 0x%04x → 通用 Command_Complete(status=0)", op);
		break;
	}
	if (body)
		cc[1] = 1 + body;
	if (write(mfd, cc, n) < 0)
		say("write 应答失败: %s", strerror(errno));
}

int main(int argc, char **argv)
{
	int timeout = argc > 1 ? atoi(argv[1]) : 15;
	int mfd = open("/dev/ptmx", O_RDWR | O_NOCTTY);
	if (mfd < 0) {
		say("open /dev/ptmx 失败: %s", strerror(errno));
		return 1;
	}
	int n = 0;
	if (ioctl(mfd, TIOCGPTN, &n) < 0) {
		say("TIOCGPTN 失败: %s", strerror(errno));
		return 1;
	}
	char slave_path[64];
	snprintf(slave_path, sizeof(slave_path), "/dev/pts/%d", n);
	int lk = 0;
	ioctl(mfd, TIOCSPTLCK, &lk);
	int sfd = open(slave_path, O_RDWR | O_NOCTTY);
	if (sfd < 0) {
		say("open %s 失败: %s", slave_path, strerror(errno));
		return 1;
	}
	say("pty: master=%d slave=%s(%d)", mfd, slave_path, sfd);

	int disc = N_HCI;
	if (ioctl(sfd, TIOCSETD, &disc) < 0) {
		say("TIOCSETD N_HCI 失败: %s  ← 关键判定点", strerror(errno));
		return 1;
	}
	say("TIOCSETD N_HCI OK");

	int proto = HCI_UART_H4;
	int rc1 = ioctl(sfd, HCIUARTSETPROTO, proto);
	say("第 1 次 HCIUARTSETPROTO(H4) rc=%d errno=%d(%s)", rc1, errno, strerror(errno));
	int got = -1, idx = -1;
	int e1 = ioctl(sfd, HCIUARTGETPROTO, &got);
	say("  GETPROTO rc=%d proto=%d errno=%s", e1, got, strerror(errno));
	/* 本内核的 legacy（tty 型）hci_uart 要第二次 SETPROTO 才走到 hci_uart_register_dev()
	 * （hci_uart_tty_ioctl: if (!test_and_set_bit(PROTO_SET)) set_proto() else if (has_legacy_proto()) register_dev()） */
	int rc2 = ioctl(sfd, HCIUARTSETPROTO, proto);
	say("第 2 次 HCIUARTSETPROTO(H4) rc=%d errno=%s", rc2, strerror(errno));
	got = -1;
	e1 = ioctl(sfd, HCIUARTGETPROTO, &got);
	int e2 = ioctl(sfd, HCIUARTGETDEVICE, &idx);
	say("  GETPROTO rc=%d proto=%d | GETDEVICE rc=%d index=%d errno=%s",
	    e1, got, e2, idx, strerror(errno));
	if (e2 == 0 && idx >= 0)
		say("★ 内核已注册 adapter：hci%d ← 零新内核模块路线成立", idx);
	else
		say("✗ 仍未注册，看 /sys/class/bluetooth 确认");
	DIR *dp = opendir("/sys/class/bluetooth");
	if (dp) {
		struct dirent *de;
		say("/sys/class/bluetooth:");
		while ((de = readdir(dp)))
			if (de->d_name[0] != '.')
				say("   %s", de->d_name);
		closedir(dp);
	} else {
		say("opendir /sys/class/bluetooth 失败: %s", strerror(errno));
	}

	struct termios tio;
	if (tcgetattr(sfd, &tio) == 0) {
		tio.c_lflag &= ~(ICANON | ECHO);
		tio.c_iflag = 0;
		tio.c_oflag = 0;
		tcsetattr(sfd, TCSANOW, &tio);
	}

	signal(SIGINT, on_term);
	signal(SIGTERM, on_term);

	/* 心跳：定期从 master 读（内核会主动发 HCI_Reset / 各种 read 命令），并回最小应答 */
	for (time_t end = time(NULL) + timeout; g_run && time(NULL) < end;) {
		fd_set r;
		FD_ZERO(&r);
		FD_SET(mfd, &r);
		struct timeval tv = { 1, 0 };
		int s = select(mfd + 1, &r, NULL, NULL, &tv);
		if (s <= 0)
			continue;
		unsigned char buf[1024];
		int len = read(mfd, buf, sizeof(buf));
		if (len > 0)
			answer(mfd, buf, len);
	}

	int back = N_TTY;
	ioctl(sfd, TIOCSETD, &back);
	close(sfd);
	close(mfd);
	say("退出：ldisc 已还原，hci 设备随之注销");
	return 0;
}
