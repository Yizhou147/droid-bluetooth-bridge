// pty_write_policy_test.cpp — 在**真 pty**上验证丢弃策略的前提是否成立。
// 要回答的问题是这台内核给的答案，不是我以为的答案：
//   Q1 非阻塞 pty 写满时，write() 会不会返回短计数（=会往流里留半包）？还是只会 EAGAIN？
//   Q2 已经写进去的字节，读者开始排空后能不能续写完（策略要求"半包必须写完"）？
//   Q3 may_drop 在"半包"情形下必须永远返回 false（否则就等于我们主动制造跑偏）。
#include "h4framing.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <pty.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...)                                                        \
  do {                                                                          \
    if (cond) { ++g_pass; }                                                     \
    else { ++g_fail; printf("  FAIL %s:%d ", __FILE__, __LINE__);              \
           printf(__VA_ARGS__); printf("\n"); }                                 \
  } while (0)

int main() {
  int m = -1, s = -1;
  if (openpty(&m, &s, nullptr, nullptr, nullptr) != 0) {
    printf("openpty 失败: %s\n（容器里没有 pts 就跳过本测试）\n", strerror(errno));
    return 77;   // 77 = skip，让 make test 能识别
  }
  int fl = fcntl(m, F_GETFL, 0);
  fcntl(m, F_SETFL, fl | O_NONBLOCK);          // 和桥里一样：master 非阻塞
  unsigned char pkt[512];
  memset(pkt, 0x55, sizeof(pkt));

  // ① 一直写直到写不进，记录有没有出现过"短写"
  long long total = 0;
  int partial = 0, agains = 0;
  for (int i = 0; i < 2000; i++) {
    errno = 0;
    ssize_t r = write(m, pkt, sizeof(pkt));
    if (r > 0) {
      total += r;
      if (r < (ssize_t)sizeof(pkt)) partial++;
      continue;
    }
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { agains++; break; }
    printf("write 意外错误: %s\n", strerror(errno));
    return 1;
  }
  printf("  实测：写进 %lld 字节后 EAGAIN；短写次数=%d EAGAIN=%d\n", total, partial, agains);
  CHECK(agains > 0, "缓冲区总会写满，应出现 EAGAIN");
  // Q1 的结论按实测记录，不预设：短写=0 说明这台内核在 pty 上不会返回部分写入
  //（那"整包丢弃"分支主要防的是超时而不是短写）；短写>0 说明半包风险真实存在。
  // 两种情况都必须靠"半包不丢"这条策略兜住，所以这里只记录、不断言。

  // ② Q3：模拟"已经写了一半才遇到 EAGAIN"的时刻 —— may_drop 必须拒绝丢弃
  CHECK(h4::may_drop(100, 60000, 1000) == false,
        "已写 100 字节时无论如何都不许丢（丢了就是留半包）");
  CHECK(h4::may_drop(0, 1000, 1000) == true, "一个字节都没写出去且已到时限，允许丢整包");
  CHECK(h4::may_drop(0, 999, 1000) == false, "未到时限不该丢");

  // ③ Q2：读者开始排空后，半途的包必须能续写完（用和 writeFrame 相同的循环骨架）
  size_t off = 0;
  // 先制造一个"写到一半"的状态：写满到 EAGAIN
  size_t left = sizeof(pkt);
  while (left) {
    ssize_t r = write(m, pkt + off, left);
    if (r > 0) { off += r; left -= (size_t)r; continue; }
    if (errno == EINTR) continue;
    break;                      // EAGAIN：停在"可能写了一半"的位置
  }
  bool partial_state = off > 0 && off < sizeof(pkt);
  printf("  停在 off=%zu/%zu（是否半包=%s）\n", off, sizeof(pkt),
         partial_state ? "是" : "否（正好边界或已写满缓冲前未开写）");
  int w = 0;
  while (off < sizeof(pkt) && w++ < 500) {
    struct pollfd pf{m, POLLOUT, 0};
    poll(&pf, 1, 10);
    ssize_t r = write(m, pkt + off, sizeof(pkt) - off);
    if (r > 0) off += (size_t)r;
  }
  CHECK(off == sizeof(pkt), "半包必须能续写完：最终 off=%zu/%zu", off, sizeof(pkt));

  close(m);
  close(s);
  printf("pty-write-policy: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
