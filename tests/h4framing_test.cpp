// h4framing_test.cpp — 宿主机单测（g++ 直接编，不需要 NDK/设备）。
// 覆盖：单包 / 一次回调多包 / 头不全 / 声称长度超过实给字节 / 未知类型 / ISO 不拆 /
//       write_full 的短写补写与超时放弃。
// 编译运行：make test   （或 g++ -std=c++17 -Isrc -o /tmp/h4t tests/h4framing_test.cpp && /tmp/h4t）
#include "h4framing.h"

#include <cstdio>
#include <vector>

static int g_fail = 0, g_pass = 0;

#define CHECK(cond, ...)                                                        \
  do {                                                                          \
    if (cond) { ++g_pass; }                                                     \
    else { ++g_fail; printf("  FAIL %s:%d ", __FILE__, __LINE__);              \
           printf(__VA_ARGS__); printf("\n"); }                                 \
  } while (0)

using namespace h4;

int main() {
  // ① 单个事件包：[04][code][plen][..plen 字节] → 总长 3+plen
  {
    std::vector<uint8_t> b = {0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kWhole && len == 7,
          "单事件包应 kWhole/7，得 len=%zu", len);
  }
  // ② 一次回调里两个事件包 → kMulti，且第一段正好是第一个包
  {
    std::vector<uint8_t> b = {
        0x04, 0x0e, 0x01, 0x00,             // 包1：3+1 = 4
        0x04, 0x05, 0x02, 0xaa, 0xbb,       // 包2：3+2 = 5
    };
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kMulti && len == 4,
          "多包应 kMulti/第一段 4，得 kind/len=%zu", len);
    // 调用方按第一段写完后，剩下的字节要能继续拆出第二个包
    size_t len2 = 0;
    CHECK(classify(b.data() + 4, b.size() - 4, &len2) == Kind::kWhole && len2 == 5,
          "第二段应 kWhole/5，得 %zu", len2);
  }
  // ③ 命令包 [01][op_lo][op_hi][plen][..] → 4+plen
  {
    std::vector<uint8_t> b = {0x01, 0x03, 0x0c, 0x00};   // HCI_Reset(opcode 0x0c03, plen 0)
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kWhole && len == 4,
          "命令包应 kWhole/4，得 %zu", len);
  }
  // ④ ACL 包 → 5 + dlen（dlen 小端两字节）
  {
    std::vector<uint8_t> b(5 + 0x0102);
    b[0] = 0x02; b[1] = 0x40; b[2] = 0x20; b[3] = 0x02; b[4] = 0x01;  // dlen=0x0102
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kWhole && len == 5 + 0x102,
          "ACL 应 kWhole/%d，得 %zu", 5 + 0x102, len);
  }
  // ⑤ SCO 包 → 4 + len
  {
    std::vector<uint8_t> b = {0x03, 0x01, 0x00, 0x03, 0x11, 0x22, 0x33};
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kWhole && len == 7,
          "SCO 应 kWhole/7，得 %zu", len);
  }
  // ⑥ 头还不全（事件只给了 2 字节）→ 不猜长度，整块原样，等下一次回调再判
  {
    std::vector<uint8_t> b = {0x04, 0x05};
    size_t len = 0;
    Kind k = classify(b.data(), b.size(), &len);
    CHECK(k == Kind::kWhole && len == 2, "头不全应整块原样(kWhole/2)，得 kind=%d len=%zu",
          (int)k, len);
  }
  // ⑦ 声称长度大于实给字节 = 上游已经截断 ⇒ 必须报 kShort，绝不能写半包了事
  {
    std::vector<uint8_t> b = {0x04, 0x0e, 0x10, 0x01, 0x02};  // plen=0x10 却只给 5 字节
    size_t len = 0;
    Kind k = classify(b.data(), b.size(), &len);
    CHECK(k == Kind::kShort, "应 kShort（截断）");
    // out_len 必须给出"它声称有多长"，否则日志会打成"声称 0 字节"这种谎话
    CHECK(len == 3 + 0x10, "kShort 也要回报声称长度(19)，得 %zu", len);
  }
  // ⑧ 未知类型字节
  {
    std::vector<uint8_t> b = {0x09, 0x00, 0x00};
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kBadType, "未知类型应 kBadType");
  }
  // ⑨ ISO(0x05) 不拆：本机用不到、长度算法未核实 ⇒ 整块原样，绝不瞎拆
  {
    std::vector<uint8_t> b = {0x05, 1, 2, 3, 4, 5, 6, 7, 8};
    size_t len = 0;
    CHECK(classify(b.data(), b.size(), &len) == Kind::kWhole && len == b.size(),
          "ISO 应整块原样，得 len=%zu", len);
  }
  // ⑩ write_full：短写要补写；超时未写满必须返回短于 len（调用方据此记 drop）
  {
    std::vector<uint8_t> b = {1, 2, 3, 4, 5, 6, 7, 8};
    size_t off = 0, calls = 0;
    // 每次只吃掉 3 字节（模拟短写）
    size_t done = write_full(b.data(), b.size(), &off, [&](const uint8_t*, size_t n) {
      ++calls;
      return n < 3 ? n : 3;
    }, false);
    CHECK(done == 8 && calls == 3, "短写应补写满 8 字节/3 次调用，得 done=%zu calls=%zu",
          done, calls);

    // 写不进（返回 0）且已超时 ⇒ 立刻放弃并返回已写字节数，不无限重试
    off = 0; calls = 0;
    done = write_full(b.data(), b.size(), &off, [&](const uint8_t*, size_t) {
      ++calls;
      return (size_t)0;
    }, true);
    CHECK(done == 0 && calls == 0, "超时放弃应 done=0/不调用，得 done=%zu calls=%zu",
          done, calls);

    // 写不进但未超时：只试一次就返回，交给调用方 poll（绝不 busy-spin）
    off = 0; calls = 0;
    done = write_full(b.data(), b.size(), &off, [&](const uint8_t*, size_t) {
      ++calls;
      return (size_t)0;
    }, false);
    CHECK(done == 0 && calls == 1, "EAGAIN 应只试 1 次后返回，得 calls=%zu", calls);
  }
  printf("h4framing: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
