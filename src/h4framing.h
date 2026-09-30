// h4framing.h — H4(HCI UART) 分包与"写全"逻辑，纯函数，可在宿主机直接单测。
//
// 为什么单独抽出来：桥里这两段逻辑（一次回调里可能带多个包 / pty 可能只吃掉半包）
// 是"内核 H4 解析器一旦跑偏就永久哑"的入口，必须先能离线断言，再上设备。
// 参考 HCI 4 号线的包格式（type 字节 + 各自头）：
//   0x01 Command : [t][op_lo][op_hi][plen][..]  总长 = 4 + plen
//   0x02 ACL     : [t][h_lo][h_hi][dl_lo][dl_hi][..] 总长 = 5 + dlen
//   0x03 SCO     : [t][h_lo][h_hi][len][..]    总长 = 4 + len
//   0x04 Event   : [t][code][plen][..]         总长 = 3 + plen
//   0x05 ISO     : 本机用不到；长度算法不确定 ⇒ **不许瞎拆**，整块原样写并报一次
#pragma once
#include <cstddef>
#include <cstdint>

namespace h4 {

enum class Kind {
  kWhole,     // 缓冲区里正好是一个完整包
  kMore,      // 头/体还不全，需要更多字节
  kMulti,     // 第一个包完整，后面还跟着别的包（必须拆开写）
  kBadType,   // 类型字节不认识
  kShort,     // 声称的长度比实际给的字节多（对端给的字节流已被截断）
};

// 从 b[0] 起这个包应有的总长（含类型字节）。len 未知时返回 0。
inline size_t frame_len(const uint8_t* b, size_t avail) {
  if (avail == 0) return 0;
  switch (b[0]) {
    case 0x01: return avail < 4 ? 0 : 4 + static_cast<size_t>(b[3]);
    case 0x02: return avail < 5 ? 0 : 5 + static_cast<size_t>(b[3] | (b[4] << 8));
    case 0x03: return avail < 4 ? 0 : 4 + static_cast<size_t>(b[3]);
    case 0x04: return avail < 3 ? 0 : 3 + static_cast<size_t>(b[2]);
    case 0x05: return 0;  // ISO：不拆（见文件头），返回 0 让调用方按整块处理
    default:   return static_cast<size_t>(-1);
  }
}

// 判断并拆分：out_len 填"这次该怎么处理的那一段"的长度。
// 约定：算不出长度（头不全 / ISO）时**整块原样**交给调用方写，绝不瞎猜长度——
// 猜错长度正是把内核 H4 解析器带跑的路径。
inline Kind classify(const uint8_t* buf, size_t n, size_t* out_len) {
  *out_len = 0;
  if (n == 0) return Kind::kMore;
  size_t need = frame_len(buf, n);
  if (need == static_cast<size_t>(-1)) return Kind::kBadType;
  if (need == 0) { *out_len = n; return Kind::kWhole; }   // 头不全或 ISO：整块原样
  if (need > n) { *out_len = need; return Kind::kShort; }  // 声称更长却没给够 ⇒ 上游已截断
  *out_len = need;
  return need < n ? Kind::kMulti : Kind::kWhole;           // 后面还跟着字节 ⇒ 一次回调多个包
}

// 丢弃策略（关键，别搞反）：**只有在这一个字节都还没写出去时**才允许丢弃整包。
// 一旦包写到一半就放弃，残余的半包同样会把内核 H4 解析器带偏 —— 那正是我们要防的事故，
// 所以"写不进就丢"是错的；写了一半就必须把它写完（哪怕要等）。
// elapsed/off 由调用方实测；deadline 只约束"还没开始写"的阶段。
inline bool may_drop(size_t off, long long elapsed_ms, long long deadline_ms) {
  return off == 0 && elapsed_ms >= deadline_ms;
}

// 写不进且已写过一部分时，等多久再回头看一次（不是放弃，只是别让循环空转）。
inline long long poll_slice_ms(long long elapsed_ms) {
  (void)elapsed_ms;
  return 200;
}

// 写全的纯逻辑：喂进一个"模拟一次 write 的返回值"的回调，返回要不要继续。
// wcb(buf, len) 返回本次写掉的字节数（<len 即短写；0 表示暂时写不进，由 timed_out 决定放弃）。
// 返回：实际写掉的总字节数；若中途放弃则 < len（调用方必须据此记 drop，绝不能当成功）。
template <class WriteFn>
inline size_t write_full(const uint8_t* buf, size_t len, size_t* off, WriteFn wcb, bool timed_out) {
  size_t done = *off;
  while (done < len) {
    if (timed_out) { *off = done; return done; }
    size_t w = wcb(buf + done, len - done);
    if (w == 0) { *off = done; return done; }   // 写不进：调用方决定 poll/放弃
    done += w;
  }
  *off = done;
  return done;
}

}  // namespace h4
