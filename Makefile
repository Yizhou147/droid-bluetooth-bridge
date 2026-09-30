# 宿主机侧：只跑与设备无关的纯逻辑/真 pty 行为测试。
# 设备用的二进制必须 NDK 交叉编译 ⇒ 走 build.sh / CI，不放进默认目标。
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Isrc
T := /tmp/btb-tests

.PHONY: test ci clean
test: $(T)/h4framing_test $(T)/pty_write_policy_test
	@$(T)/h4framing_test && $(T)/pty_write_policy_test
	@echo "全部单测通过"

$(T)/h4framing_test: tests/h4framing_test.cpp src/h4framing.h | $(T)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(T)/pty_write_policy_test: tests/pty_write_policy_test.cpp src/h4framing.h | $(T)
	$(CXX) $(CXXFLAGS) -o $@ $< -lutil

$(T):
	mkdir -p $(T)

# CI 里调用：静态检查 + 单测，不碰 NDK 目标
ci: test

clean:
	rm -rf $(T)
