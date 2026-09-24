#!/bin/sh
# 用 NDK 交叉编译 bthci-bridge。本机（arm64 Linux）没有 NDK → 实际由 CI 构建；
# 若在 x86_64 机器上有 NDK，直接 sh build.sh 也能出同样的产物。
set -eu

NDK="${NDK:-${ANDROID_NDK_HOME:-${ANDROID_NDK:-}}}"
if [ -z "$NDK" ]; then
    echo "需要 NDK：export NDK=/path/to/android-ndk-r27（或设 ANDROID_NDK_HOME）" >&2
    exit 1
fi

OS=$(uname -s | tr '[:upper:]' '[:lower:]')
case "$OS" in
    mingw*|msys*|windows*) HOST_TAG="windows-x86_64" ;;
    darwin)                HOST_TAG="darwin-x86_64" ;;
    *)                     HOST_TAG="linux-x86_64" ;;
esac

# 不链 -lbinder_ndk：runner 那份 NDK 的 stub 不导出 AServiceManager_*/ABinderProcess_*，
# 改成运行时 dlopen + dlsym（设备上的 /system/lib64/libbinder_ndk.so 是全的）。
# API 33：AServiceManager_getService 需要 31+
CC="$NDK/toolchains/llvm/prebuilt/$HOST_TAG/bin/aarch64-linux-android33-clang++"
[ -x "$CC" ] || { echo "找不到编译器：$CC" >&2; exit 1; }

mkdir -p out
"$CC" -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter \
    -fPIE -pie -static-libstdc++ \
    src/bthci-bridge.cpp -o out/bthci-bridge \
    -llog

ls -l out/bthci-bridge
