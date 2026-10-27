#!/usr/bin/env bash
# ============================================================
# 海康 MVS SDK 安装脚本 (Ubuntu / Debian, x86_64 / aarch64)
#
# 用法:
#   sudo bash scripts/install_mvs.sh                  # 自动找安装包，系统级安装到 /opt/MVS
#   sudo bash scripts/install_mvs.sh ~/下载/MVS.deb   # 指定安装包，系统级安装
#   bash scripts/install_mvs.sh --user                # 免 root，解压到 ~/.local/opt/MVS
#   bash scripts/install_mvs.sh --user MVS.deb
#
# 说明:
#   - MVS 官方 deb 默认安装位置就是 /opt/MVS（头文件 include/，库 lib/64/）
#   - --user 模式不解压到 /opt，因此编译时要指定 -DMVS_ROOT=<解压目录>
# ============================================================
set -euo pipefail

MODE="system"
PKG_ARG=""

usage() {
    cat <<'EOF'
海康 MVS SDK 安装脚本 (Ubuntu / Debian, x86_64 / aarch64)

用法:
  sudo bash scripts/install_mvs.sh                  # 自动找安装包, 系统级安装到 /opt/MVS
  sudo bash scripts/install_mvs.sh ~/下载/MVS.deb   # 指定安装包, 系统级安装
  bash scripts/install_mvs.sh --user                # 免 root, 解压到 ~/.local/opt/MVS
  bash scripts/install_mvs.sh --user MVS.deb        # 免 root + 指定安装包

说明:
  - MVS 官方 deb 默认安装位置就是 /opt/MVS (头文件 include/, 库 lib/64/)
  - --user 模式不写 /opt, 编译时需指定 -DMVS_ROOT=<解压目录>
EOF
}

for arg in "$@"; do
    case "$arg" in
        --user) MODE="user" ;;
        -h|--help) usage; exit 0 ;;
        *) PKG_ARG="$arg" ;;
    esac
done

log()  { printf '\033[1;32m[MVS]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[MVS]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[MVS]\033[0m %s\n' "$*" >&2; exit 1; }

# sudo 下 $HOME 会变成 /root，需要用调用者的家目录
if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ] && [ "${SUDO_USER}" != "root" ]; then
    USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    USER_HOME="$HOME"
fi

# ---------- 1. 找到安装包 ----------
if [ -z "$PKG_ARG" ]; then
    log "未指定安装包，自动在下载目录里查找最新的 MVS 包 ..."
    PKG_ARG=$(find "$USER_HOME/下载" "$USER_HOME/Downloads" "$USER_HOME/桌面" "$USER_HOME/Desktop" /tmp -maxdepth 3 \
                   \( -iname 'MVS*.deb' -o -iname 'MVS*.zip' -o -iname 'MVS*.tar.gz' \) -printf '%T@ %p\n' 2>/dev/null \
              | sort -n | tail -1 | cut -d' ' -f2- || true)
    [ -n "$PKG_ARG" ] || die "没找到安装包。请先从 https://www.hikrobotics.com/cn/machinevision/service/download?module=0 下载“机器视觉工业相机客户端MVS (Linux)(x86_64)”，或把路径作为参数传入。"
fi

[ -f "$PKG_ARG" ] || die "安装包不存在: $PKG_ARG"
PKG_ARG=$(readlink -f "$PKG_ARG")
log "安装包: $PKG_ARG"

# ---------- 2. 解出 deb（官方包可能是 deb / zip / tar.gz） ----------
WORKDIR=$(mktemp -d)
trap 'rm -rf "$WORKDIR"' EXIT
DEB=""
SDK_SRC=""
ALREADY_INSTALLED=0

case "$PKG_ARG" in
    *.deb)
        DEB="$PKG_ARG"
        ;;
    *.zip)
        command -v unzip >/dev/null 2>&1 || die "需要 unzip，请先安装: sudo apt install unzip"
        log "zip 解包中 ..."
        unzip -q -o "$PKG_ARG" -d "$WORKDIR/unzip"
        DEB=$(find "$WORKDIR/unzip" -maxdepth 2 -iname '*.deb' | head -1 || true)
        if [ -z "$DEB" ]; then
            tgz=$(find "$WORKDIR/unzip" -maxdepth 2 -iname '*.tar.gz' | head -1 || true)
            [ -n "$tgz" ] && tar -xzf "$tgz" -C "$WORKDIR/unzip"
        fi
        ;;
    *.tar.gz|*.tgz)
        log "tar.gz 解包中 ..."
        tar -xzf "$PKG_ARG" -C "$WORKDIR"
        DEB=$(find "$WORKDIR" -maxdepth 3 -iname '*.deb' | head -1 || true)
        ;;
    *)
        die "不支持的安装包格式: $PKG_ARG（支持 .deb / .zip / .tar.gz）"
        ;;
esac

# 没有 deb 时，按「目录里含 include/」的方式识别 SDK 根目录（.tar.gz + setup.sh 的情况）
if [ -z "$DEB" ]; then
    SDK_SRC=$(find "$WORKDIR" -maxdepth 5 -type f -name 'MvCameraControl.h' -printf '%h\n' 2>/dev/null | head -1)
    [ -n "$SDK_SRC" ] && SDK_SRC=$(dirname "$SDK_SRC")
    if [ -n "$SDK_SRC" ] && [ -d "$SDK_SRC/lib" ]; then
        warn "压缩包里没有 deb，直接按 SDK 目录处理: $SDK_SRC"
    else
        # 兜底：找 setup.sh 并交给官方脚本
        SETUP=$(find "$WORKDIR" -maxdepth 4 -name 'setup.sh' | head -1 || true)
        if [ -n "$SETUP" ]; then
            log "使用官方 setup.sh: $SETUP"
            if [ "$MODE" = "system" ] && [ "$(id -u)" -eq 0 ]; then
                (cd "$(dirname "$SETUP")" && bash setup.sh)
                SDK_ROOT="/opt/MVS"
                DEB=""
                ALREADY_INSTALLED=1
            else
                die "官方 setup.sh 模式需要 root，请执行: sudo bash $SETUP"
            fi
        else
            die "无法从安装包中识别 deb 或 SDK 目录结构，请检查文件是否完整"
        fi
    fi
fi

# ---------- 3. 安装 ----------
if [ "$ALREADY_INSTALLED" = "1" ]; then
    log "已通过官方 setup.sh 完成安装，跳过 dpkg"
elif [ "$MODE" = "system" ]; then
    [ "$(id -u)" -eq 0 ] || die "系统级安装需要 root，请用: sudo bash scripts/install_mvs.sh \"$PKG_ARG\""

    if [ -n "$DEB" ]; then
        log "dpkg 安装 $DEB ..."
        if ! dpkg -i "$DEB"; then
            warn "dpkg 报告依赖问题，尝试自动补齐 ..."
            apt-get update -qq || true
            apt-get install -f -y
        fi
    else
        # 压缩包里直接是 SDK 目录：拷到 /opt/MVS
        log "复制 SDK 到 /opt/MVS ..."
        mkdir -p /opt/MVS
        cp -a "$SDK_SRC/." /opt/MVS/
    fi

    SDK_ROOT="/opt/MVS"
else
    # 免 root：解压到用户目录
    SDK_ROOT="$USER_HOME/.local/opt/MVS"
    log "解压到 $SDK_ROOT ..."
    rm -rf "$SDK_ROOT"
    mkdir -p "$SDK_ROOT"
    if [ -n "$DEB" ]; then
        dpkg-deb -x "$DEB" "$WORKDIR/extract"
        if [ -d "$WORKDIR/extract/opt/MVS" ]; then
            rm -rf "$SDK_ROOT"
            mv "$WORKDIR/extract/opt/MVS" "$SDK_ROOT"
        else
            # 官方 deb 是包装包：真正内容在 opt/MVClient_Install_Temp/MVS.tar.gz
            inner=$(find "$WORKDIR/extract" -maxdepth 3 -name 'MVS.tar.gz' | head -1 || true)
            if [ -n "$inner" ]; then
                log "从包装包中解出 $(basename "$inner") ..."
                tar -xzf "$inner" -C "$SDK_ROOT"
            else
                src=$(find "$WORKDIR/extract" -maxdepth 6 -name 'MvCameraControl.h' -printf '%h\n' | head -1 || true)
                [ -n "$src" ] || die "无法从 deb 中定位 SDK 内容（既没有 opt/MVS，也没有 MVS.tar.gz）"
                cp -a "$(dirname "$src")/." "$SDK_ROOT/"
            fi
        fi
    else
        cp -a "$SDK_SRC/." "$SDK_ROOT/"
    fi
fi

# ---------- 4. 刷新动态库缓存并校验 ----------
if [ "$MODE" = "system" ] && command -v ldconfig >/dev/null 2>&1; then
    ldconfig || warn "ldconfig 执行失败，可能需要在运行时设置 LD_LIBRARY_PATH"
fi

LIB=""
for d in "$SDK_ROOT/lib/64" "$SDK_ROOT/lib/x64" "$SDK_ROOT/lib/arm64" "$SDK_ROOT/lib/aarch64"; do
    if [ -e "$d/libMvCameraControl.so" ]; then LIB="$d"; break; fi
done

HDR="$SDK_ROOT/include/MvCameraControl.h"

echo
[ -f "$HDR" ] && log "头文件 OK : $HDR" || die "缺少头文件: $HDR"
[ -n "$LIB" ] && log "动态库 OK : $LIB/libMvCameraControl.so" || die "缺少动态库 libMvCameraControl.so（已查找 $SDK_ROOT/lib/*）"

if command -v nm >/dev/null 2>&1; then
    nm -D --defined-only "$LIB/libMvCameraControl.so" 2>/dev/null | grep -q "MV_CC_Initialize" \
        && log "符号检查 OK: MV_CC_Initialize" \
        || warn "没找到 MV_CC_Initialize 符号，请确认 SDK 版本"
fi

echo
log "安装完成，SDK 根目录: $SDK_ROOT"
if [ "$MODE" = "system" ]; then
    echo "  重新配置工程即可:  cd \"$(dirname "$(readlink -f "$0")")/..\" && cmake -S . -B build && cmake --build build"
else
    echo "  配置工程:  cmake -S . -B build -DMVS_ROOT=\"$SDK_ROOT\""
    echo "  运行时如报找不到库: export LD_LIBRARY_PATH=\"$SDK_ROOT/lib/64:\$LD_LIBRARY_PATH\""
fi
echo "  提示: USB 相机若枚举不到设备，需要 root 或 udev 规则（MVS 安装时通常会一并配置）。"
