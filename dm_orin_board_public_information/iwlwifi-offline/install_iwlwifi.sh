#!/bin/bash
# iwlwifi 离线安装
# 开发者：DaMiao/达妙智控
# 用法：sudo ./install_iwlwifi.sh

set -e  # 遇到错误立即退出

echo "========================================"
echo "    iwlwifi 驱动离线安装脚本"
echo "    iwlwifi Driver Offline Installation Script"
echo "========================================"
echo "系统信息 / System: $(lsb_release -d | cut -f2)"
echo "内核版本 / Kernel: $(uname -r)"
echo "架构 / Architecture: $(uname -m)"
echo "========================================"
echo ""

# 检查 root 权限
if [ "$EUID" -ne 0 ]; then 
    echo "错误：请使用 sudo 运行此脚本"
    echo "Error: Please run this script with sudo"
    echo "用法 / Usage: sudo ./install_iwlwifi.sh"
    exit 1
fi

# 定义颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查包文件是否存在
check_packages() {
    local packages=(
        "cpp-12*.deb"
        "libasan8*.deb"
        "libtsan2*.deb"
        "libgcc-12-dev*.deb"
        "gcc-12*.deb"
        "dctrl-tools*.deb"
        "dkms*.deb"
        "backport-iwlwifi-dkms*.deb"
    )
    
    echo "检查包文件..."
    echo "Checking package files..."
    
    for pattern in "${packages[@]}"; do
        if ls $pattern >/dev/null 2>&1; then
            log_info "找到 / Found: $(ls $pattern | head -1)"
        else
            log_error "未找到匹配 $pattern 的包文件"
            log_error "No package file matching $pattern found"
            return 1
        fi
    done
    
    echo ""
    return 0
}

# 安装函数
install_packages() {
    echo "开始安装依赖包..."
    echo "Starting installation of dependency packages..."
    echo "----------------------------------------"
    
    # 第一阶段：基础库
    echo "安装基础库..."
    echo "Installing basic libraries..."
    for pkg in cpp-12*.deb libasan8*.deb libtsan2*.deb libgcc-12-dev*.deb; do
        log_info "安装 / Installing: $(basename $pkg)"
        dpkg -i $pkg 2>&1 | grep -E "(Selecting|Unpacking|Setting)" || true
    done
    
    # 第二阶段：编译工具
    echo "安装编译工具..."
    echo "Installing compilation tools..."
    for pkg in gcc-12*.deb; do
        log_info "安装 / Installing: $(basename $pkg)"
        dpkg -i $pkg 2>&1 | grep -E "(Selecting|Unpacking|Setting)" || true
    done
    
    # 第三阶段：工具包
    echo "安装工具包..."
    echo "Installing utility packages..."
    for pkg in dctrl-tools*.deb; do
        log_info "安装 / Installing: $(basename $pkg)"
        dpkg -i $pkg 2>&1 | grep -E "(Selecting|Unpacking|Setting)" || true
    done
    
    # 第四阶段：DKMS框架
    echo "安装DKMS框架..."
    echo "Installing DKMS framework..."
    for pkg in dkms*.deb; do
        log_info "安装 / Installing: $(basename $pkg)"
        dpkg -i $pkg 2>&1 | grep -E "(Selecting|Unpacking|Setting)" || true
    done
    
    # 第五阶段：驱动包
    echo "安装iwlwifi驱动..."
    echo "Installing iwlwifi driver..."
    for pkg in backport-iwlwifi-dkms*.deb; do
        log_info "安装 / Installing: $(basename $pkg)"
        dpkg -i $pkg 2>&1 | grep -E "(Selecting|Unpacking|Setting)" || true
    done
    
    echo "----------------------------------------"
    echo "所有包已安装完成！"
    echo "All packages installed successfully!"
}

# 修复依赖问题
fix_dependencies() {
    echo "修复依赖关系..."
    echo "Fixing dependencies..."
    apt-get install -f -y 2>&1 | tail -5
}

# 验证安装
verify_installation() {
    echo ""
    echo "验证安装结果..."
    echo "Verifying installation..."
    echo "----------------------------------------"
    
    # 检查dkms状态
    echo "DKMS状态："
    echo "DKMS Status:"
    dkms status | grep -i iwlwifi || {
        log_warn "未找到iwlwifi DKMS条目"
        log_warn "No iwlwifi DKMS entry found"
    }
    
    # 检查内核模块
    echo "内核模块："
    echo "Kernel modules:"
    ls -la /lib/modules/$(uname -r)/updates/dkms/iwl*.ko 2>/dev/null || {
        log_warn "未找到iwlwifi内核模块"
        log_warn "No iwlwifi kernel modules found"
    }
    
    # 尝试加载模块
    echo "测试模块加载..."
    echo "Testing module loading..."
    modprobe -r iwlwifi 2>/dev/null || true
    if modprobe iwlwifi 2>&1; then
        log_info "✅ iwlwifi模块加载成功"
        log_info "✅ iwlwifi module loaded successfully"
    else
        log_error "❌ iwlwifi模块加载失败"
        log_error "❌ iwlwifi module failed to load"
    fi
    
    # 检查无线设备
    echo "网络设备："
    echo "Network devices:"
    ip link show | grep -E "(wl|wlan|wlp)" || {
        log_warn "未检测到无线网卡"
        log_warn "No wireless network card detected"
    }
}

# 主函数
main() {
    echo "安装前检查..."
    echo "Pre-installation check..."
    echo ""
    
    # 切换到包所在目录
    # local PACKAGE_DIR="$HOME/iwlwifi-offline/dependency_packages"
    local USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)
    local PACKAGE_DIR="$USER_HOME/iwlwifi-offline/dependency_packages"
    
    if [ -d "$PACKAGE_DIR" ]; then
        cd "$PACKAGE_DIR"
        log_info "已切换到包目录 / Switched to package directory: $(pwd)"
    else
        log_error "包目录不存在 / Package directory does not exist: $PACKAGE_DIR"
        log_error "当前用户家目录 / Current user home: $HOME"
        log_error "请确保 iwlwifi-offline 目录存在"
        log_error "Please ensure the iwlwifi-offline directory exists"
        exit 1
    fi
    
    # 显示将要安装的包
    echo ""
    echo "将安装以下包："
    echo "The following packages will be installed:"
    echo "----------------------------------------"
    ls *.deb 2>/dev/null || find . -maxdepth 1 -name "*.deb" 2>/dev/null
    echo "----------------------------------------"
    
    # 检查包文件
    if ! check_packages; then
        log_error "包文件检查失败，请确保所有deb文件在当前目录"
        log_error "Package check failed, please ensure all deb files are in the current directory"
        exit 1
    fi
    
    # 确认安装
    echo ""
    echo "是否继续安装？"
    echo "Do you want to continue the installation?"
    read -p "(y/N): " confirm
    
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        echo ""
        log_info "安装已取消"
        log_info "Installation cancelled"
        exit 0
    fi
    
    echo ""
    
    # 执行安装
    install_packages
    
    # 修复依赖
    fix_dependencies
    
    # 验证安装
    verify_installation
    
    echo ""
    log_info "安装完成！"
    log_info "Installation completed!"
    echo ""
    log_info "可能需要重启系统使驱动生效"
    log_info "A system reboot may be required for the driver to take effect"
    log_info "重启命令 / Reboot command: sudo reboot"
}

# 运行主函数
main