#!/usr/bin/env bash
# setup.sh — Openclaw-SOARM-Catch_cube 一键安装脚本
#
# 用法:
#   bash setup.sh [--env ENV] [--platform jetson|desktop] [--skip-lerobot] [--skip-pinocchio]
#
# 默认:
#   --env lerobot  --platform jetson
#
# 安装顺序:
#   1. (Jetson) LeRobot 环境 (torch GPU wheel + lerobot)
#      (Desktop) conda env + pip install lerobot
#   2. Pinocchio (逆运动学库)
#   3. soarm-control 校准文件 + 串口权限

set -euo pipefail

# ── 默认值 ────────────────────────────────────────────────────────────────────
ENV_NAME="lerobot"
PLATFORM="jetson"
SKIP_LEROBOT=false
SKIP_PINOCCHIO=false
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── 参数解析 ──────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)             ENV_NAME="$2";      shift 2 ;;
    --platform)        PLATFORM="$2";      shift 2 ;;
    --skip-lerobot)    SKIP_LEROBOT=true;  shift ;;
    --skip-pinocchio)  SKIP_PINOCCHIO=true; shift ;;
    -h|--help)
      echo "用法: bash setup.sh [--env ENV] [--platform jetson|desktop] [--skip-lerobot] [--skip-pinocchio]"
      exit 0 ;;
    *) echo "[error] 未知参数: $1"; exit 1 ;;
  esac
done

log()  { echo -e "\n\033[1;32m[setup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[warn]\033[0m  $*"; }
die()  { echo -e "\033[1;31m[error]\033[0m $*" >&2; exit 1; }

log "项目目录: $PROJECT_DIR"
log "Conda 环境: $ENV_NAME"
log "平台: $PLATFORM"

# ── Conda 检测 ────────────────────────────────────────────────────────────────
CONDA_BIN=""
for _p in \
  "$(command -v conda 2>/dev/null)" \
  "$HOME/miniconda3/bin/conda" \
  "$HOME/anaconda3/bin/conda" \
  "$HOME/miniforge3/bin/conda" \
  "$HOME/mambaforge/bin/conda" \
  "/opt/conda/bin/conda"; do
  if [[ -n "$_p" && -x "$_p" ]]; then
    CONDA_BIN="$_p"
    break
  fi
done
[[ -z "$CONDA_BIN" ]] && die "未找到 conda。请先安装 Miniconda:\n  wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-$(uname -m).sh\n  bash Miniconda3-latest-Linux-$(uname -m).sh"
log "conda: $CONDA_BIN"

# ══════════════════════════════════════════════════════════════════════════════
# Step 1: LeRobot 环境安装
# ══════════════════════════════════════════════════════════════════════════════
if [[ "$SKIP_LEROBOT" == true ]]; then
  log "跳过 LeRobot 安装 (--skip-lerobot)"
else
  if "$CONDA_BIN" run -n "$ENV_NAME" python3 -c "import lerobot" 2>/dev/null; then
    log "Step 1: LeRobot 已安装在 '$ENV_NAME' 中 — 跳过"
  else
    if [[ "$PLATFORM" == "jetson" ]]; then
      log "Step 1: Jetson 平台 — 使用 lerobot-env-setup 安装 (分阶段执行)"
      LEROBOT_SCRIPT="$PROJECT_DIR/lerobot-env-setup/scripts/install_lerobot.sh"
      [[ -f "$LEROBOT_SCRIPT" ]] || die "找不到 $LEROBOT_SCRIPT"

      for phase in env download torch-pre deps torch-post; do
        log "  Phase: $phase"
        bash "$LEROBOT_SCRIPT" --env "$ENV_NAME" --robot-type so-arm --phase "$phase"
      done
    else
      log "Step 1: Desktop 平台 — 标准 pip 安装"
      # 创建 conda 环境
      if ! "$CONDA_BIN" env list 2>/dev/null | grep -qE "^${ENV_NAME}\s"; then
        log "  创建 conda 环境 '$ENV_NAME' (python=3.10)..."
        "$CONDA_BIN" create -y -n "$ENV_NAME" python=3.10
      fi

      # 克隆并安装 lerobot
      LEROBOT_DIR="$HOME/lerobot"
      if [[ ! -d "$LEROBOT_DIR" ]]; then
        log "  克隆 lerobot..."
        git clone https://github.com/huggingface/lerobot.git "$LEROBOT_DIR"
      fi
      log "  安装 lerobot (editable + feetech)..."
      "$CONDA_BIN" run -n "$ENV_NAME" pip install -e "$LEROBOT_DIR/.[feetech]"
    fi
  fi
  log "Step 1: LeRobot 环境 ✓"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Step 2: Pinocchio 安装
# ══════════════════════════════════════════════════════════════════════════════
if [[ "$SKIP_PINOCCHIO" == true ]]; then
  log "跳过 Pinocchio 安装 (--skip-pinocchio)"
else
  if "$CONDA_BIN" run -n "$ENV_NAME" python3 -c "import pinocchio" 2>/dev/null; then
    log "Step 2: Pinocchio 已安装 — 跳过"
  else
    log "Step 2: 安装 Pinocchio..."
    PINOCCHIO_SCRIPT="$PROJECT_DIR/pinocchio-install/scripts/install_pinocchio.sh"
    if [[ -f "$PINOCCHIO_SCRIPT" ]]; then
      ARCH="$(uname -m)"
      if [[ "$ARCH" == "aarch64" ]]; then
        METHOD="source"
        log "  aarch64 平台，使用源码编译..."
      else
        METHOD="conda"
        log "  x86_64 平台，使用 conda 安装..."
      fi
      bash "$PINOCCHIO_SCRIPT" --env "$ENV_NAME" --method "$METHOD" --phase all
    else
      log "  使用 conda 直接安装 pinocchio..."
      "$CONDA_BIN" install -n "$ENV_NAME" -c conda-forge pinocchio -y
    fi
  fi
  log "Step 2: Pinocchio ✓"
fi

# ══════════════════════════════════════════════════════════════════════════════
# Step 3: soarm-control 配置
# ══════════════════════════════════════════════════════════════════════════════
log "Step 3: 配置 soarm-control..."

# 3a. 校准文件
CALIB_DIR="$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower"
CALIB_SRC="$PROJECT_DIR/soarm-control/references/openclaw_soarm.json"
CALIB_DST="$CALIB_DIR/openclaw_soarm.json"

if [[ -f "$CALIB_DST" ]]; then
  log "  校准文件已存在 — 跳过"
else
  log "  复制校准文件..."
  mkdir -p "$CALIB_DIR"
  cp "$CALIB_SRC" "$CALIB_DST"
  log "  校准文件: $CALIB_DST"
fi

# 3b. 串口权限
if id -nG | grep -qw dialout; then
  log "  用户已在 dialout 组 — 跳过"
else
  log "  添加用户到 dialout 组..."
  sudo usermod -aG dialout "$USER"
  warn "需要重新登录才能生效"
fi

# 3c. 移除 brltty (会抢占串口)
if dpkg -l brltty 2>/dev/null | grep -q '^ii'; then
  log "  移除 brltty (避免串口冲突)..."
  sudo apt remove -y brltty
fi

# 3d. udev 规则
UDEV_RULE="/etc/udev/rules.d/99-serial-ports.rules"
if [[ -f "$UDEV_RULE" ]]; then
  log "  udev 规则已存在 — 跳过"
else
  log "  创建串口 udev 规则..."
  sudo tee "$UDEV_RULE" > /dev/null <<'EOF'
KERNEL=="ttyUSB[0-9]*", MODE="0666"
KERNEL=="ttyACM[0-9]*", MODE="0666"
EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger
fi

log "Step 3: soarm-control 配置 ✓"

# ══════════════════════════════════════════════════════════════════════════════
# 验证
# ══════════════════════════════════════════════════════════════════════════════
log "Step 4: 验证安装..."

FAIL=0
echo -n "  lerobot:   "
"$CONDA_BIN" run -n "$ENV_NAME" python3 -c "import lerobot; print('✓')" 2>/dev/null || { echo "✗"; FAIL=1; }

echo -n "  pinocchio: "
"$CONDA_BIN" run -n "$ENV_NAME" python3 -c "import pinocchio; print('✓ ' + pinocchio.__version__)" 2>/dev/null || { echo "✗"; FAIL=1; }

echo -n "  numpy:     "
"$CONDA_BIN" run -n "$ENV_NAME" python3 -c "import numpy; print('✓ ' + numpy.__version__)" 2>/dev/null || { echo "✗"; FAIL=1; }

echo -n "  校准文件:  "
[[ -f "$CALIB_DST" ]] && echo "✓" || { echo "✗"; FAIL=1; }

echo -n "  URDF:      "
[[ -f "$PROJECT_DIR/soarm-control/references/so101_new_calib.urdf" ]] && echo "✓" || { echo "✗"; FAIL=1; }

if [[ "$FAIL" -eq 0 ]]; then
  log "所有检查通过 ✓"
  echo ""
  echo "============================================"
  echo "  安装完成！使用方法:"
  echo ""
  echo "  # 激活环境"
  echo "  conda activate $ENV_NAME"
  echo ""
  echo "  # 移动机械臂到指定坐标"
  echo "  python soarm-control/scripts/pinocchio_xyz_drive.py --x 0.2 --y 0.0 --z 0.1"
  echo ""
  echo "  # 查看机械臂状态"
  echo "  python soarm-control/scripts/soarm_status.py"
  echo "============================================"
else
  warn "部分检查未通过，请查看上方输出排查问题"
  exit 1
fi
