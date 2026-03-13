#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_EXEC="/home/seeed/miniconda3/envs/lerobot/bin/python"
VISUAL_SERVO_SERVER="$SCRIPT_DIR/visual_servo_server.py"
DETECTION_SERVER="$SCRIPT_DIR/detection_server.py"
DETECTION_WEIGHTS="$SCRIPT_DIR/best.pt"

# 退出时清理子进程
cleanup() {
    echo ""
    echo "正在停止所有服务..."
    kill $DETECT_PID $SERVO_PID 2>/dev/null
    wait $DETECT_PID $SERVO_PID 2>/dev/null
    echo "✓ 所有服务已停止"
}
trap cleanup EXIT INT TERM

if [ ! -f "$PYTHON_EXEC" ]; then
    echo "Error: Python executable not found at $PYTHON_EXEC"
    exit 1
fi

# 启动摄像头检测服务 (port 5000)
echo "Starting YOLOv11 Detection Server (port 5000)..."
"$PYTHON_EXEC" "$DETECTION_SERVER" --weights "$DETECTION_WEIGHTS" "$@" &
DETECT_PID=$!

# 等待检测服务启动
echo "等待检测服务启动..."
sleep 3

# 启动视觉伺服抓取服务 (port 8002)
echo "Starting Visual Servo Pick Server (port 8002)..."
"$PYTHON_EXEC" "$VISUAL_SERVO_SERVER" &
SERVO_PID=$!

echo ""
echo "============================================"
echo "  所有服务已启动"
echo "  检测服务:   http://localhost:5000"
echo "  抓取服务:   http://localhost:8002"
echo ""
echo "  触发抓取:"
echo "  curl -X POST http://localhost:8002/pick"
echo "  查询状态:"
echo "  curl http://localhost:8002/status"
echo "============================================"
echo ""

# 等待任意子进程退出
wait -n $DETECT_PID $SERVO_PID
echo "[警告] 某个服务已退出，正在关闭..."
