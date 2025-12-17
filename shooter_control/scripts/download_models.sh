#!/bin/bash
set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Try to find models directory (installed or source)
if [ -d "$SCRIPT_DIR/../share/shooter_control/models" ]; then
    MODELS_DIR="$SCRIPT_DIR/../share/shooter_control/models"
elif [ -d "$SCRIPT_DIR/../models" ]; then
    MODELS_DIR="$SCRIPT_DIR/../models"
else
    echo "Error: Cannot find models directory"
    exit 1
fi

cd "$MODELS_DIR"
echo "Downloading detection models to: $MODELS_DIR"

# MobileNet-SSD (recommended for Raspberry Pi)
echo ""
echo "=== MobileNet-SSD (Recommended for Raspberry Pi) ==="
if [ ! -f "MobileNetSSD_deploy.prototxt" ]; then
    echo "Downloading MobileNetSSD_deploy.prototxt..."
    wget -q --show-progress https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/MobileNetSSD_deploy.prototxt
else
    echo "MobileNetSSD_deploy.prototxt already exists, skipping."
fi

if [ ! -f "MobileNetSSD_deploy.caffemodel" ]; then
    echo "Downloading MobileNetSSD_deploy.caffemodel (23MB)..."
    wget -q --show-progress https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.caffemodel
else
    echo "MobileNetSSD_deploy.caffemodel already exists, skipping."
fi

# YOLOv4-tiny weights (optional, higher accuracy but slower)
echo ""
echo "=== YOLOv4-tiny (Optional, higher accuracy) ==="
if [ ! -f "yolov4-tiny.weights" ]; then
    echo "Downloading yolov4-tiny.weights (24MB)..."
    wget -q --show-progress https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
else
    echo "yolov4-tiny.weights already exists, skipping."
fi

echo ""
echo "Done! Models in: $MODELS_DIR"
ls -lh "$MODELS_DIR"/*.caffemodel "$MODELS_DIR"/*.prototxt "$MODELS_DIR"/*.weights 2>/dev/null || true
