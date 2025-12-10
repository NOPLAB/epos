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
echo "Downloading YOLO models to: $MODELS_DIR"

# YOLOv4-tiny weights (required)
if [ ! -f "yolov4-tiny.weights" ]; then
    echo "Downloading yolov4-tiny.weights (24MB)..."
    wget -q --show-progress https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
else
    echo "yolov4-tiny.weights already exists, skipping."
fi

# YOLO11n (optional, for future use)
if [ ! -f "yolo11n.pt" ]; then
    echo "Downloading yolo11n.pt (5.4MB)..."
    wget -q --show-progress https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
else
    echo "yolo11n.pt already exists, skipping."
fi

# NanoTrack models (required for object tracking)
if [ ! -f "nanotrack_backbone_sim.onnx" ]; then
    echo "Downloading nanotrack_backbone_sim.onnx (286KB)..."
    curl -L -o nanotrack_backbone_sim.onnx \
        https://github.com/opencv/opencv_zoo/raw/main/models/object_tracking_nanotrack/nanotrack_backbone_sim.onnx
else
    echo "nanotrack_backbone_sim.onnx already exists, skipping."
fi

if [ ! -f "nanotrack_head_sim.onnx" ]; then
    echo "Downloading nanotrack_head_sim.onnx (286KB)..."
    curl -L -o nanotrack_head_sim.onnx \
        https://github.com/opencv/opencv_zoo/raw/main/models/object_tracking_nanotrack/nanotrack_head_sim.onnx
else
    echo "nanotrack_head_sim.onnx already exists, skipping."
fi

echo ""
echo "Done! Models in: $MODELS_DIR"
ls -lh "$MODELS_DIR"/*.weights "$MODELS_DIR"/*.pt "$MODELS_DIR"/*.onnx 2>/dev/null || true
