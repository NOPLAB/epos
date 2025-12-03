# YOLO Models

This directory contains YOLO model files for human body detection.

## Quick Setup

Run the download script:

```bash
ros2 run shooter_control download_models.sh
```

## Manual Download

If the script doesn't work, download manually:

1. **yolov4-tiny.weights** (24MB) - Required
   ```bash
   wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
   ```

2. **yolo11n.pt** (5.4MB) - Optional (for future use)
   ```bash
   wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
   ```

## File List

| File | Size | Status |
|------|------|--------|
| coco.names | 625B | Git tracked |
| yolov4-tiny.cfg | 3.2KB | Git tracked |
| yolov4-tiny.weights | 24MB | Download required |
| yolo11n.pt | 5.4MB | Optional download |
