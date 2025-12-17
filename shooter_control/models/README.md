# Detection Models

This directory contains model files for human body detection.

## Quick Setup

Run the download script:

```bash
ros2 run shooter_control download_models.sh
```

## Manual Download

If the script doesn't work, download manually:

### MobileNet-SSD (Recommended for Raspberry Pi)

Faster than YOLO, suitable for edge devices.

1. **MobileNetSSD_deploy.prototxt** - Network definition
   ```bash
   wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/MobileNetSSD_deploy.prototxt
   ```

2. **MobileNetSSD_deploy.caffemodel** (23MB) - Required
   ```bash
   wget https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.caffemodel
   ```

### YOLO v4-tiny (Higher accuracy, slower)

1. **yolov4-tiny.weights** (24MB) - Required
   ```bash
   wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
   ```

## File List

| File | Size | Model | Status |
|------|------|-------|--------|
| MobileNetSSD_deploy.prototxt | 28KB | MobileNet-SSD | Download required |
| MobileNetSSD_deploy.caffemodel | 23MB | MobileNet-SSD | Download required |
| coco.names | 625B | YOLO | Git tracked |
| yolov4-tiny.cfg | 3.2KB | YOLO | Git tracked |
| yolov4-tiny.weights | 24MB | YOLO | Download required |

## Model Selection

Set the `model_type` parameter in your launch file:

```yaml
model_type: "mobilenet-ssd"  # Recommended for Raspberry Pi (faster)
model_type: "yolo"           # Higher accuracy but slower
```
