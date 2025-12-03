# shooter

## 概要

Maxon EPOS4モーターコントローラ用の`ros2_control`ハードウェアインターフェースとC++ラッパーライブラリを提供するROS 2パッケージです。EPOS4デバイスをROS 2 controlフレームワークと統合し、USB経由でCANopenプロトコルによる通信を行います。

### アーキテクチャ

```
ros2_control framework (diff_drive_controller)
    ↓
EPOSHardwareInterface (ros2_control SystemInterfaceプラグイン)
    ↓
EPOSController (C++ラッパー)
    ↓
libEposCmd (Maxonベンダーライブラリ)
    ↓
USB → EPOS4ハードウェア
```

### 依存関係

- ROS 2 Humble (`ros2_control`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`)
- libEposCmd (Maxonベンダーライブラリ、[Maxon公式サイト](https://www.maxongroup.com/)から別途ダウンロードして`EPOS_Linux_Library/`に配置)
- OpenCV (人体検出用)

### 対応アーキテクチャ

x86, x86_64, ARM (soft-float, hard-float, aarch64)

## ライセンス・権利

MIT License

Copyright 2025 nop

libEposCmdライブラリはMaxon Motor AGの所有物であり、Maxonのライセンス条項に従います。本リポジトリには含まれていないため、別途ダウンロードが必要です。

