# CLAUDE.md

このファイルは、Claude Code (claude.ai/code) がこのリポジトリのコードを扱う際のガイダンスを提供します。
必ず日本語で応答してください。

## プロジェクト概要

Maxon EPOS4モーターコントローラ用の`ros2_control`ハードウェアインターフェースとC++ラッパーライブラリを提供するROS 2パッケージです。EPOS4デバイスをROS 2 controlフレームワークと統合することを目的としています。

**通信方式:** USB経由でEPOS4ハードウェアデバイスとCANopenプロトコルで通信

## パッケージ構成

```
chikurin/
├── epos/                    # 独立ライブラリ (ros2_control HW IF + C++ラッパー)
├── chikurin_description/    # URDF + 起動 + シミュレーション統合
├── chikurin_control/        # 制御ノード + ホーミングサービス
├── EPOS_Linux_Library/      # Maxonベンダーライブラリ（変更不要）
└── docker/                  # Docker設定（変更不要）
```

## アーキテクチャ

### レイヤー構成
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

### 主要コンポーネント

**eposパッケージ**
- `EPOSController` (`epos/include/epos/EPOSController.h`) - Maxon EPOS4 API用のC++ラッパー
- `EPOSHardwareInterface` (`epos/include/epos/EPOSHardwareInterface.h`) - ros2_control SystemInterfaceプラグイン
- `Definitions.h` - ベンダー提供のC APIヘッダ

**chikurin_descriptionパッケージ**
- `urdf/chikurin.urdf.xacro` - ロボット本体の定義
- `urdf/ros2_control.urdf.xacro` - 実機用ros2_control設定
- `urdf/gazebo.urdf.xacro` - シミュレーション用設定
- `launch/robot.launch.py` - 実機起動
- `launch/sim.launch.py` - シミュレーション起動
- `config/controllers.yaml` - 実機用コントローラ設定
- `config/sim_controllers.yaml` - シミュレーション用コントローラ設定

**chikurin_controlパッケージ**
- `body_tracker_node` - YOLO/OpenCV人体検出・追跡
- `pan_tilt_homing_node` - パンチルトホーミング
- `pan_tilt_joy_node` - ジョイスティック制御
- `homing_service_node` - 汎用ホーミングサービス
- `srv/Homing.srv` - ホーミングサービス定義

## URDF設定

ハードウェアインターフェースはURDFの`<ros2_control>`タグで設定します：

```xml
<ros2_control name="epos_system" type="system">
  <hardware>
    <plugin>epos/EPOSHardwareInterface</plugin>
    <param name="device_name">EPOS4</param>
    <param name="port_name">USB0</param>
    <param name="baudrate">1000000</param>
    <param name="counts_per_revolution">4096</param>
  </hardware>
  <joint name="left_wheel_joint">
    <param name="node_id">1</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## 依存関係

- **ROS 2 Humble** (`ros2_control`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`を含む)
- **libEposCmd** - Maxonベンダーライブラリ（`-lEposCmd`でリンク、`EPOS_Linux_Library/`に同梱）
- **OpenCV** - body_tracker_node用の人体検出
- **対応アーキテクチャ:** x86, x86_64, ARM (soft-float, hard-float, aarch64)

## EPOSController APIリファレンス

**動作モード:**
- 位置モード: `activatePositionMode()`, `moveToPosition()`, `setPositionProfile()`
- 速度モード: `activateVelocityMode()`, `moveWithVelocity()`, `setVelocityProfile()`

**状態管理:**
- `initialize()` - デバイスを開き、フォルトをクリアし、モーターを有効化
- `clearFault()`, `enable()`, `disable()`
- `getFaultState()`, `getEnableState()`

**モーションフィードバック:**
- `getPosition()` - クワッドカウントを返す（エンコーダ分解能の4倍）
- `getVelocity()` - RPMを返す
- `isTargetReached()`

**単位:** 位置はクワッドカウント、速度はRPM、加速度はRPM/s

## 起動方法

**実機起動:**
```bash
ros2 launch chikurin_description robot.launch.py
# ホーミング有効:
ros2 launch chikurin_description robot.launch.py enable_homing:=true
```

**シミュレーション起動:**
```bash
ros2 launch chikurin_description sim.launch.py
```

## 開発メモ

- **C++標準**: C++14
- **ビルドシステム**: ament_cmake (ROS 2)
- **プラグイン登録**: `epos_hardware.xml`で`epos/EPOSHardwareInterface`をエクスポート
- **ベンダーサンプル**: `EPOS_Linux_Library/examples/HelloEposCmd/`で生のlibEposCmd使用例を確認可能
