# CLAUDE.md

このファイルは、Claude Code (claude.ai/code) がこのリポジトリのコードを扱う際のガイダンスを提供します。
必ず日本語で応答してください。

## プロジェクト概要

Maxon EPOS4モーターコントローラ用の`ros2_control`ハードウェアインターフェースとC++ラッパーライブラリを提供するROS 2パッケージです。EPOS4デバイスをROS 2 controlフレームワークと統合することを目的としています。

**通信方式:** USB経由でEPOS4ハードウェアデバイスとCANopenプロトコルで通信

## ビルド・実行コマンド

### Docker開発環境（推奨）

```bash
# イメージのビルド
docker compose build

# 開発シェル（ソースマウントによるライブ編集対応）
docker compose run shooter

# 実機 + キーボード操作
docker compose --profile real --profile keyboard up

# 実機 + ボディトラッキング
docker compose --profile real --profile body up

# Gazeboシミュレーション + キーボード操作
docker compose --profile sim --profile keyboard up

# Gazeboシミュレーション + ボディトラッキング
docker compose --profile sim --profile body up
```

### ネイティブビルド（ROS 2ワークスペースルートから）

```bash
# パッケージのビルド
colcon build --packages-select epos shooter

# IDE対応のためcompile_commands.jsonを生成してビルド
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# shooterロボットの起動
ros2 launch shooter shooter.launch.py

# ボディトラッカーの起動（別ターミナル）
ros2 launch shooter body_tracker.launch.py

# キーボードテレオペ（別ターミナル）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

**EPOSHardwareInterface** (`epos/include/epos/EPOSHardwareInterface.h`)
- ros2_control用の`hardware_interface::SystemInterface`を実装
- 複数のジョイントを管理し、各ジョイントが独自のEPOSControllerインスタンスを持つ
- 速度コマンドと位置/速度状態インターフェースをエクスポート
- URDFの`<ros2_control>`タグで設定（`shooter/urdf/shooter.urdf.xacro`参照）

**EPOSController** (`epos/include/epos/EPOSController.h`)
- Maxon EPOS4 API用の低レベルファサード
- デバイス初期化、モード有効化、モーションコマンドを処理
- コンストラクタのデフォルト値: EPOS4デバイス、USB0ポート、1 Mbpsボーレート、ノードID 1
- 全パブリックメソッドは`bool`を返し、エラーは`getLastErrorCode()`で取得

**BodyTrackerNode** (`shooter/src/body_tracker_node.cpp`)
- HOG記述子を使用したOpenCVベースの人体検出
- 追跡用PIコントローラ（`/diff_drive_controller/cmd_vel_unstamped`にパブリッシュ）
- 直接カメラデバイスまたはROS画像トピックを使用可能

**Definitions.h** - ベンダー提供のC APIヘッダ（CANopen/USB通信用の100以上の関数）

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

## 開発メモ

- **C++標準**: C++14
- **ビルドシステム**: ament_cmake (ROS 2)
- **プラグイン登録**: `epos_hardware.xml`で`epos/EPOSHardwareInterface`をエクスポート
- **ベンダーサンプル**: `EPOS_Linux_Library/examples/HelloEposCmd/`で生のlibEposCmd使用例を確認可能
