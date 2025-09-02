# Dual Arm Realtime Control

このパッケージには、left_arm_realtime_controlとright_arm_realtime_controlトピックを受信して、デュアルアームのエンドエフェクタをリアルタイムで制御するノードが含まれています。

## ノード一覧

### 1. dual_arm_realtime_control
基本的なリアルタイム制御ノード。MoveItのposeTargetを使用して制御を行います。

### 2. dual_arm_cartesian_control
Cartesian経路計画を使用したより滑らかな制御ノード。推奨されるノードです。

## 使用方法

### 基本的な起動

```bash
# Cartesian制御ノードを起動（推奨）
ros2 launch robot_config dual_arm_realtime_control.launch.py control_type:=cartesian

# リアルタイム制御ノードを起動
ros2 launch robot_config dual_arm_realtime_control.launch.py control_type:=realtime
```

### パラメータ付きで起動

```bash
# 制御周波数と速度スケールを設定
ros2 launch robot_config dual_arm_realtime_control.launch.py \
    control_type:=cartesian \
    control_frequency:=30.0 \
    velocity_scale:=0.2 \
    timeout_duration:=1.0
```

### 個別ノードの起動

```bash
# Cartesian制御ノード
ros2 run robot_config dual_arm_cartesian_control

# リアルタイム制御ノード
ros2 run robot_config dual_arm_realtime_control
```

## トピック

### 入力トピック
- `/left_arm_realtime_control` (geometry_msgs/msg/Twist): 左アームの速度指令
- `/right_arm_realtime_control` (geometry_msgs/msg/Twist): 右アームの速度指令

### Twistメッセージの構成
```
linear:
  x: X軸方向の線形速度
  y: Y軸方向の線形速度
  z: Z軸方向の線形速度（通常0）
angular:
  x: ロール角速度（通常0）
  y: ピッチ角速度（通常0）
  z: ヨー角速度
```

## パラメータ

- `control_frequency`: 制御ループの周波数 (Hz) [デフォルト: 20.0]
- `velocity_scale`: 速度スケーリング係数 [デフォルト: 0.1]
- `timeout_duration`: コマンドタイムアウト時間 (秒) [デフォルト: 0.5]

## 安全機能

1. **位置制限**: エンドエフェクタが安全範囲外に移動しようとした場合は停止
2. **タイムアウト**: 指定時間内にコマンドが来ない場合は停止
3. **速度制限**: MoveItの速度・加速度制限を適用

## 例：テスト用のコマンド送信

```bash
# 左アームを前方に移動
ros2 topic pub /left_arm_realtime_control geometry_msgs/msg/Twist \
    '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 右アームを時計回りに回転
ros2 topic pub /right_arm_realtime_control geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'

# 停止コマンド
ros2 topic pub /left_arm_realtime_control geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 注意事項

1. MoveItが正しく起動されている必要があります
2. アームの初期位置が安全範囲内にある必要があります
3. 制御開始前に、ロボットが適切な初期姿勢にあることを確認してください
4. Cartesian制御の方が滑らかな動作を提供しますが、計算負荷が高くなります

## トラブルシューティング

### エラー: "Failed to get current pose"
- MoveItが正しく起動されているか確認
- アームのgroup名が正しいか確認（"left_arm", "right_arm"）

### エラー: "Planning failed"
- 目標位置が到達可能か確認
- 障害物がないか確認
- 速度スケールを下げてみる

### 動作が遅い・カクカクする
- control_frequencyを上げる
- velocity_scaleを調整
- Cartesian制御を使用する
