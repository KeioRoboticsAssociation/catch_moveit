# d415\_rgb\_depth\_3d

GUIでタップした**画像座標 (u, v)** を 3D 目標姿勢 `PoseStamped` に変換して配信する ROS 2 Python ノードです。
D415 のカラーに**アライン済み**の深度画像とカメラ内部・外部パラを使い、以下の2つの投影モードを切り替えられます。

* **`fixed_world_z`（推奨/デフォルト）**: 深度を使わず、世界座標の高さ `Z = fixed_world_z`（例: テーブル高さ）とカメラ視線レイの**交点**を出力
* **`depth`**: 深度から (X, Y, Z) を復元し、固定外部パラで world に変換。安全のため `hover_offset` を Z に加算して出力

---

## 1. 動作イメージ

```
GUI（u,v） --/tap/d415_pixel-->
          D415TapToPose  ──> /d415/target_pose (PoseStamped)
                      ↑
    /camera/color/camera_info, /camera/aligned_depth_to_color/image_raw（depthモードのみ）
    ＋ 固定カメラ外部パラ（位置・姿勢, 起動時パラメータ）
```

---

## 2. 主要機能

* タップ座標 (u,v) → ピンホールモデルでレイ生成
* `fixed_world_z` モード：レイと **world Z=一定** 平面の交点を厳密計算（深度不要）
* `depth` モード：タップ近傍の深度中央値でノイズ低減 → 3D 復元
* world 変換は**固定外部パラ**（カメラ位置・姿勢）で実行（`/camera/pose` は不要）
* 出力は `PoseStamped`（位置+姿勢）。姿勢はカメラ姿勢を基本とし、必要ならツール補正を別途合成可能

---

## 3. トピック

### 入力

* `/tap/d415_pixel` : `geometry_msgs/Point` — **x=u, y=v**（z未使用）
* `/camera/color/camera_info` : `sensor_msgs/CameraInfo`
* `/camera/aligned_depth_to_color/image_raw` : `sensor_msgs/Image`（**depth モード時のみ必須**）
* `/camera/color/image_raw` : `sensor_msgs/Image`（任意・同期/デバッグ用途）

### 出力

* `/d415/target_pose` : `geometry_msgs/PoseStamped`

---

## 4. パラメータ一覧（よく使う順）

### 投影モード関連

* `projection_mode` (string, default: `"fixed_world_z"`)

  * `"fixed_world_z"` | `"depth"`
* `fixed_world_z` (float, default: `0.10`)

  * **world座標の絶対 Z 高さ \[m]**（テーブル上面など）

### カメラ外部パラ（固定）

* `camera_position_x` / `camera_position_y` / `camera_position_z` (float)

  * world 中のカメラ原点 \[m]
* 姿勢は **どちらか一方で指定**

  * **クォータニオン**：`camera_orientation_{x,y,z,w}`
  * **RPY(度)**：`camera_{roll,pitch,yaw}_deg`（内部で ZYXオーダの回転行列に変換）

> 姿勢の定義は **world ← camera**（カメラ座標のベクトルを world に回す回転）。RVizで矢印向きを要確認。

### depth モード専用

* `median_ksize` (int, default: `5`)

  * タップ近傍の深度中央値窓サイズ（奇数）
* `hover_offset` (float, default: `0.03`)

  * depth モードでZに足す安全マージン \[m]

### トピック名

* `tap_topic` (string, default: `"/tap/d415_pixel"`)
* `camera_info_topic` (string, default: `"/camera/color/camera_info"`)
* `depth_topic` (string, default: `"/camera/aligned_depth_to_color/image_raw"`)
* `color_topic` (string, default: `"/camera/color/image_raw"`)
* `target_frame` (string, default: `"world"`)

---

## 5. インストール

### 依存

* ROS 2 (Humble 以降推奨)
* `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`, `opencv-python`
* RealSense 側は**カラーにアラインした深度**を出力設定に

### ビルド

```bash
cd ~/ros2_ws/src
# すでに本パッケージが配置済みと仮定
cd ..
colcon build --symlink-install
source install/setup.bash
```

---

## 6. 起動例（launch パラメータ）

### A. 固定 World-Z 投影（深度を使わない）

```python
# d415_rgb_depth_3d.launch.py の parameters 例
parameters=[{
  'projection_mode': 'fixed_world_z',
  'fixed_world_z': 0.10,          # テーブル高さ（world絶対Z）
  'camera_position_x': 0.50,      # 例: テーブル原点から右方向に 0.5 m
  'camera_position_y': 0.00,
  'camera_position_z': 0.80,      # カメラ高さ 0.8 m
  'camera_pitch_deg': -30.0,      # 俯角。軸定義に合わせ調整
  'target_frame': 'world',

  # info/カラーは購読継続（depthは未使用）
  'camera_info_topic': '/d415/color/camera_info',
  'color_topic': '/d415/color/image_raw',
}]
```

### B. Depth 使用

```python
parameters=[{
  'projection_mode': 'depth',
  'hover_offset': 0.03,

  # world 変換のため外部パラは引き続き必要
  'camera_position_x': 0.50,      # 例: テーブル原点から右方向に 0.5 m
  'camera_position_y': 0.00,
  'camera_position_z': 0.80,
  'camera_pitch_deg': -30.0,
  'target_frame': 'world',

  # depth と info
  'camera_info_topic': '/d415/color/camera_info',
  'depth_topic': '/d415/aligned_depth_to_color/image_raw',
}]
```

---

## 7. 手動テスト

1. RealSense / CameraInfo を流す（depth モードなら `aligned_depth_to_color` も）
2. タップを擬似的に publish:

```bash
# u=320, v=240 をタップした想定
ros2 topic pub /tap/d415_pixel geometry_msgs/Point "{x: 320.0, y: 240.0, z: 0.0}"
```

3. 出力を確認:

```bash
ros2 topic echo /d415/target_pose
```

4. RViz2 で `Pose` と `TF` を可視化すると、テーブル面の Z=固定高さに矢印が落ちることを確認できます。

---

## 8. 既知の注意点 / トラブルシュート

* **レイが平面にほぼ平行**（`fixed_world_z` モード）：
  カメラが水平に近いと交点が計算できません。カメラ俯角を増やすか、`fixed_world_z` を見直してください。
* **座標系の向き**：
  world の軸は **+X 右, +Y 奥, +Z 上**。テーブル上面が `fixed_world_z` になるよう環境基準を統一してください。
* **CameraInfo 未受信**：
  `K=(fx,fy,cx,cy)` が無いと投影できません。`camera_info_topic` が正しく流れているか確認。
* **depth モードで穴/ノイズ**：
  `median_ksize` を広げる、RealSense の Temporal/Spatial フィルタを有効化、ROI内の外れ値除去などで改善。

---

## 9. パッケージ構成

```
d415_rgb_depth_3d/
├─ d415_rgb_depth_3d/
│  ├─ __init__.py
│  └─ d415_rgb_depth_3d_node.py
├─ launch/
│  └─ d415_rgb_depth_3d.launch.py
├─ package.xml
├─ setup.py
├─ setup.cfg
└─ resource/
   └─ d415_rgb_depth_3d
```

---

## 10. 座標系

**GUIタップ座標の原点とスケール**

原点 (0,0)
→ 画像の左上隅（CameraInfo と同じ定義）

軸方向
x（u）: 右方向に増加
y（v）: 下方向に増加

```
World座標系（右手系）
 ├─ X+: 右
 ├─ Y+: 奥
 └─ Z+: 上

Camera座標系（RealSense標準）
 ├─ X+: 右（画像右方向）
 ├─ Y+: 下（画像下方向）
 └─ Z+: 前（カメラが見ている方向）
```
