# Qwen Code プロジェクトドキュメント

このドキュメントは、Qwen Codeを使用して開発されたプロジェクトの情報を記載しています。

# QWENの解説する言語
常に日本語で解説してください

## プロジェクト概要

このプロジェクトは、ROS2 MoveIt2を使用したデュアルアームロボットの設定とデモを提供します。

## 主要機能

- デュアルアームロボットのMoveIt2設定
- 競技フィールドの表示（赤/青チーム対応）
- ロボットアームの制御とパスプランニング
- RVizによる可視化

## 使用方法

### 基本的な起動方法

```bash
ros2 launch robot_config dual_arm_demo.launch.py
```

### フィールドオプションの指定

競技フィールドの表示をカスタマイズするには、`field`パラメータを使用します：

```bash
# 赤チーム用フィールド（回転なし）
ros2 launch robot_config dual_arm_demo.launch.py field:=red

# 青チーム用フィールド（180度回転）
ros2 launch robot_config dual_arm_demo.launch.py field:=blue
```

## カスタマイズ

フィールドメッシュの回転角度は、`publish_collision_mesh.py`ファイルで定義されています。
必要に応じて変更可能です。
