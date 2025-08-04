# これはrvizにキャチアームを表示して動作計画を体験できるものです
moveitの環境構築ができたらsrc下にある0525_arm_description-20250707T055749Z-1-001とrobot_configをクローンして、

```
cd your_workspace_name
colcon build --packages-select 0525_arm_description robot_config
source install/setup.bash
ros2 launch robot_config demo.launch.py
```

rviz上にアームが出てきてplanとexecuteができたら成功

フィールドを出現させたい場合はrvizが開けたらMotionplanningのSceneObjectsを選択し、左下のドロップダウンリストを押し、MeshfromFileを選択、fieldのstlファイル(catch/src/field_description-20250804T033832Z-1-001/field_description/meshes/base_link.stl)を選択する。ちゃんと表示されたらpublishボタンを押す。
