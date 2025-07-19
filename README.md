# complex_networks_hw

大学院授業:複雑ネットワーク特論（2025年度開講）の課題

## 起動

### rvizシミュレーション

```bash
ros2 launch sushi_bot_bringup sushi_bot.launch.py
```

### gazeboシミュレーション

```bash
ros2 launch sushi_bot_bringup sushi_bot_sim_gazebo_classic.launch.py 
```

### joint_trajectory_controllerからの制御を試す

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
