# complex_networks_hw

大学院授業:複雑ネットワーク特論（2025年度開講）の課題

## 起動

### デモ

Joint Known

```bash
ros2 launch sushi_bot_worlds launch_world_with_sushi_bot.launch.py
ros2 run sushi_bot_task_executor task_executor
```

### rvizシミュレーション

```bash
ros2 launch sushi_bot_bringup sushi_bot.launch.py
```

### gazeboシミュレーション

Empty World

```bash
ros2 launch sushi_bot_bringup sushi_bot_sim_gazebo_classic.launch.py 
```

Sushi World

```bash
ros2 launch sushi_bot_bringup sushi_bot_sim_gazebo_classic.launch.py world:="/home/wsl_ubuntu/colcon_ws/src/complex_networks_hw/sushi_bot_worlds/worlds/sushi_pick_and_place.sdf"
```

### joint_trajectory_controllerからの制御を試す

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

