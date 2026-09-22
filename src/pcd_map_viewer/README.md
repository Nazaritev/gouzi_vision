# pcd_map_viewer

读取 PCD 文件并发布为 ROS 2 `sensor_msgs/PointCloud2`，用于 RViz 显示。

## 构建

```bash
cd ~/obstacle_race_src
source /opt/ros/humble/setup.bash
colcon build --packages-select pcd_map_viewer --symlink-install
source install/setup.bash
```

## 使用

```bash
ros2 launch pcd_map_viewer view_pcd.launch.py \
  pcd_path:=/absolute/path/to/scans.pcd
```

默认话题为 `/pcd_map`，PCD 坐标系为 `camera_init`。launch 会额外发布
`map -> camera_init` 的单位静态 TF；在 RViz 中将 Fixed Frame 设置为 `map`，
再添加 `PointCloud2` 并选择 `/pcd_map`。`camera_init` 也会出现在 TF 树中。

若 PCD 使用其他坐标系，可以同时指定两个不同的坐标系名称：

```bash
ros2 launch pcd_map_viewer view_pcd.launch.py \
  pcd_path:=/absolute/path/to/scans.pcd \
  frame_id:=my_pcd_frame \
  fixed_frame:=map
```

也可以直接运行节点：

```bash
ros2 run pcd_map_viewer pcd_map_publisher \
  --ros-args -p pcd_path:=/absolute/path/to/scans.pcd
```

节点会以 transient-local QoS 保存最后一帧，并周期性重发，RViz 后启动也能收到地图。
