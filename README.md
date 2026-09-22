## 修改日志

### 2026-05-22
- 修改目的：排查 `orange_pole_detector` 反复打印“等待对齐深度图”时，发现总 launch 传给 RealSense 的 `color_qos/depth_qos/...` 参数并没有在 `rs_launch.py` 里声明，导致相机发布侧 QoS 实际从未按预期切换；同时原日志无法区分“启动初期深度未起来”和“链路持续未接通”。
- 关键行为变化：`realsense2_camera/launch/rs_launch.py` 新增 `color_qos`、`color_info_qos`、`depth_qos`、`depth_info_qos` 四个 launch 参数，令总状态机里传入的 `SENSOR_DATA` QoS 真正生效；`orange_pole_detector` 新增首帧 aligned depth 到达日志，以及 `depth_wait_warn_after_sec` 超时后升级为 warn 的等待日志。
- 回退方式：删除 `rs_launch.py` 中新增的四个 QoS launch 参数；`orange_pole_detector` 删除 `depth_wait_warn_after_sec` 与首帧深度日志，恢复原先统一的“等待对齐深度图”提示。

### 2026-05-22
- 修改目的：镜像桥后 `final_descent_approach` 实测一旦下坡段横向偏差放大，目标点会迅速跑到机身后方，`pose_controller` 随后退化成大角度原地转，卡死在第9点。
- 关键行为变化：镜像 `final_descent_approach` 改为固定 `mode=0` 直驱前进，同时发布路径起终点参考线，锁存 `path_correction_profile=2`，并放宽该点的 pass-through 横向容差；最终朝向仍交给下一个 `final_heading_align` 单独处理。
- 回退方式：`obstacle_waypoints_mirror.yaml` 恢复 `final_descent_approach.path_correction_profile_on_arrival=0`，删除新增的 `direct_step_override` / `publish_path_reference*` / `pass_through_lateral_tolerance` 字段，即回到普通 `/goal_pose` 跟踪。

### 2026-05-22
- 修改目的：AprilTag 到点逻辑本身已能工作，但总状态机日志反复出现 `image_raw` 与 `camera_info` 配对稀疏，导致有效样本窗口增长太慢。
- 关键行为变化：`obstacle_race_visual_supervisor.launch.py` 里 D435 的 `color/depth image` 与 `camera_info` 全部切到 `SENSOR_DATA` QoS；`tags_36h11.yaml` 的 AprilTag detector `decimate` 从 `1.0` 调到 `1.5`，优先换取更稳的处理回调频率。
- 回退方式：删掉 launch 里的 `color_qos/color_info_qos/depth_qos/depth_info_qos`，并把 `tags_36h11.yaml` 的 `detector.decimate` 改回 `1.0`。

### 2026-05-22
- 修改目的：当前总状态机终端输出几乎全是默认 `INFO` 色，`waypoint` 切换、到点成功、检测命中、串口高频调试行混在一起，不利于现场抓关键事件。
- 关键行为变化：新增共享日志着色 helper，并给 `obstacle_manager`、`nav_executor`、`cmd_vel_to_serial`、`slope_branch_detector`、`limit_bar_duck_test`、`orange_hurdle_jump_test`、`orange_pole_detector` 的关键日志按类别上色：成功、切换/发送、状态锁存、检测命中、未命中、串口与高频调试分别用不同 ANSI 颜色显示。
- 回退方式：删除 `auto_nav_pkg/log_style.py` 的调用；`cmd_vel_to_serial_node.cpp` 去掉 ANSI 前后缀即可恢复纯文本日志。

### 2026-05-22
- 修改目的：镜像 `turn_180_spin` 实测已进入第5点自旋，但 `dist≈0.19m` 时 `yaw_err` 仍长期在 `135deg` 左右，说明目标 yaw 不是按预期的站立后稳定基准计算，导致“趴下通过后虽然能站起，但自旋一直到不了点”。
- 关键行为变化：仅修改镜像 `turn_180_spin` 的 yaw 解析语义，新增 `body_yaw_mode: previous_target_delta`，让第5点目标 yaw 改为“沿用 `wall_jump_prep` 的稳定目标 yaw 再转 -85°”，不再用发送瞬间当前 pose yaw 做基准。
- 回退方式：删除 `obstacle_waypoints_mirror.yaml` 中 `turn_180_spin.body_yaw_mode`，恢复默认 `delta` 语义。

### 2026-05-22
- 修改目的：镜像上坡段虽然已经锁存 `path_correction_profile=3`，但实测 `path_lat_err=0.008~0.033m`、`path_yaw_err=1~2deg` 时仍处于死区内，导致机身先偏出斜坡边缘再开始修正。
- 关键行为变化：仅调整上坡专用修正参数，不改普通段逻辑；`uphill_path_lateral_correction_deadband_m` 从 `0.05` 收紧到 `0.03`，并开启 `uphill_path_yaw_correction_enabled=true`，让上坡更早按目标线方向回正。
- 回退方式：`cmd_vel_to_serial.yaml` 恢复 `uphill_path_lateral_correction_deadband_m: 0.05`，并把 `uphill_path_yaw_correction_enabled` 改回 `false`。

### 2026-05-22
- 修改目的：`wall_jump_prep` 在 `15fps` 下仍经常只拿到 `2/2` 个 AprilTag 新鲜样本，无法满足 `4-of-5` 过滤，先恢复默认帧率验证样本连续性。
- 关键行为变化：`obstacle_race_visual_supervisor.launch.py` 默认 `color_profile`、`depth_profile` 从 `640x480@15` 改回 `640x480@30`；其余 AprilTag stale/filter 参数保持不变，便于单独观察帧率影响。
- 回退方式：把 launch 默认值改回 `640,480,15`。

### 2026-05-22
- 修改目的：`wall_jump_prep` 实测 `apriltag_z` 已低于阈值，但 `age≈0.61s` 时因 `stale_timeout=0.30` 被判过期，无法从趴下切站立。
- 关键行为变化：`wall_jump_prep` 的 `apriltag_arrival_stale_timeout_sec` 从 `0.30` 放宽到 `0.80`；AprilTag 检测配置改为 `qos_profile=sensor_data`、`detector.threads=2`，优先提高更新连续性和时效性。
- 回退方式：`stale_timeout` 改回 `0.30`；AprilTag 配置改回 `qos_profile=default`、`threads=1`。

### 2026-05-22
- 修改目的：D435 在总状态机运行时连续出现 `UVCIOC_CTRL_QUERY Protocol error`，曾短暂回退默认相机帧率排查是否为链路时序余量不足。
- 关键行为变化：该回退测试已结束，默认帧率当前重新恢复为 `640x480@30`。
- 回退方式：如需再次压低链路负载，把 launch 默认值改回 `640,480,15`。

### 2026-05-22
- 修改目的：排查“偏航是否消耗前进位移，导致 waypoint 到点距离和原量不一致”。
- 关键行为变化：`obstacle_manager` 进度日志新增 `line_progress`、`line_remaining`、`line_lateral`，用于区分沿目标线进度不足、横向偏移过大、还是单纯欧氏距离未到；本次只增强日志，不改变到点判定。
- 回退方式：删除 `obstacle_manager.py` 中 `_goal_line_progress_text()` 的日志拼接即可恢复旧日志格式。

### 2026-05-22
- 修改目的：上斜坡偶发走歪后难以修正，存在从侧面掉出的风险。
- 关键行为变化：上坡步长地板改为公共前进偏置，保留 path lateral/yaw 产生的左右差；同时增强上坡横向修正参数。
- 回退方式：`cmd_vel_to_serial_node.cpp` 恢复为左右腿分别抬到最小步长；`cmd_vel_to_serial.yaml` 恢复 `deadband=0.08, gain=0.4, max=0.08`。

### 2026-05-22
- 修改目的：降低 AprilTag PnP z 偶发跳变导致提前/延后站起的概率。
- 关键行为变化：AprilTag 检测 `decimate=1.0`；到点逻辑改为滑动中值 + N-of-M + hold。相机默认帧率已在后续排障中回退到 `640x480@15`。
- 回退方式：恢复 `detector.decimate=2.0`，并移除 waypoint 中 `apriltag_arrival_filter_*` 参数。

1.调试步骤跟之前一样，开三个终端窗口，一个启动雷达驱动，一个启动雷达建图，一个跑总状态机。
现在测斜坡障碍物放的顺序用这个命令启动总状态机：
ros2 launch auto_nav_pkg obstacle_race_visual_supervisor.launch.py   waypoints_file:=/home/gouzi/obstacle_race_src/src/auto_nav_pkg/config/obstacle_waypoints_mirror.yaml （如果是之前的放置斜坡位置，则不需要参数）

调试注意的东西：1.首先还是老问题，下完楼梯后机身位置不确定要对应给点位，现在这版点位应该是没有问题的，如果上斜坡有问题就对应改这个点- name: platform_center ：
 比如说： - label: near_right
        min_inclusive: 0.00
        max_inclusive: 0. 
        body_left_m: -0.18
比如说这个，它会根据下完楼梯后会有一段对齐，然后对齐完走0.2m然
后开始识别进入斜坡分支，这一时刻，如果它的位置x轴代表左右，
处在0.00-0.05之间，那么就要给点left为0.18。如果上斜坡有问题，就停止代码，然后搜索“解析机身”，按两次下，就是说第二个，会有关键词说pose_switch="  "，然后对应改即可

2.这个问题很奇怪，我跑原来的代码没有这个问题，一跑这个镜像代码就出现这个问题。上完斜坡后要自旋，自旋完要趴下，然后我是给趴下走的这段加了yaw角的朝向修正，没开机身修正。然后我给的角度是-90度，这样子它自旋完是歪的，角度偏大，但是基准线是对的，所以最终也能修回去，会浪费几秒时间。我记得我之前给codex说这里的基准线是要改成自旋完那一时刻的角度。我跑没有镜像的代码是给的87度，这个角度是刚好自旋完是正的，然后基准线也是正的，很奇怪，昨晚一晚上都是这样，看你操作。

3.这个小问题就是深度相机深度值识别apriltag码出现深度值偏差，会差几厘米，5次可能会出现2次左右，这个我还没改，测的时候有时候识别不到站不起来是位置机身偏了，扫不到，到时候将码放远一点然后改个阈值即可。

4.还有一个小问题就是识别完码站起来自旋，然后走木桥，这里是最不稳的，狗子有时候可以走直线有时侯走直线会歪，这里我用的是邪修，改自旋角度值，根据它走的路径，如果它走歪了就故意自旋多一点或者少一点（要看狗子歪那边），还有可能是顺姐说自旋完启动走木桥的那一刻，因为步长突变，所以才导致狗子歪了一下，才导致后面走歪。这里可以加滤波系数，不要突变试试，我还没改，交给你。

5.这个问题可解决可不解决，下楼梯问题，我感觉应该可以解决，就是下楼梯下稳，现在步长给的是0.8，10次下楼梯可能有三次是会摔的，或者歪很多，虽然有修正角度，但是也有可能导致斜坡识别不到，如果可以识别到斜坡，它走完0.2m会顿一下的，没识别到会直接走那就要重来。有时候会出现的情况就是下楼梯下的很好，但是也会出现相机识别不到斜坡的情况，这种情况一般5次有2次吧，我一般会拔插一下相机就好了，如果后续斜坡或者其他障碍物识别有问题，你可以上传识别时候的距离的图片让它改参数，或者说跑代码，然后去rviz看debug图，可以看看是什么东西导致它误识别，也可以复制日志去问codex是什么原因没识别到，可能是面积啥的。

6.最后一个就是在所有情况都调稳的状况下把速度拉快，看看那里可以拉快就拉快。
