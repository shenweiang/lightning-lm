# Lightning-LM

激光建图+定位模块，内部使用手册。

## 功能

- 3D Lidar SLAM：快速 LIO 前端（FasterLIO）+ 回环检测
- 3D→2D 栅格地图转换（g2p5），可选
- 地图分区动态加载，适用大场景
- NDT-OMP 激光定位 + 多源融合位姿图优化（PGO）
- RTK/INS 全链路融合：前端 ESKF + 建图后端 LoopClosing + 定位后端 PGO
- IMU→base_link 杆臂补偿，支持 LeiShen（镭神）雷达
- 离线/在线双模式，离线可断点调试
- 高频率 IMU 平滑输出（100Hz），平滑因子可调
- 轻量优化库 miao（基于 g2o，更轻更快，支持增量优化）
- CPU 占用低：在线定位 ~0.8 核，建图 ~1.2 核

## 编译

### 环境

Ubuntu 22.04 + ROS2 Humble。

### 安装依赖

```bash
./scripts/install_dep.sh
```

### 编译 Pangolin（首次）

```bash
cd thirdparty
unzip Pangolin-0.9.3.zip -d Pangolin
cd Pangolin/Pangolin-0.9.3 && mkdir build && cd build
sudo apt install -y libepoxy-dev
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_FFMPEG=OFF
make -j$(nproc) && sudo make install
```

### 编译项目

```bash
cd ~/lightning-lm_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lightning
source install/setup.bash

# 无 Livox（默认）
colcon build --cmake-args -DUSE_LIVOX=OFF --packages-select lightning

# Debug 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lightning
```

## 建图测试

### 离线建图（推荐，更快）

```bash
ros2 run lightning run_slam_offline \
  --config ./config/default.yaml --input_bag <bag路径>
```

结束后自动保存至 `data/new_map/`，含全局点云 `global.pcd` 和栅格地图 `map.pgm`。

### 在线建图

```bash
ros2 run lightning run_slam_online --config ./config/default.yaml

# 保存地图
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

## 定位测试

### 离线定位

```bash
ros2 run lightning run_loc_offline \
  --config ./config/default.yaml --input_bag <bag路径>
```

### 在线定位

```bash
ros2 run lightning run_loc_online --config ./config/default.yaml
```

定位程序发布 `map→base_link` TF（与 IMU 同频，50-100Hz），接收 `geometry_msgs::TransformStamped` 即可获取定位结果。

### 配置要点

1. 地图路径：YAML 中 `system.map_path`，默认 `new_map`
2. 建图时自动保存 `georeference.yaml`（含 LLA+UTM 原点），定位时优先加载以确保 RTK 坐标一致
3. 雷达类型：`fasterlio.lidar_type`（1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense, 5=LeiShen）
4. 话题：`common.lidar_topic`、`common.imu_topic`、`common.rtk_topic`
5. 先用离线模式调参，通过后再调在线

## 参数说明

### ESKF（`fasterlio`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lidar_type` | 2 | 雷达类型 1-5 |
| `time_scale` | 0.001 | 时间戳缩放（Velodyne 微秒→毫秒，LeiShen 不适用） |
| `point_filter_num` | 1 | 降采样间隔，调大减少点数 |
| `imu_filter` | false | IMU 低通滤波（斜装应关闭） |
| `use_aa` | false | Anderson 加速 |
| `enable_icp_part` | true | 点到点 ICP（辅助点到面） |
| `icp_weight` | 100 | 点到点 ICP 权重 |
| `extrinsic_T` | [0, 0, 0.28] | 雷达在 IMU 下的平移（LeiShen/merge_cloud 场景 = `-T_imu_base`） |
| `extrinsic_R` | I | 雷达在 IMU 下的旋转 |
| `b_acc_cov` | 0.0001 | 加速度计零偏过程噪声 |
| `b_gyr_cov` | 0.0001 | 陀螺仪零偏过程噪声 |
| `kf_dis_th` | 2.0 | 关键帧位移阈值 (m) |
| `kf_ang_th` | 15° | 关键帧角度阈值 |
| `rtk_noise_ratio` | 1.0 | RTK 观测噪声比例（>1 降低权重） |
| `rtk_rot_noise` | 0.0052 | RTK 姿态观测噪声 (rad)，≈0.3° |
| `keep_first_imu_estimation` | false | 保留第一帧 IMU 估计的偏置 |

### 回环检测（`loop_closing`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `with_height` | true | 高度约束（多层室内应关闭） |
| `loop_kf_gap` | 20 | 回环检查间隔（关键帧数） |
| `rtk_pos_noise` | 0.1 | RTK 位置噪声回退值 (m) |
| `rtk_ang_noise` | 0.0052 | RTK 姿态噪声回退值 (rad) |
| `rtk_outlier_th` | 10.0 | RTK 异常值卡方阈值 |

### 位姿图优化 PGO（`pgo`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `smooth_factor` | 0.01 | 输出位姿平滑因子 |
| `lidar_loc_pos_noise` | 0.3 | NDT 定位位置噪声 (m) |
| `lidar_loc_ang_noise` | 2.0° | NDT 定位姿态噪声 |
| `lidar_loc_outlier_th` | 10.0 | NDT 异常值卡方阈值 |
| `lidar_odom_pos_noise` | 0.3 | LIO 里程计位置噪声 (m) |
| `pgo_frame_converge_pos_th` | 0.05 | 滑窗帧收敛位置阈值 (m) |
| `pgo_frame_converge_ang_th` | 1.0° | 滑窗帧收敛角度阈值 |
| `rtk_pos_noise` | 0.1 | RTK 位置噪声回退值 (m) |
| `rtk_ang_noise` | 0.0052 | RTK 姿态噪声回退值 (rad) |
| `rtk_outlier_th` | 10.0 | RTK 异常值卡方阈值 |

### 系统（`system`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `with_ui` | false | 3D Pangolin 可视化 |
| `with_2dui` | false | 2D 栅格显示 |
| `with_loop_closing` | true | 回环检测 |
| `with_g2p5` | false | 栅格地图输出 |
| `map_path` | `new_map` | 地图存储/加载路径 |

### RTK（`common`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rtk_topic` | `/sensing/ins/ins_raw` | INS 融合位姿话题 |
| `T_imu_base` | [0, 0, 0] | IMU→base_link 杆臂 (m)，前/左/上 |

### 雷达 ROI（`roi`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `height_max` | 10.0 | 点云高度上限 (m) |
| `height_min` | -2.0 | 点云高度下限 (m) |

## 传感器适配

1. 确认雷达类型，设置 `fasterlio.lidar_type`（1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense, 5=LeiShen）
2. 配置 `common.lidar_topic`、`common.imu_topic`
3. 检查 `fasterlio.time_scale`（LeiShen 不适用）
4. IMU 和雷达外参默认为零即可，对系统不敏感
5. 离线模式参数调试 → 在线模式验证
6. 有 RTK 时配置 `common.rtk_topic` 和 `common.T_imu_base`，建图后 `georeference.yaml` 自动保存
