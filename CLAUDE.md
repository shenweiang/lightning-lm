# 🤖 Lightning-LM 项目宪法

## 1. 项目上下文

- **项目名称**：Lightning-LM（Lightning-Speed Lidar Localization and Mapping）
- **核心功能**：3D 激光 SLAM + 定位：LIO 前端、回环检测、地图管理、NDT 激光定位、位姿图优化
- **ROS2 版本**：Humble（`package.xml` 中 `buildtool_depend ament_cmake`，Docker 基础镜像 `ros:humble-perception`）
- **C++ 标准**：C++17（`CMAKE_CXX_STANDARD 17`）
- **构建系统**：CMake 3.16+，`ament_cmake`，Colcon 编排
- **代码规模**：约 27,000 行 C++
- **代码仓库**：GitHub `gaoxiang12/lightning-lm`

### 第三方依赖

| 类别 | 库 | 来源 |
|------|-----|------|
| 优化/数学 | Eigen3, Sophus, miao（自研） | Sophus 内置 `thirdparty/`，miao 内置 `src/core/miao/` |
| 点云/地图 | PCL, pcl_conversions, pcl_ros | apt (`ros-humble-*`) |
| 可视化 | Pangolin, OpenGL | `thirdparty/Pangolin-0.9.3.zip` 源码编译 |
| 通信 | ROS2 原生, tf2, rosbag2 | apt |
| 日志/工具 | glog, gflags, yaml-cpp | apt |
| 图像 | OpenCV | apt |
| 并行 | OpenMP, TBB | apt |

🚫 未经确认禁止新增依赖。

## 2. 项目目录结构

```
lightning-lm/
├── CMakeLists.txt              # 顶层 CMake，项目名 lightning
├── package.xml                 # ROS2 包描述
├── .clang-format               # Google Style 变体（4 空格缩进，120 列宽）
├── config/                     # YAML 配置文件
│   ├── default.yaml            # 默认配置
│   ├── default_nclt.yaml / default_vbr.yaml / default_livox.yaml
│   ├── default_robosense.yaml / default_utbm.yaml
├── cmake/packages.cmake        # find_package 和全局编译选项
├── scripts/                    # install_dep.sh, merge_bags.py 等
├── tests/rtk_utils_test.cc     # LLA→UTM 单元测试
├── docker/                     # Dockerfile (ros:humble-perception)
├── srv/                        # LocCmd.srv, SaveMap.srv
├── thirdparty/                 # Sophus, Pangolin, livox_ros_driver
└── src/
    ├── CMakeLists.txt
    ├── common/                  # 基础数据结构
    │   ├── eigen_types.h        # Eigen/Sophus 类型别名 (SE3, SO3, Vec3d 等)
    │   ├── nav_state.h/cc       # 导航状态（12 维：pos/rot/vel/bg）
    │   ├── imu.h                # IMU 数据结构
    │   ├── keyframe.h           # 关键帧（含 LIO/优化位姿 + RTK 关联）
    │   ├── odom.h               # 里程计数据
    │   ├── point_def.h          # 点云定义（含 LeiShenPoint）
    │   ├── constant.h           # 常量
    │   ├── params.h / options.h/cc  # 参数/配置
    │   ├── timed_pose.h         # 带时间戳位姿
    │   ├── measure_group.h      # 测量组（IMU + RTK + 激光扫描）
    │   ├── rtk_data.h           # RTK 观测数据结构体
    │   ├── rtk_utils.h/cc       # LLA→UTM 转换 + RTKConverter + MapOrigin
    │   ├── functional_points.h  # 功能点
    │   ├── loop_candidate.h     # 回环候选
    │   ├── pose_rpy.h / s2.hpp / std_types.h
    ├── io/                      # 文件 IO
    │   ├── yaml_io.cc/h         # YAML 读取
    │   ├── file_io.cc/h         # 地图保存/加载
    │   └── dataset_type.h       # 数据集类型枚举
    ├── wrapper/                 # ROS2 封装层（算法与 ROS 的边界）
    │   ├── bag_io.cc/h          # ROS2 Bag 读写
    │   └── ros_utils.h          # ROS2 工具函数
    ├── utils/                   # 通用工具
    │   ├── timer.cc/h, pointcloud_utils.cc/h
    │   ├── sync.h, async_message_process.h
    ├── ui/                      # Pangolin 3D/2D 可视化
    │   ├── pangolin_window.cc/h/_impl
    │   └── ui_car.cc/h, ui_cloud.cc/h, ui_trajectory.cc/h
    ├── core/                    # 核心算法（ROS-free）
    │   ├── lightning_math.hpp   # 通用数学函数
    │   ├── lio/                 # LIO 前端
    │   │   ├── eskf.hpp/cc      # ESKF（12 维误差状态卡尔曼滤波）
    │   │   ├── laser_mapping.h/cc   # 激光里程计（点到面/点到点 ICP）
    │   │   ├── imu_processing.hpp   # IMU 预积分 + 去畸变 + RTK 穿插
    │   │   ├── imu_filter.h         # IMU 低通滤波
    │   │   ├── pointcloud_preprocess.h/cc  # 点云预处理（5 种雷达格式）
    │   │   ├── pose6d.h / anderson_acceleration.h
    │   ├── ivox3d/              # iVox 3D 空间索引（Hilbert 曲线）
    │   ├── loop_closing/        # 回环检测 + 位姿图优化
    │   ├── g2p5/                # 3D→2D 栅格地图（地面分割+投影）
    │   ├── maps/                # 瓦片地图管理（动态加载/卸载）
    │   ├── localization/        # 激光定位
    │   │   ├── localization.h/cpp        # 定位主逻辑
    │   │   ├── localization_result.h/cc  # 定位结果 + TF 输出
    │   │   ├── lidar_loc/                # NDT-OMP 地图匹配
    │   │   │   └── pclomp/               # OpenMP 加速 NDT + VoxelGrid
    │   │   └── pose_graph/               # 位姿图优化 (PGO)
    │   │       ├── pgo.h/cc/_impl.h       # 多源融合 PGO
    │   │       ├── pose_extrapolator.h/cc  # 位姿外推
    │   │       └── smoother.h             # 平滑器
    │   ├── miao/                # 自研轻量图优化库
    │   │   ├── core/            # graph/, solver/, sparse/, opti_algo/, robust_kernel/, types/, math/
    │   │   ├── utils/ / examples/
    │   └── system/              # 系统调度
    │       ├── slam.h/cc        # 建图系统
    │       └── loc_system.h/cc  # 定位系统
    └── app/                     # 可执行程序入口
        ├── run_slam_online.cc / run_slam_offline.cc
        ├── run_loc_online.cc / run_loc_offline.cc
        ├── run_frontend_offline.cc / run_loop_offline.cc
        └── test_ui.cc
```

## 3. 坐标系定义

> **Debug 核心原则**：排查任何位姿/点云偏移问题时，第一步就是确认数据当前处于哪个坐标系。

### 3.1 坐标系一览

| 坐标系 | 符号 | 含义 | 数据示例 |
|--------|------|------|----------|
| **sensor** | `lidar` | 雷达原始测量坐标系 | 原始点云 `P_i`（`PointCloud2` 中的 xyz） |
| **imu_link** | `I` | IMU 敏感轴坐标系 | ESKF 状态、LIO 地图、IMU 加速度/角速度 |
| **base_link** | `B` | 车体中心坐标系（后轴中心投影到地面） | RTK 输出、TF `child_frame_id` |
| **world (ENU)** | `W` | 局部 ENU 米制坐标系，原点由 RTK 初始位置确定 | 所有绝对位姿、PGO 优化位姿、全局地图 |
| **UTM** | — | WGS84 椭球 UTM 投影 | RTK 转换中间量 |
| **LLA** | — | WGS84 经纬高 | RTK 原始消息（`nav_msgs/Odometry` 的 position 字段） |

### 3.2 核心变换关系

```
物理安装:
            ┌── extrinsic(T_imu_lidar) ──→ sensor
  imu_link ─┤
            └── T_imu_base⁻¹ ──→ base_link
              (base_link 在 imu_link 下的位姿)

ESKF 状态表示:
  x.pos, x.rot = T_W_I   (imu_link 在 world 下的位姿)

位姿链:
  T_W_sensor = T_W_I · T_I_sensor   (extrinsic)
  T_W_B      = T_W_I · T_I_B        (T_I_B = T_imu_base⁻¹)
```

### 3.3 关键变量与坐标系的对应关系（Debug 速查表）

| 变量 | 类型 | 所在坐标系 | 说明 |
|------|------|-----------|------|
| `pointcloud_preprocess` 输出 | `PointType` (x,y,z) | **sensor** 系 | 去畸变前，仍在雷达/merge_cloud 坐标系 |
| `scan_undistort_` | `PointType` (x,y,z) | **sensor** 系 | 去畸变后，仍在 sensor 系 |
| `NavState::pos_` | `Vec3d` | **world (ENU)** | ESKF 估计的 IMU 在 world 下的位置 |
| `NavState::rot_` | `SO3` | **world→imu_link** | 将 imu_link 向量转到 world 的旋转矩阵 `R_W_I` |
| `kf_->GetLIOPose()` | `SE3` | **world (ENU)** | 关键帧的 LIO 位姿：`T_W_I` |
| `kf_->GetOptPose()` | `SE3` | **world (ENU)** | 回环优化后的位姿：`T_W_I` |
| `current_rtk_.position` | `Vec3d` | **world (ENU)** | RTK 观测的 IMU 位置（已过杆臂补偿） |
| `current_rtk_.orientation` | `SO3` | **world→imu_link** | RTK 观测的 IMU 姿态 |
| `PGOFrame::rtk_pose_` | `SE3` | **world (ENU)** | 插值后的 RTK 位姿 |
| `PGOFrame::lidar_loc_pose_` | `SE3` | **world (ENU)** | NDT 定位结果 |
| `LocalizationResult::pose_` | `SE3` | **world (ENU)** | 定位输出位姿（imu_link→world） |
| `extrinsic_T/R` | `Vec3d, Mat3d` | **sensor→imu_link** | 雷达在 IMU 下的外参 `T_I_sensor` |
| `offset_R_lidar_fixed_` | `Mat3d` | **sensor→imu_link** | 等同于 `R_I_sensor` |
| `offset_t_lidar_fixed_` | `Vec3d` | **imu_link 系下的位移** | `t_I_sensor`，sensor 原点在 imu_link 下的位置 |
| `T_imu_base_` | `SE3` | **base_link→imu_link** | IMU 在 base_link 下的位姿（YAML `common.T_imu_base`）|

### 3.4 两种 sensor 场景的区别

| 场景 | sensor 坐标系 | extrinsic 语义 | YAML 配置 |
|------|-------------|---------------|-----------|
| **标准 LiDAR 直连** | 雷达坐标系 | `T_I_lidar` | `extrinsic_T/R` = LiDAR 在 IMU 下的外参 |
| **LeiShen/merge_cloud** | **base_link**（融合节点已将点云转到车体） | `T_I_base` | `extrinsic = T_imu_base⁻¹` |

> **Debug 检查**：如果 LeiShen 场景下 extrinsic 配错了（比如配成了单位阵），去畸变会把 base_link 系点云当成 sensor 系来补偿，导致运动补偿方向反了或量级错了。

---

## 4. 建图模式（SLAM）详细数据流

### 4.1 在线建图 — 逐帧流程

```
┌─ 回调层 (ROS2, wrapper/) ────────────────────────────────────────────┐
│                                                                       │
│  IMU msg ──→ SlamSystem::ProcessIMU()                                 │
│    sensor_msgs::msg::Imu                                               │
│    │ 类型转换: imu->angular_velocity / linear_acceleration             │
│    │ 坐标系: imu_link (IMU 测的是安装点处的加速度/角速度)              │
│    ▼                                                                  │
│    imu_buffer_.emplace_back()                                          │
│                                                                       │
│  LiDAR msg ──→ SlamSystem::ProcessLidar()                             │
│    sensor_msgs::msg::PointCloud2                                       │
│    │ 预处理: pointcloud_preprocess (格式转换 + 盲区过滤 + 降采样)      │
│    │ 坐标系: sensor (雷达/merge_cloud) → 输出仍在 sensor 系            │
│    │ time: 各格式统一为 double 毫秒                                    │
│    ▼                                                                  │
│    lidar_buffer_.push_back()                                           │
│                                                                       │
│  RTK msg ──→ SlamSystem::ProcessRTK()                                 │
│    nav_msgs::msg::Odometry (position=LLA, orientation=ENU)             │
│    │ RTKConverter::Convert():                                          │
│    │   ① LLA→UTM (WGS84 椭球投影)                                     │
│    │   ② 中值滤波(10帧) → origin_UTM (只在未初始化时执行)              │
│    │   ③ ENU = UTM - origin_UTM                                        │
│    │   ④ 杆臂补偿: p_imu = p_base + R_base * t                        │
│    │      (将 RTK 位姿从 base_link 转到 imu_link)                      │
│    │   ⑤ 跳变检测: |Δp|/dt > 10 m/s 则跳过                            │
│    │ 输出坐标系: imu_link 在 world(ENU) 下的位姿                       │
│    ▼                                                                  │
│    lio_->ProcessRTK(rtk) → rtk_buffer_.emplace_back()                  │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘

┌─ LIO 前端 (core/lio/) ───────────────────────────────────────────────┐
│                                                                       │
│  LaserMapping::Run()                                                   │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ① SyncPackages()                                                 │ │
│  │    . 取 lidar_buffer_.front() → measures_.scan_ (sensor 系)      │ │
│  │    . 计算 lidar_end_time_ (scan_开始时间 + 最后点的时间偏移)      │ │
│  │    . 弹出 imu_buffer_ 中 timestamp < lidar_end_time_ 的所有IMU   │ │
│  │      → measures_.imu_ (imu_link 系)                               │ │
│  │    . 收集 [lidar_begin, lidar_end] 内的 RTK → measures_.rtk_     │ │
│  │      (world ENU 系)                                              │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ② ImuProcess::Process() → UndistortPcl()                         │ │
│  │                                                                  │ │
│  │   a. 将上帧末尾 IMU 插入队列头，逐对遍历 IMU [head, tail]        │ │
│  │                                                                  │ │
│  │   b. RTK 穿插更新 (先 Update 再 Predict):                         │ │
│  │      while rtk.timestamp ∈ [head, tail):                          │ │
│  │        current_rtk_ = rtk     ← 存入成员变量 (world ENU 系)       │ │
│  │        kf_.Update(GPS)         ← RTK 观测约束 ESKF               │ │
│  │      kf_.Predict(head→tail)    ← IMU 递推                        │ │
│  │                                                                  │ │
│  │   c. 保存每个 IMU 时刻的位姿到 imu_pose_[]:                       │ │
│  │      imu_state = kf_.GetX()    ← 全部在 world(ENU) 系            │ │
│  │                                                                  │ │
│  │   d. 逐点去畸变 (反向遍历):                                       │ │
│  │      for each point (time = t_i, 坐标 P_i 在 sensor 系):         │ │
│  │        ┌─ 在当前 IMU 位姿间插值得到 R_i, T_ei (world 系)         │ │
│  │        │  P_compensate =                                            │ │
│  │        │    R_I_sensor^T · [ R_W_I^{-1} · (R_i · (R_I_sensor·P_i  │ │
│  │        │                      + t_I_sensor) + T_ei) - t_I_sensor ] │ │
│  │        │  ① R_I_sensor·P_i + t_I_sensor: sensor→imu_link         │ │
│  │        │  ② R_i * (...) + T_ei:       imu_link→world             │ │
│  │        │  ③ R_W_I^{-1} * (...):       world→imu_link (frame-end) │ │
│  │        │  ④ R_I_sensor^T·(... - t):   imu_link→sensor            │ │
│  │        └─ 输出: scan_undistort_ (sensor 系，已补偿运动)           │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ③ 体素降采样: scan_undistort_ → scan_down_body_ (sensor 系)      │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ④ kf_.Update(ObsType::LIDAR) → ObsModel()                        │ │
│  │                                                                  │ │
│  │    对降采样后每个点 (sensor 系):                                  │ │
│  │      a. 转到世界系:                                                │ │
│  │         p_world = R_W_I · (R_I_sensor · p_sensor + t_I_sensor)   │ │
│  │                  + t_W_I                                          │ │
│  │         ★ 用当前 ESKF 的 rot_/pos_ 作为 R_W_I / t_W_I            │ │
│  │      b. 在 iVox 地图 (world 系) 中搜索 5 个最近邻点               │ │
│  │      c. 估计平面 (RANSAC)，计算点到面残差                         │ │
│  │      d. 计算雅可比 J (关于 pose 的 6 维导数)                      │ │
│  │      e. HTH += J^T·J, HTr += J^T·r  (OpenMP 并行累加)            │ │
│  │                                                                  │ │
│  │    ESKF::Update() 内部:                                           │ │
│  │      . 迭代: 每次更新 x_ → 重新计算 ObsModel (重线性化)           │ │
│  │      . 退化检测: eigen(HTH) → 低特征值方向投影掉                  │ │
│  │      . 更新限幅: Δtrans ≤0.5m, Δrot ≤5°, Δvel ≤2m/s              │ │
│  │      . 收敛后 P 阵对称化+裁剪                                     │ │
│  │                                                                  │ │
│  │    输出: state_point_ = kf_.GetX() (world ENU 系, imu_link 位姿) │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ⑤ MapIncremental()                                               │ │
│  │    将 scan_down_body_ 和 nearest_points_ 比较，判断是否需要       │ │
│  │    向 iVox 地图 (world 系) 添加新点                               │ │
│  │    ★ 输入: sensor 系点 + ESKF 位姿 → 转到 world 系存储           │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ⑥ MakeKF()                                                       │ │
│  │    . 条件: |Δpos| > 2m 或 |Δangle| > 15°                         │ │
│  │    . 存储: scan_undistort_ (sensor 系) + state_point_ (world 系) │ │
│  │    . 关联 current_rtk_ (|dt| < 0.5s)                              │ │
│  │    . 初始 opt_pose_ = lio_pose_                                   │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘

┌─ 回环检测 & 后端优化 (core/loop_closing/) ───────────────────────────┐
│                                                                       │
│  LoopClosing::HandleKF()                                              │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ① DetectLoopCandidates()                                         │ │
│  │    遍历历史关键帧, 距离 < 30m → 候选                              │ │
│  │    坐标系: 用 opt_pose_ (world ENU 系) 判断距离                  │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ② ComputeLoopCandidates()                                        │ │
│  │    多分辨率 NDT: 10m→5m→2m→1m                                    │ │
│  │    以 opt_pose_ 为初值 (world 系)                                 │ │
│  │    输出: T_ij (相对位姿约束，SE3 在 world 系下的相对变换)         │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ③ PoseOptimization()                                             │ │
│  │    顶点: opt_pose_ (world ENU 系)                                 │ │
│  │    运动边: LIO 帧间相对位姿 (world 系下两帧的 SE3 差)            │ │
│  │    回环边: NDT 配准的相对约束                                     │ │
│  │    RTK 边: EdgeSE3Prior (绝对位姿, world ENU 系)                  │ │
│  │    高度先验: z≈0 (平面场景)                                       │ │
│  │                                                                  │ │
│  │    优化后: 写回 kf->SetOptPose(pose) (world ENU 系)              │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘

┌─ 保存地图 ───────────────────────────────────────────────────────────┐
│  SaveMap()                                                            │
│    遍历 all_keyframes_:                                               │
│      p_world = opt_pose_ * p_sensor  (sensor→world 变换)             │
│    合并为 global.pcd (world ENU 系)                                   │
│    ★ 保存 georeference.yaml (LLA + UTM 原点，用于定位模式恢复)       │
└───────────────────────────────────────────────────────────────────────┘
```

### 4.2 离线建图 — 差异

- 不使用 ROS2 回调，通过 `RosbagIO` 链式调用逐条读取 bag 消息
- IMU/LiDAR/RTK 回调函数与在线模式相同，同步逻辑不变
- 坐标系路径与在线模式完全一致

---

## 5. 定位模式（Localization）详细数据流

### 5.1 在线定位 — 逐帧流程

```
┌─ 回调层 (LocSystem) ─────────────────────────────────────────────────┐
│  同 SLAM 模式:                                                        │
│  IMU  → imu_buffer_     (imu_link 系)                                 │
│  LiDAR → lidar_buffer_  (sensor 系, 经预处理)                         │
│  RTK  → RTKConverter → lio_->ProcessRTK() + pgo_->ProcessRTK()        │
│        (world ENU 系, 已过杆臂补偿)                                   │
│                                                                       │
│  Init() 时: 尝试从 map_path/georeference.yaml 加载原点               │
│    成功 → 跳过运行时中值滤波，直接使用建图时的原点                    │
│    失败 → 运行时中值滤波初始化原点 (要求定位起点≈建图起点)            │
└───────────────────────────────────────────────────────────────────────┘

┌─ Localization 调度 (core/localization/localization.cpp) ─────────────┐
│                                                                       │
│  ProcessLidarMsg(cloud)                                               │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ① LidarOdomProcCloud(cloud)     ← 异步线程                       │ │
│  │    cloud 坐标系: sensor                                          │ │
│  │    . preprocess_ (格式转换，输出: sensor 系)                      │ │
│  │    . lio_->ProcessPointCloud2(cloud) → lidar_buffer_              │ │
│  │    . lio_->Run() → [SyncPackages → ImuProcess → ESKF Update]     │ │
│  │    ★ LIO 前端流程与 SLAM 模式完全相同                             │ │
│  │    ★ 区别: 定位模式不建关键帧、不维护 iVox 地图                   │ │
│  │    输出:                                                          │ │
│  │      - lio_kf_ (当前帧关键帧, sensor 系点云 + world 系位姿)       │ │
│  │      - NavState (world ENU 系, imu_link 位姿)                    │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ② LidarLocProcCloud(cloud)      ← 异步线程                       │ │
│  │    cloud 坐标系: sensor                                          │ │
│  │    . LidarLoc::ProcessCloud(cloud)                                │ │
│  │    . LidarLoc::ProcessLO(nav_state)  ← LIO 位姿作为 NDT 初值     │ │
│  │                                                                  │ │
│  │    LidarLoc::Align(input):                                        │ │
│  │      input 坐标系: sensor (当前扫描)                              │ │
│  │      map 坐标系: world ENU (TiledMap 加载的 pcd)                  │ │
│  │      a. 用 LIO 位姿 W_T_I 作为 NDT 初值                           │ │
│  │      b. pcl_ndt_->align(*output, W_T_I.matrix())                  │ │
│  │         NDT 内部: 将 sensor 系点云按初值转到 world 系做匹配       │ │
│  │      c. 输出: current_abs_pose_ (world ENU 系, imu_link 位姿)    │ │
│  │                                                                  │ │
│  │    初始化:                                                        │ │
│  │      . 栅格搜索 yaw (360° 范围，按步长搜索)                       │ │
│  │      . 功能点初始化 (init_with_fp)                                 │ │
│  └──────────────────────────────────────────────────────────────────┘ │
│  ┌──────────────────────────────────────────────────────────────────┐ │
│  │ ③ PGO 融合                                                       │ │
│  │                                                                  │ │
│  │    PGO::ProcessLidarOdom(lio_result)   ← NavState (world 系)     │ │
│  │      → lidar_odom_pose_queue_ (帧间相对约束用)                    │ │
│  │                                                                  │ │
│  │    PGO::ProcessLidarLoc(loc_result)     ← 触发优化                │ │
│  │      → BuildProblem():                                            │ │
│  │          顶点: opti_pose_ (world ENU 系)                          │ │
│  │          LidarLoc 边: EdgeSE3Prior (绝对位姿, world 系)           │ │
│  │          RTK 边:      EdgeSE3Prior (绝对位姿, world 系)           │ │
│  │          LidarOdom 边: EdgeSE3 (帧间相对, world 系)               │ │
│  │          Prior 边:    边缘化先验                                  │ │
│  │      → RunOptimization(5次迭代)                                   │ │
│  │      → 输出: LocalizationResult::pose_ (world ENU 系, imu_link)   │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘

┌─ 输出层 (TF 发布) ───────────────────────────────────────────────────┐
│                                                                       │
│  LocalizationResult::ToGeoMsg(T_imu_base)                             │
│                                                                       │
│  ┌─ 杆臂补偿 (输出侧) ────────────────────────────────────────────┐  │
│  │                                                                  │ │
│  │  pose_ 是 T_W_I (imu_link 在 world 下的位姿)                     │ │
│  │                                                                  │ │
│  │  若 T_imu_base 非 Identity:                                      │ │
│  │    pose_out = pose_ * T_imu_base.inverse()                       │ │
│  │    → T_W_B = T_W_I * T_I_B                                       │ │
│  │                                                                  │ │
│  │  展开: p_base = p_imu - R_imu * t_imu_in_base                   │ │
│  │        (当旋转分量为 Identity 时)                                 │ │
│  │                                                                  │ │
│  │  输出: geometry_msgs::TransformStamped                            │ │
│  │    header.frame_id = "map" (world ENU)                           │ │
│  │    child_frame_id  = "base_link"                                  │ │
│  └──────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘
```

### 5.2 RTK 数据流（SLAM & 定位共用）

```
RTK 原始消息 (nav_msgs::msg::Odometry)
  │
  │ position.x = latitude (°), position.y = longitude (°)
  │ position.z = altitude (m)
  │ orientation = ENU 姿态 (四元数)
  │ covariance[0] = east_std², [7] = north_std², [8] = up_std²
  │
  ├─→ RTKConverter::Convert()
  │     ① LLA → UTM:     WGS84 椭球投影 → (easting, northing, alt, zone)
  │     ② 原点初始化:     中值滤波 10 帧 → origin_utm_
  │        若 georeference.yaml 存在则直接从文件加载原点
  │     ③ UTM → ENU:     enu = utm - origin_utm_
  │     ④ 杆臂补偿:       p_imu = p_base + R_base * t_imu_in_base
  │        输入: RTK 位姿在 base_link
  │        输出: RTKData 在 imu_link
  │     ⑤ 跳变检测:       |Δp| / dt > 10 m/s → WARNING + 跳过
  │
  ├─→ SLAM 模式: lio_->ProcessRTK(rtk) → rtk_buffer_
  │      └→ SyncPackages() 收集 → UndistortPcl() 穿插 Update(GPS)
  │      └→ MakeKF() 关联 current_rtk_
  │      └→ LoopClosing::AddRTKFactors()
  │
  └─→ 定位模式:
        ├→ lio_->ProcessRTK(rtk)    ← ESKF 前端 (同 SLAM)
        └→ pgo_->ProcessRTK(rtk)    ← rtk_pose_queue_ 入队
             └→ AssignRTKPoseIfNeeded(frame)
                  时间插值 (位置线性 + 姿态 slerp)
                  噪声取 best_match (最近帧, 不参与插值)
             └→ AddRTKFactors() → EdgeSE3Prior + Cauchy 核
```

### 5.3 RTK 观测模型 (GPSObsModel)

```
GPSObsModel(NavState& s, CustomObservationModel& obs):
  ★ s 是 ESKF 当前状态 (与所有观测源一致的坐标系)

  ① 输入校验:
     if (!position.allFinite() || !orientation.coeffs().allFinite())
       → obs.valid_ = false, return

  ② 位置残差 (ENU 坐标系, 直接向量差):
     r_pos = rtk.position - s.pos       (world ENU 系下 3D 差)

  ③ 姿态残差 (SO(3) Log 映射):
     delta_rot = s.rot⁻¹ * rtk.orientation
     r_rot = delta_rot.log()
     if |r_rot| > π*0.8 → r_rot = 0    (防 π 附近奇异)

  ④ 信息加权 (H = I₆, 所以 H^T W H = W):
     info = 1 / max(σ², 1e-12)          (除零保护)
     HTH = diag(info_pos, info_rot)
     HTr = W * [r_pos; r_rot]

  ⑤ ESKF::Update(GPS):
     converge_ = true (线性模型, 一次收敛, 跳过迭代)
```

### 5.4 定位模式的坐标变换总览

```
输入阶段:
  IMU (imu_link)  ──────────────────────→ ESKF Predict (imu_link 状态递推)
  LiDAR 点云 (sensor) ──→ 去畸变(sensor) ──→ ICP 匹配 (转到 world)
                                          ──→ NDT 匹配 (转到 world)
  RTK (LLA) ──→ UTM ──→ ENU(base_link) ──→ 杆臂补偿 → ENU(imu_link)

处理阶段:
  所有算法在 world(ENU) 系下工作
  ESKF 状态表示 imu_link 在 world 下的位姿
  PGO 优化在 world 系下做绝对约束 + 相对约束

输出阶段:
  pose_imu (world 系) ──→ 杆臂逆补偿 → pose_base (world 系) ──→ TF(base_link)
```

---

## 6. 核心算法详述

### 6.1 ESKF — 误差状态卡尔曼滤波

- **状态维度**：12 维 `[pos(3), rot(3), vel(3), bg(3)]`，`ba` 和 `grav` 固定不参与在线估计
- **状态表示**：`NavState`，重力向量固定 `[0, 0, -9.81]`
- **Predict**：中值积分 `x.oplus(f_, dt)`，协方差传播 `P = F·P·Fᵀ + G·Q·Gᵀ`
- **Update（迭代 ESKF）**：
  1. 在误差状态空间迭代线性化，每次调用 `ObsModel`（LIDAR）或 `GPSObsModel`（GPS）
  2. 退化检测：`HᵀH` 特征分解，低特征值方向投影掉，协方差膨胀
  3. 状态更新限幅：平移 ≤0.5m，旋转 ≤5°，速度 ≤2m/s
  4. GPS 观测是线性模型（`H=I₆`），一次迭代收敛
- **数值保护**：`SymmetrizeAndFloorCovariance(P)`，对角裁剪 `[1e-9, 100]`，NaN/Inf→1.0
- **Anderson 加速**：可选，维护最近 10 次迭代历史加速 ICP 收敛

### 6.2 LaserMapping — 激光里程计

- **主循环** `Run()`：SyncPackages → ImuProcess → 降采样 → ESKF Update(LIDAR) → MakeKF → MapIncremental
- **观测模型 `ObsModel()`**：
  - **点到面 ICP**：iVox 搜索 5 个最近邻 → 估计平面 → 点到面残差，OpenMP 并行累加 `HᵀH`/`Hᵀr`
  - **点到点 ICP**（可选，`icp_weight_=100`）
  - 有效点数 <20 → 无效观测
- **关键帧**：位移 > 2m 或角度 > 15°；定位模式超 2s 强制创建
- **IMU 初始化**（前 20 帧）：估计重力方向、陀螺零偏，自适应检测加速度单位

### 6.3 LoopClosing — 回环检测

1. 每隔 20 帧检查，历史帧距离 <30m → 候选
2. 多分辨率 NDT（10m→5m→2m→1m）配准，score > 阈值 → 回环
3. 位姿图：顶点（opt_pose）+ 运动边（LIO 相对）+ 回环边（NDT）+ 高度先验 + RTK 先验边
4. 增量优化，仅在回环触发时执行

### 6.4 PGO — 多源融合位姿图优化

- **滑动窗口**：最多 5 帧，自适应调整
- **观测源**：LidarLoc（绝对，含退化检测）、RTK（绝对，100Hz→插值降频）、LidarOdom（相对）、DR（相对）、Prior（边缘化）
- **边缘化**：滑出帧的约束压缩到保留帧

### 6.5 miao 优化库

| 组件 | 说明 |
|------|------|
| 图结构 | `BaseVertex`, `BaseEdge`, `BaseUnaryEdge`, `BaseBinaryEdge`，支持增量模式 |
| 求解器 | Dense, Eigen Cholesky, PCG, CCS |
| 优化算法 | Gauss-Newton, Levenberg-Marquardt, DogLeg |
| 鲁棒核 | Huber, Cauchy, Tukey, Welsch, DCS, Fair, PseudoHuber, Saturated, McClure |
| 预定义类型 | `EdgeSE3`, `EdgeSE3Prior`, `EdgeSE3HeightPrior`, `EdgeSE2`, `EdgeSE2Prior`, `VertexSE3`, `VertexSE2`, `VertexPointXYZ` |

### 6.6 地图管理

- **TiledMap**：瓦片分区动态加载，`load_map_size`/`unload_map_size` 控制范围
- **iVox**：Hilbert 曲线 3D 体素栅格，支持 CENTER/NEARBY6/18/26 邻域
- **g2p5**：3D→2D 栅格投影，地面参数可动态估计

## 7. 雷达类型与点云预处理

| lidar_type | 雷达 | 格式 | time 字段 | time_scale |
|-----------|------|------|-----------|------------|
| 1 | Livox AVIA | CustomMsg | — | — |
| 2 | Velodyne 32 | `Point` (ring+time) | float | 可用 |
| 3 | Ouster 64 | `Point` (ring+time) | uint32 (μs→ms) | 可用 |
| 4 | RoboSense | `Point` (ring+time) | double | 可用 |
| 5 | LeiShen 镭神 | `lightning::LeiShenPoint` (ring+time) | float 秒 | **不用**（固定 ×1e3） |

- LeiShen 使用 `lightning::LeiShenPoint`（避免与官方 SDK 的 ODR 冲突），`pcl::fromROSMsg` 按字段名匹配
- 所有格式统一转为 `PointType`（包含 `x, y, z, intensity, time(double 毫秒)`）

## 8. 常用构建命令

```bash
# 激活 ROS2 环境
ros

# 首次编译前：构建 Pangolin
cd src/lightning-lm/thirdparty && unzip Pangolin-0.9.3.zip -d Pangolin
cd Pangolin/Pangolin-0.9.3 && mkdir build && cd build
sudo apt install -y libepoxy-dev
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_FFMPEG=OFF
make -j$(nproc) && sudo make install

# 编译
cd ~/lightning-lm_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lightning
source install/setup.bash

# Debug 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lightning

# 无 Livox / march=native
colcon build --cmake-args -DUSE_LIVOX=OFF --packages-select lightning
colcon build --cmake-args -DBUILD_WITH_MARCH_NATIVE=ON --packages-select lightning

# 运行
ros2 run lightning run_slam_offline --config ./config/default.yaml --input_bag <bag>
ros2 run lightning run_slam_online --config ./config/default.yaml
ros2 run lightning run_loc_offline --config ./config/default.yaml --input_bag <bag>
ros2 run lightning run_loc_online --config ./config/default.yaml
```

## 9. 代码编写约定

- **风格**：`.clang-format`（Google Style 变体，4 空格，120 列宽，指针左对齐）
- **命名**：类名 CamelCase，函数/变量 snake_case，成员变量 `_` 后缀，常量 `k` 前缀全大写，命名空间 `lightning`
- **注释**：中文
- **头文件保护**：`#pragma once`
- **Eigen**：通过 `common/eigen_types.h` 引入类型别名，用 Sophus（`SE3`/`SO3`）而非裸 `Eigen::Quaternion`
- **线程安全**：共享容器 `std::mutex` + `std::lock_guard`，注意 OpenMP 并行安全
- **ROS2 隔离**：仅在 `wrapper/` 和 `app/` 使用 ROS2 API，核心算法 ROS-free
- **禁止**：修改 `eigen_types.h` 类型别名、新增第三方依赖、删除业务注释、过度设计

## 10. 搜索与调试

### 搜索策略
- 约 27,000 行代码，优先 `grep` 搜关键字/类名/函数名
- YAML 配置在 `config/*.yaml`，先查配置再查代码
- miao 库边/顶点类型在 `src/core/miao/core/types/`

### 调试要点
- 离线模式可 gdb 逐帧调试；在线模式多线程并发
- `LOG(INFO/WARNING/ERROR)` 日志，`system.with_ui: true` 开启 Pangolin 3D 显示
- ESKF/优化器调试：检查协方差对角线、Hessian 特征值、残差均值/最大值
- IMU/传感器：检查时间戳单调性、四元数归一化、Eigen NaN/Inf

### 数值稳定性
- ESKF `SymmetrizeAndFloorCovariance`：对称化 + 对角裁剪 `[1e-9, 100]` + NaN→1.0
- 退化检测：`HᵀH` 特征分解，低特征值方向投影掉，协方差膨胀 `degeneracy_cov_inflation_=1.02`
- 更新限幅：平移 ≤0.5m，旋转 ≤5°，速度 ≤2m/s
- RTK：`allFinite()` 校验，SO(3).log() π 附近奇异性退化，除零保护 `max(σ, 1e-6)²`

## 11. 数据集与适配

已验证数据集：NCLT, VBR, Livox Multi Floor, UrbanLoco, 斜装 30°, 云深处四足, 吉利, 高架桥, LeiShen（镭神）

新增传感器适配流程：
1. 设置 `fasterlio.lidar_type`（1-5），如需新格式在 `point_def.h` + `pointcloud_preprocess` 新增 Handler
2. 配置 `common.lidar_topic`, `common.imu_topic`
3. 检查 `fasterlio.time_scale`（LeiShen 不适用）
4. 离线模式参数调试 → 在线模式验证

## 12. 配置要点

### 关键 YAML 参数速查

```yaml
common:
  rtk_topic: "/sensing/ins/ins_raw"  # INS 融合位姿话题
  T_imu_base: [0.0, 0.0, 0.0]       # IMU→base_link 杆臂 (m)，前/左/上

fasterlio:
  lidar_type: 2            # 雷达类型
  time_scale: 0.001        # 时间戳缩放（LeiShen 不用）
  imu_filter: false        # IMU 低通滤波（斜装应关闭）
  use_aa: false            # Anderson 加速
  enable_icp_part: true    # 点到点 ICP 辅助
  kf_dis_th: 2.0           # 关键帧位移阈值 (m)
  extrinsic_T/R            # 雷达→IMU 外参（merge_cloud 场景见 §3.4）
  rtk_noise_ratio: 1.0     # RTK 观测噪声比例（>1 降低权重）
  rtk_rot_noise: 0.0052    # RTK 姿态观测噪声 (rad)，≈0.3°

loop_closing:
  with_height: true        # 高度约束（多层室内应关闭）
  loop_kf_gap: 20          # 回环检查间隔（KF 数）
  rtk_pos_noise: 0.1       # RTK 位置噪声回退值 (m)
  rtk_ang_noise: 0.0052    # RTK 姿态噪声回退值 (rad)
  rtk_outlier_th: 10.0     # RTK 异常值卡方阈值

pgo:
  pgo_smooth_factor: 0.01  # PGO 平滑因子
  rtk_pos_noise: 0.1       # RTK 位置噪声回退值 (m)
  rtk_ang_noise: 0.0052    # RTK 姿态噪声回退值 (rad)
  rtk_outlier_th: 10.0     # RTK 异常值卡方阈值

system:
  with_ui: true            # 3D 可视化
  with_2dui: true          # 2D 栅格显示
```
