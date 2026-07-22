# 🤖 Lightning-LM 项目宪法

## 1. 项目上下文

- **项目名称**：Lightning-LM（Lightning-Speed Lidar Localization and Mapping）
- **核心功能**：完整的 3D 激光 SLAM + 定位模块，包括 LIO 前端、回环检测、地图管理、激光定位
- **ROS2 版本**：ROS2 Humble（`package.xml` 中 `buildtool_depend ament_cmake`，Docker 基础镜像 `ros:humble-perception`）
- **本地开发环境**：Ubuntu 22.04 + ROS2 Humble（见 `~/.claude/CLAUDE.md`）
- **环境激活**：本地使用 `ros` 命令激活 ROS2 环境（配置在 `~/.zshrc` 中）
- **C++ 标准**：C++17（`CMAKE_CXX_STANDARD 17`，见 `CMakeLists.txt`）
- **构建系统**：CMake 3.16+，使用 `ament_cmake`，顶层用 Colcon 编排
- **代码规模**：约 34,000 行 C++ 代码
- **第三方依赖**：
  - 优化/数学：Eigen3、Sophus（内置在 `thirdparty/`）、miao（自研轻量优化库，在 `src/core/miao/`）
  - 点云/地图：PCL、pcl_conversions、pcl_ros
  - 可视化：Pangolin（`thirdparty/Pangolin-0.9.3.zip`）、OpenGL
  - 通信/序列化：ROS2 原生（`std_msgs`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`）、tf2、rosbag2
  - 日志/工具：glog、gflags、yaml-cpp
  - 图像：OpenCV
  - 并行：OpenMP、TBB
  - 完整依赖安装见 `scripts/install_dep.sh` 和 `docker/Dockerfile`
- **依赖管理**：通过 apt 安装（`ros-humble-*` 和系统包），Sophus 和 Pangolin 源码内置在 `thirdparty/` 中；🚫 未经确认禁止新增依赖
- **代码仓库**：GitHub `gaoxiang12/lightning-lm`（公开仓库）

## 2. 项目目录结构

```
lightning-lm/
├── CMakeLists.txt              # 顶层 CMake，定义项目名 lightning
├── package.xml                 # ROS2 包描述
├── .clang-format               # 代码风格配置（基于 Google Style，4 空格缩进，120 列宽）
├── README.md / README_CN.md    # 中英文文档
├── config/                     # YAML 配置文件
│   ├── default.yaml            # 默认配置（NCLT 数据集）
│   ├── default_nclt.yaml
│   ├── default_vbr.yaml
│   ├── default_livox.yaml
│   ├── default_robosense.yaml
│   └── default_utbm.yaml
├── cmake/
│   └── packages.cmake          # find_package 和全局编译选项
├── scripts/
│   ├── install_dep.sh          # 依赖安装脚本
│   ├── merge_bags.py           # bag 合并工具
│   └── save_default_map.sh
├── tests/
│   └── rtk_utils_test.cc       # LLA→UTM 单元测试（零外部依赖，可手动编译）
├── docker/
│   ├── Dockerfile              # 基于 ros:humble-perception 的 Docker 镜像
│   └── README.md
├── srv/
│   ├── LocCmd.srv              # 定位指令服务
│   └── SaveMap.srv             # 保存地图服务
├── pcd/                        # 示例点云数据
├── doc/                        # 文档图片和 GIF
├── thirdparty/
│   ├── Sophus/                 # 李群李代数库（SE3/SO3）
│   ├── Pangolin-0.9.3.zip      # 3D 可视化库
│   └── livox_ros_driver/       # Livox 雷达 ROS2 驱动
└── src/
    ├── CMakeLists.txt           # 核心库 + 可执行程序构建
    ├── common/                  # 基础数据结构
    │   ├── eigen_types.h        # Eigen/Sophus 类型别名（SE3, SO3, Vec3d 等）
    │   ├── nav_state.h/cc       # 导航状态（12 维：pos/rot/vel/bg）
    │   ├── imu.h                # IMU 数据结构
    │   ├── keyframe.h           # 关键帧
    │   ├── odom.h               # 里程计数据
    │   ├── point_def.h          # 点云定义
    │   ├── constant.h           # 常量定义
    │   ├── params.h / options.h/cc  # 参数/配置加载
    │   ├── timed_pose.h         # 带时间戳的位姿
    │   ├── measure_group.h      # 测量组（含 IMU + RTK 同步数据）
    │   ├── rtk_data.h           # RTK/INS 观测数据结构体
    │   ├── rtk_utils.h/cc       # LLA→UTM 坐标转换（零依赖）
    │   └── ...
    ├── io/                      # 文件 IO / 数据读写
    │   ├── yaml_io.cc/h         # YAML 参数读取
    │   ├── file_io.cc/h         # 文件读写（地图保存/加载）
    │   └── dataset_type.h       # 数据集类型枚举
    ├── wrapper/                 # ROS2 封装层
    │   ├── bag_io.cc/h          # ROS2 Bag 读写
    │   └── ros_utils.h          # ROS2 工具函数
    ├── utils/                   # 通用工具
    │   ├── timer.cc/h           # 计时器
    │   ├── pointcloud_utils.cc/h # 点云处理工具
    │   ├── sync.h               # 同步工具
    │   └── async_message_process.h
    ├── ui/                      # Pangolin 3D/2D 可视化界面
    │   ├── pangolin_window.cc/h/_impl
    │   ├── ui_car.cc/h          # 车辆模型
    │   ├── ui_cloud.cc/h        # 点云渲染
    │   └── ui_trajectory.cc/h   # 轨迹渲染
    ├── core/                    # 核心算法模块
    │   ├── lio/                 # LIO 前端（FasterLIO）
    │   │   ├── eskf.hpp/cc      # ESKF（误差状态卡尔曼滤波，12 维状态）
    │   │   ├── laser_mapping.h/cc  # 激光里程计（点到面 ICP + 可选点到点 ICP）
    │   │   ├── imu_processing.hpp   # IMU 预积分/处理
    │   │   ├── imu_filter.h         # IMU 滤波器
    │   │   ├── pointcloud_preprocess.h/cc  # 点云预处理（运动补偿、降采样）
    │   │   ├── pose6d.h             # 6 自由度位姿
    │   │   └── anderson_acceleration.h  # Anderson 加速（可选）
    │   ├── ivox3d/              # iVox 3D 空间索引
    │   │   ├── ivox3d.h/_node.hpp   # 体素栅格索引
    │   │   └── hilbert.hpp          # Hilbert 曲线排序
    │   ├── loop_closing/        # 回环检测与闭环
    │   │   └── loop_closing.h/cc
    │   ├── g2p5/                # 3D 点云转 2D 栅格地图（地面分割+投影）
    │   │   ├── g2p5.h/cc
    │   │   ├── g2p5_map.cc/h
    │   │   └── g2p5_subgrid.cc/h
    │   ├── maps/                # 瓦片地图管理（动态加载/卸载）
    │   │   ├── tiled_map.cc/h
    │   │   └── tiled_map_chunk.cc/h
    │   ├── localization/        # 激光定位
    │   │   ├── localization.h/cpp         # 定位主逻辑
    │   │   ├── localization_result.h/cc   # 定位结果
    │   │   ├── lidar_loc/                 # 激光雷达地图匹配（NDT-OMP）
    │   │   │   ├── lidar_loc.h/cc
    │   │   │   └── pclomp/               # OpenMP 加速的 NDT + VoxelGrid
    │   │   └── pose_graph/               # 位姿图优化（PGO）+ 平滑器
    │   │       ├── pgo.h/cc/_impl.h       # 位姿图优化（基于 miao）
    │   │       ├── pose_extrapolator.h/cc  # 位姿外推器
    │   │       └── smoother.h             # 平滑器
    │   ├── miao/                # 自研轻量优化库（来自 g2o，更轻更快，支持增量优化）
    │   │   ├── core/
    │   │   │   ├── graph/       # 图优化核心（顶点、边、优化器）
    │   │   │   ├── solver/      # 线性求解器（Dense、Eigen、PCG、CCS）
    │   │   │   ├── sparse/      # 稀疏块矩阵
    │   │   │   ├── opti_algo/   # 优化算法（GN、LM、DogLeg）
    │   │   │   ├── robust_kernel/  # 鲁棒核函数（Huber、Cauchy、Tukey 等）
    │   │   │   ├── types/       # 预定义边/顶点类型（SE3、SE2）
    │   │   │   ├── math/        # 数学工具（边缘协方差、矩阵运算）
    │   │   │   └── common/      # 配置、宏
    │   │   ├── utils/           # 采样器等工具
    │   │   └── examples/        # 示例程序（BA、ICP、Pose Graph 等）
    │   ├── system/              # 系统级调度（在线/离线模式）
    │   │   ├── slam.h/cc        # 建图系统
    │   │   └── loc_system.h/cc   # 定位系统
    │   └── lightning_math.hpp   # 项目通用数学函数
    └── app/                     # 可执行程序入口
        ├── run_slam_online.cc       # 在线建图
        ├── run_slam_offline.cc      # 离线建图（从 bag 读取）
        ├── run_loc_online.cc        # 在线定位
        ├── run_loc_offline.cc       # 离线定位（从 bag 读取）
        ├── run_frontend_offline.cc  # 仅前端（LIO）离线运行
        ├── run_loop_offline.cc      # 离线回环检测
        └── test_ui.cc              # UI 测试
```

## 3. 核心模块详解

### 3.1 LIO 前端（`src/core/lio/`）

- **ESKF**：误差状态卡尔曼滤波，12 维状态变量（位置 3 + 旋转 3 + 速度 3 + 陀螺零偏 3），固定重力向量不参与在线估计
- **LaserMapping**：点到面 ICP + 可选点到点 ICP，支持 OpenMP 并行加速
- **点云预处理**：运动补偿、体素降采样、时间戳缩放（`time_scale` 参数）
- **Anderson 加速**：可选（`use_aa` 配置），加速 ICP 迭代收敛

### 3.2 miao 优化库（`src/core/miao/`）

- 自研轻量图优化库，API 类似 g2o 但更轻更快
- 支持增量优化（新增节点不需要重新构建整个优化问题）
- 求解器：Dense、Eigen Cholesky、PCG（共轭梯度）、CCS
- 优化算法：Gauss-Newton、Levenberg-Marquardt、DogLeg
- 鲁棒核：Huber、Cauchy、Tukey、Welsch、DCS、Fair、PseudoHuber、Saturated、McClure
- 预定义类型：`EdgeSE3`、`EdgeSE3Prior`、`EdgeSE3HeightPrior`、`VertexSE3` 等

### 3.3 定位模块（`src/core/localization/`）

- **LidarLoc**：基于 NDT-OMP 的激光雷达地图匹配定位
  - 支持全局初始化（栅格搜索角度范围 + 步长）
  - 支持关键帧级别定位（`loc_on_kf`）
  - ICP 调整（`enable_icp_adjust`）
- **PoseGraph (PGO)**：基于 miao 的位姿图优化，融合 LIO 里程计 + 激光定位 + RTK（可选）约束
- **PoseExtrapolator**：基于历史位姿的线性外推
- **Smoother**：对优化后的轨迹做平滑处理（`smooth_factor` 可调）

### 3.4 回环检测（`src/core/loop_closing/`）

- 基于关键帧间隔 + NDT 配准分值检测回环
- 优化后的位姿作为检测初值

### 3.5 地图管理（`src/core/maps/`）

- **TiledMap**：瓦片地图分区动态加载方案，适用大场景
- 支持加载/卸载区域大小配置（`load_map_size`、`unload_map_size`）
- 动态图层分离：短期/中期/永久三种策略（`dyn_cloud_policy`）

### 3.6 g2p5（`src/core/g2p5/`）

- 3D 点云转 2D 栅格地图
- 地面参数可选动态估计（`esti_floor`）
- 栅格分辨率可配置（`grid_map_resolution`）

## 4. 常用构建命令

```bash
# 激活 ROS2 环境
ros

# === 首次编译前：构建 Pangolin ===
cd src/lightning-lm/thirdparty
unzip Pangolin-0.9.3.zip -d Pangolin
cd Pangolin/Pangolin-0.9.3 && mkdir build && cd build
sudo apt install -y libepoxy-dev  # Pangolin 依赖
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_FFMPEG=OFF
make -j$(nproc) && sudo make install

# === 编译 ===
cd ~/lightning-lm_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# === 单独编译 lightning 包 ===
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lightning

# Debug 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select lightning

# 无 Livox 环境编译（跳过 livox_ros_driver 依赖）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_LIVOX=OFF --packages-select lightning

# 启用 march=native 优化（本地 CPU 特性）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=ON --packages-select lightning

# === 运行 ===
# 离线建图
ros2 run lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag ~/data/NCLT/20130110.db3

# 在线建图
ros2 run lightning run_slam_online --config ./config/default_nclt.yaml

# 离线定位
ros2 run lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag <bag_path>

# 在线定位
ros2 run lightning run_loc_online --config ./config/default_nclt.yaml

# 保存地图（在线建图时）
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

## 5. 代码编写约定

- **代码风格**：遵循 `.clang-format` 配置（Google Style 变体）
  - 4 空格缩进，不使用 Tab
  - 列宽上限 120 字符
  - 指针/引用对齐靠左（`int* p`）
  - 短函数/if/循环允许放在一行
  - include 按规则排序
- **命名约定**（从现有代码推断）：
  - 类名：CamelCase（如 `NavState`, `ESKF`, `TiledMap`）
  - 变量名/函数名：snake_case（如 `pos_`, `GetPose()`, `boxminus()`）
  - 成员变量：`_` 后缀（如 `pos_`, `rot_`, `timestamp_`）
  - 常量/枚举值：全大写 snake_case（如 `kPosIdx`, `LIDAR`）
  - 命名空间：`lightning`
- **注释语言**：代码注释使用中文（见 `~/.claude/CLAUDE.md` 全局规则）
- **头文件保护**：使用 `#pragma once`
- **Eigen 使用**：通过 `common/eigen_types.h` 引入类型别名（`Vec3d`, `SE3`, `SO3` 等），使用 Sophus 而非 Eigen::Quaternion 直接表示旋转
- **锁的使用**：多线程共享容器访问必须加锁（`std::lock_guard<std::mutex>`），项目使用 OpenMP 并行，注意并行安全性
- **ROS2**：仅在 `wrapper/` 和 `app/` 层面使用 ROS2 API，核心算法模块保持 ROS-free
- **禁止事项**：
  - 🚫 未经确认不要修改 `src/common/eigen_types.h` 中的类型别名
  - 🚫 不要引入新的第三方依赖（`CMakeLists.txt` / `package.xml` 中）
  - 🚫 不要删除已有的业务逻辑注释
  - 🚫 不要过度设计——保持 KISS 原则

## 6. 搜索策略

- 项目代码量约 34,000 行，**优先使用 `grep` 搜索**关键字/类名/函数名
- 使用 `glob` 查找特定模式文件（如 `**/*.h`, `**/*.yaml`）
- 阅读文件时尽量指定行号范围以节省 Token
- 参数配置在 `config/*.yaml` 中，先查配置再查代码
- miao 库的顶点/边类型定义在 `src/core/miao/core/types/` 下，优先查阅

## 7. 关键注意事项

- **状态维度**：ESKF 状态变量为 12 维（pos 3 + rot 3 + vel 3 + bg 3），ba（加速度计零偏）和 grav（重力）不参与在线估计。这是 2026.4.2 更新后的设计
- **坐标系统**：使用 Sophus SE3/SO3 表示刚体变换，`WGlobal_T_WLocal` 为全局到局部的变换
- **时间戳处理**：`fasterlio.time_scale` 参数对 Velodyne 雷达敏感，用于兼容不同数据集的时间戳单位（1e-3 对应微秒转毫秒）
- **雷达类型**：`fasterlio.lidar_type` — 1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense, 5=LeiShen（镭神），不同雷达的点云预处理路径不同。LeiShen 的 `time_scale` 不生效（time 字段固定为 float 秒，预处理中直接 `× 1e3` 转毫秒）
- **回环检测**：使用优化后的位姿作为检测初值，`with_height` 配置高度约束（适用于平面场景，不适用多层室内）
- **并发模型**：激光匹配使用 OpenMP 并行（NDT-OMP、VoxelGrid-OMP），`std::vector<bool>` 在并行化时有坑（已修复）
- **数值稳定性**：ESKF 对协方差 P 阵做对称化处理和保护最小值，防止数值发散
- **IU 滤波**：`imu_filter` 默认关闭，斜装场景可能需要关闭
- **动态图层**：三种策略 — 短期/中期/永久（`dyn_cloud_policy: persistent`），默认永久保留

## 8. 测试与调试

### 测试流程

```bash
# 1. 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lightning
source install/setup.bash

# 2. 离线建图测试（使用 bag 数据）
ros2 run lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag <bag_path>

# 3. 离线定位测试（需要先建好地图）
ros2 run lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag <bag_path>

# 4. 查看地图结果
pcl_viewer ./data/new_map/global.pcd
```

### 调试注意事项

- **离线模式可断点调试**：`run_slam_offline` / `run_loc_offline` 是单线程遍历 bag，可用 gdb 逐帧调试
- **在线模式多线程并发**：`run_slam_online` / `run_loc_online` 使用多线程处理，处理速度快
- **glog 日志**：使用 `LOG(INFO)`, `LOG(WARNING)`, `LOG(ERROR)` 输出日志
- **Pangolin UI**：`system.with_ui: true` 开启 3D 可视化，`system.with_2dui: true` 开启 2D 栅格显示
- **PCL 查看器**：`pcl_viewer` 可直接查看保存的 `.pcd` 点云地图

## 9. Docker 支持

```bash
# Docker 镜像基于 ros:humble-perception
# 包含所有依赖：PCL、OpenCV、glog、yaml-cpp、Pangolin、hpp-fcl 等

cd docker
docker build -t lightning-lm .
```

## 10. 数据集兼容性

已验证的数据集和场景（来自 README_CN.md）：
- **NCLT** ✅ — 校园环境，32 线 Velodyne
- **VBR** ✅ — 校园/城市环境
- **Livox Multi Floor** ✅ — 多层室内（无法闭环）
- **UrbanLoco** — 城市峡谷
- **斜装 30°** ✅ — 需关闭 IMU filter
- **云深处四足机器人** ✅ — RoboSense 雷达（lidar_type=4）
- **吉利** ✅
- **高架桥** ✅

## 11. 新增传感器/数据集适配流程

1. 确认雷达类型，设置 `fasterlio.lidar_type`（1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense, 5=LeiShen）
2. 修改 `common.lidar_topic` 和 `common.imu_topic` 为实际话题名
3. 检查 `fasterlio.time_scale` 时间戳单位
4. 先用离线模式调试参数，通过后再调在线模式
5. IMU 和雷达外参默认为零即可，项目对此不敏感

## 12. USE_LIVOX 编译开关

项目支持通过 CMake 选项 `USE_LIVOX` 控制 Livox LiDAR 驱动的编译（默认 OFF）：

```bash
# 默认不编译 livox 依赖，适用于标准 Velodyne/RoboSense 等雷达
colcon build --cmake-args -DUSE_LIVOX=OFF

# 如需 Livox 支持
colcon build --cmake-args -DUSE_LIVOX=ON
```

所有 livox 相关代码（`CustomMsg` 类型、订阅器、回调函数）通过 `#ifdef USE_LIVOX` / `#endif` 包裹，不影响默认编译。

## 13. RTK/INS 融合

### 13.1 背景与传感器

- **传感器**：组合导航 INS 模块（如 ins5715），同时输出两路 100Hz 数据：
  - 原始 IMU（未融合 RTK，直接送 ESKF 做 Predict）
  - INS 融合位姿（经纬高 + NED 姿态，模块内部已融合 RTK+IMU）
- **RTK 类型**：双天线 RTK，位置精度 ~2cm，姿态精度 ~0.3°
- **消息类型**：`nav_msgs::msg::Odometry`（position 字段编码 LLA，orientation 已转 ENU，协方差用自定义索引）
- **参考实现**：`~/codes/fusions_slam`（IESKF 中 RTK 穿插更新的方法）

### 13.2 融合架构

```
INS 100Hz ──→ ① ESKF 前端 (gps_obs_func_)        ← ✅ 已实现
                 Predict() 之前穿插 Update(ObsType::GPS)
                         │
              LIO 位姿 ──┤
                         │
         ┌───────────────┴───────────────┐
         ▼                               ▼
  ② 建图后端 LoopClosing          ③ 定位后端 PGO
     PoseOptimization()              BuildProblem()
     + EdgeSE3Prior(RTK)             + EdgeSE3Prior(RTK)
     ← ✅ 已实现                     ← 📋 待实现

  同时支持：
  ④ 定位模式 RTK 订阅（LocSystem → Localization → LIO）
     ← ✅ 已实现（2026-07-22）
  ⑤ georeference.yaml 地图地理元数据
     ← ✅ 已实现（2026-07-22）
  ⑥ IMU→base_link 坐标系统一
     ← ✅ 已实现
```

---

### 13.3 ① ESKF 前端 ✅ 已实现

#### 13.3.1 实现概览

RTK 数据流完全集成在 LIO 前端中，采用 **SyncPackages 合并 + UndistortPcl 按序穿插** 的方式：

```
SlamSystem::ProcessRTK(msg)
  │  Odometry 解码 → LLA→UTM→ENU → RTKData → rtk_buffer_
  ▼
SyncPackages()
  │  measures_.rtk_ = RTK 数据 in [lidar_begin, lidar_end]
  │  清理过期 RTK（timestamp < lidar_begin_time_）
  ▼
UndistortPcl()
  │  it_rtk = meas.rtk_.begin()
  │  for each IMU pair (head, tail):
  │    while rtk.timestamp in [head, tail):  ← 先 RTK Update
  │      current_rtk_ = rtk → kf.Update(GPS)
  │    kf.Predict(head→tail)                 ← 再 Predict
```

**关键设计决策**：先 Update 再 Predict（而非先 Predict 再 Update），将 RTK 观测近似对齐到 head 时刻（中低速场景下 IMU 间隔足够小，误差可忽略），与 fusions_slam 参考实现策略一致。

#### 13.3.2 改动文件

| 文件 | 说明 |
|------|------|
| `common/rtk_data.h` | **新增** `RTKData` 结构体（ENU 位置 + 姿态 + 噪声标准差） |
| `common/rtk_utils.h/cc` | **新增** WGS84 椭球 LLA→UTM 投影 + 公共 `MapOrigin` 结构体 + `RTKConverter` 类（坐标转换逻辑复用） |
| `common/measure_group.h` | 新增 `std::deque<RTKData> rtk_` 字段 |
| `core/lio/laser_mapping.h` | 新增 `GPSObsModel()`、`current_rtk_`、`rtk_noise_ratio_`、`rtk_buffer_` |
| `core/lio/laser_mapping.cc` | `Init()` 绑定 `gps_obs_func_` + `SetRTKContext`；`SyncPackages()` 收集 RTK；`ProcessRTK()` 入队；`GPSObsModel()` 实现（含 NaN 检查、旋转角保护、除零保护） |
| `core/lio/imu_processing.hpp` | `UndistortPcl()` 中按时间线穿插 RTK Update（将 RTK 近似对齐到 head 时刻）；`SetRTKContext()` 接口 |
| `core/lio/eskf.cc` | `Update()` 中 GPS 观测跳过迭代循环（线性模型一次收敛） |
| `core/system/slam.h/cc` | `ProcessRTK()` 使用 `RTKConverter` 简化；`MapOrigin` 改用 `rtk_utils.h` 公共定义；`SaveMap()` 写 `georeference.yaml`（2026-07-22 重构） |
| `config/default.yaml` | 新增 `rtk_topic`（common 下）、`rtk_noise_ratio`、`rtk_rot_noise`（fasterlio 下） |
| `tests/rtk_utils_test.cc` | **新增** LLA→UTM 单元测试（6 个用例，零外部依赖，可手动编译运行） |

#### 13.3.3 观测模型 (GPSObsModel)

```
// 1. 输入校验：过滤 NaN/Inf
if (!position.allFinite() || !orientation.coeffs().allFinite()) → obs.valid_ = false

// 2. 位置残差：ENU 坐标系下直接做向量差
r_pos = rtk.position - state.pos

// 3. 姿态残差：SO(3) Log 映射
delta_rot = state.rot^{-1} * rtk.orientation
r_rot = delta_rot.log()
if (|r_rot| > π * 0.8) → 退化姿态分量，r_rot = 0  // 防止 π 附近数值奇异

// 4. 信息加权（含除零保护，min_std = 1e-6）
info_pos = 1 / max(pos_std², 1e-12)
info_rot = 1 / max(rot_std², 1e-12)
HTH = diag(info_pos, info_rot)     // H = I_6，所以 H^T W H = W
HTr = W * [r_pos; r_rot]
```

协方差来源：`msg->pose.covariance[0]`=东向方差, `[7]`=北向方差, `[8]`=高程方差（ins5715 驱动自定义索引）；姿态噪声从 YAML `rtk_rot_noise` 读取（默认 0.0052 rad ≈ 0.3°）。

#### 13.3.4 坐标转换链

```
RTK LLA → LLAtoUTM(lat, lon, alt) → UTM
中值滤波（10帧）→ rtk_origin_ 初始化（对单帧坏值不敏感）
UTM 跨带检查（zone 变化时跳过，防止数百米跳变）
ENU = UTM - origin_UTM → 位置跳变检测（>10 m/s 视为异常）
→ 送入 ESKF/位姿图（本地 ENU）
```

#### 13.3.5 YAML 配置

```yaml
common:
  rtk_topic: "/sensing/ins/ins_raw"    # INS 融合位姿话题

fasterlio:
  rtk_noise_ratio: 1.0                 # RTK 观测噪声比例因子（>1 降低权重）
  rtk_rot_noise: 0.0052                # RTK 姿态观测噪声标准差 (rad)，0.3°
```

---

### 13.4 ② 建图后端 LoopClosing ✅ 已实现（2026-07-16）

#### 13.4.1 设计思路

RTK 观测以 `EdgeSE3Prior`（一元绝对位姿约束）的形式加入 `LoopClosing::PoseOptimization()`，在回环触发全局优化时提供绝对锚点。信息矩阵用 RTK 协方差的逆（优先使用 keyframe 携带的 per-RTK 噪声，回退到 YAML 默认值），加 Cauchy 鲁棒核防止跳变。

**架构说明**：
- SLAM 模式下后端优化器平时"静默"（`candidates_.empty()` 时直接 return），只在回环事件时触发优化
- RTK 边在 `PoseOptimization()` 中**每次都会添加**（无论是否有回环候选），但仅在回环触发优化时才参与求解
- 由于优化器是增量模式（`incremental_mode_ = true`），RTK 边会累积在图中，等回环发生时一起约束
- RTK 边的作用是"防守型"：防止回环优化把轨迹拉偏，而非主动提升无回环路段的精度

**噪声约定**：所有 `*_noise` / `*_std` 参数统一使用**标准差**，构建信息矩阵时平方转方差（`info = 1 / σ²`）。

#### 13.4.2 改动文件

| 文件 | 说明 |
|------|------|
| `common/keyframe.h` | 新增 `#include "common/rtk_data.h"`；新增 `rtk_data_`/`rtk_valid_` 成员；新增 `SetRTKData()`/`HasRTK()`/`GetRTKPose()`/`GetRTKPosStd()`/`GetRTKRotStd()` 方法（全部加锁） |
| `core/lio/laser_mapping.cc` | `MakeKF()` 中在 `kf->SetState()` 之后关联 `current_rtk_`（时间差 < 0.5s 且数据有效） |
| `core/loop_closing/loop_closing.h` | 新增 `#include "core/types/edge_se3_prior.h"`；Options 新增 `rtk_pos_noise_`/`rtk_ang_noise_`/`rtk_outlier_th_`；新增 `rtk_edges_` 成员；新增 `AddRTKFactors()` 声明 |
| `core/loop_closing/loop_closing.cc` | `Init()` 从 YAML 读取 RTK 参数；`PoseOptimization()` 中在高度先验后调用 `AddRTKFactors()`，优化后处理 RTK outlier；`AddRTKFactors()` 实现（含噪声回退逻辑 + Cauchy 鲁棒核） |
| `config/default*.yaml`（6 个） | `loop_closing:` 段新增 `rtk_pos_noise`/`rtk_ang_noise`/`rtk_outlier_th` |

#### 13.4.3 代码审查修复（实现时发现并修复，共 4 条）

| 编号 | 严重度 | 问题 | 修复方式 |
|------|--------|------|----------|
| F1 | 🔴 | `info_rtk_` 死代码：Init 中先构建默认信息矩阵再读 YAML，顺序错误；且 `AddRTKFactors()` 已内联构建 per-keyframe 信息矩阵，`info_rtk_` 从未使用 | **删除** `info_rtk_` 成员及其初始化代码 |
| F2 | 🔴 | 噪声为零时信息矩阵 = 1e6（过度约束）：`std::max(σ², 1e-6)` 保护不足 | 增加 `safe_pos_std`/`safe_rot_std`：当 σ < 1e-4 时回退到 YAML 配置的默认值 |
| F3 | 🟡 | `HasRTK()`/`GetRTKPose()` 等 getter 声明为 `const`，但 `std::unique_lock` 需要非 const mutex | 去掉 `const` 限定符，与现有 `GetLIOPose()`/`GetOptPose()` 风格一致 |
| F4 | 🟢 | `rtk_edges_` 线性累积（与 `edge_loops_` 相同模式） | `AddRTKFactors()` 开头 `rtk_edges_.clear()`，每轮只追踪本轮新增边 |

#### 13.4.4 AddRTKFactors() 核心逻辑

```cpp
void LoopClosing::AddRTKFactors() {
    rtk_edges_.clear();  // F4: 每轮只追踪本轮新增的边

    auto v = optimizer_->GetVertex(cur_kf_->GetID());
    if (v == nullptr || !cur_kf_->HasRTK()) return;

    SE3 rtk_pose = cur_kf_->GetRTKPose();
    Vec3d pos_std = cur_kf_->GetRTKPosStd();
    Vec3d rot_std = cur_kf_->GetRTKRotStd();

    // F2: 噪声过小或为零 → 回退到 YAML 默认值（避免 info = 1e6 过度约束）
    auto safe_pos_std = [&](double s) { return (s > 1e-4) ? s : options_.rtk_pos_noise_; };
    auto safe_rot_std = [&](double s) { return (s > 1e-4) ? s : options_.rtk_ang_noise_; };

    // 用 RTK 实际协方差构建信息矩阵（1/σ²）
    Mat6d rtk_info = Mat6d::Zero();
    rtk_info(0,0) = 1.0 / (safe_pos_std(pos_std.x()) * safe_pos_std(pos_std.x()));
    // ... (y, z, roll, pitch, yaw 同理)

    auto e = std::make_shared<miao::EdgeSE3Prior>();
    e->SetVertex(0, v);
    e->SetMeasurement(rtk_pose);
    e->SetInformation(rtk_info);

    auto rk = std::make_shared<miao::RobustKernelCauchy>();
    rk->SetDelta(options_.rtk_outlier_th_);
    e->SetRobustKernel(rk);

    optimizer_->AddEdge(e);
    rtk_edges_.emplace_back(e);
}
```

#### 13.4.5 PoseOptimization() 调用位置

```
添加顶点 → 运动边(前2帧) → 高度先验 → AddRTKFactors()  ← 插入位置
→ 回环边 → if candidates_.empty() return → Optimize
→ 回环 outlier 检测 → RTK outlier 检测 → 写回位姿
```

注意：RTK 边在 `candidates_.empty()` 判断**之前**添加，因此即使当前 KF 无回环，RTK 边也会被加入增量优化器。下次回环触发优化时，所有累积的 RTK 边一起约束。

#### 13.4.6 YAML 配置

```yaml
loop_closing:
  # ... 现有配置 ...
  rtk_pos_noise: 0.1        # RTK 位置噪声标准差 (m)，仅在关键帧噪声无效（<1e-4）时作为回退值使用
  rtk_ang_noise: 0.0052     # RTK 姿态噪声标准差 (rad)，≈0.3°，仅在关键帧噪声无效时作为回退值
  rtk_outlier_th: 10.0      # RTK 异常值卡方阈值（Cauchy delta）
```

#### 13.4.7 include 依赖

- `keyframe.h` → `#include "common/rtk_data.h"`
- `loop_closing.h` → `#include "core/types/edge_se3_prior.h"`
- `loop_closing.cc` → 复用已有 `#include "core/robust_kernel/cauchy.h"`（无需新增）

---

### 13.5 ③ 定位后端 PGO RTK 因子 ✅ 已实现（2026-07-22）

**动机**（2026-07-22 重新评估）：虽然 ESKF 前端已用 RTK 约束 LIO 位姿，但 PGO 后端仅用 NDT 定位结果作为绝对约束（EdgeSE3Prior），LIO 贡献的是帧间相对边（EdgeSE3）。当 NDT 精度（~0.3m）低于 RTK（~0.02m）时，PGO 优化会将绝对位姿拉向 NDT 方向，RTK 在绝对位置上的贡献被覆盖。加上 RTK 先验边后，PGO 拥有两个独立的绝对锚点，可互相制衡。

**频率问题**：RTK 100Hz 不直接送入 PGO。通过 `rtk_pose_queue_` 缓冲 + 时间插值，仅在 PGO 帧时刻（1-5Hz，由 LidarLoc 触发）添加 RTK 边。

#### 13.5.1 数据流

```
ProcessRTK(RTKData)                ← 100Hz，仅入队（PGO::ProcessRTK → PGOImpl::ProcessRTK）
    │  rtk_pose_queue_
    ▼
AddPGOFrame(frame)
    │  AssignRTKPoseIfNeeded(frame)  ← is_in_map_ 确认后、RunOptimization 前
    │    math::PoseInterp<RTKData>    ← 位置线性插值，姿态 slerp
    │    噪声取自 best_match（最近帧），不参与插值
    ▼
BuildProblem()
    │  if (is_in_map_) {
    │    AddLidarLocFactors()          ← NDT 绝对约束（已有）
    │    AddRTKFactors()               ← RTK 绝对约束（新增 EdgeSE3Prior + Cauchy）
    │  }
    │  AddLidarOdomFactors()           ← LIO 相对约束（已有）
    │  AddPriorFactors()              ← 边缘化约束（已有）
    ▼
RunOptimization()
    │  optimizer_->Optimize(5)
    │  遍历 rtk_edges_ → ComputeError → 写回 PGOFrame::rtk_chi2_
    ▼
CollectOptimizationStatistics()        ← 打印 RTK 边统计
```

#### 13.5.2 改动文件

| 文件 | 说明 |
|------|------|
| `core/localization/pose_graph/pgo_impl.h` | 取消 `PGOFrame::rtk_*` 注释 + 新增 `rtk_pos_std_`/`rtk_rot_std_`；新增 `rtk_pose_queue_`/`rtk_edges_`/`rtk_noise_`；新增 `ProcessRTK`/`AssignRTKPoseIfNeeded`/`AddRTKFactors` 声明；Options 新增 `rtk_pos_noise`/`rtk_ang_noise`/`rtk_outlier_th` |
| `core/localization/pose_graph/pgo_impl.cc` | 构造函数初始化 `rtk_noise_`；`ProcessRTK()` 高频入队；`AssignRTKPoseIfNeeded()` PoseInterp 插值 + 噪声从 best_match 取；`AddRTKFactors()` EdgeSE3Prior + Cauchy + 噪声回退；`BuildProblem()` 在 `is_in_map_` 下调用 `AddRTKFactors()`；`RunOptimization()` 优化后回写 `rtk_chi2_`；`CleanProblem()` 清理 `rtk_edges_`；`CollectOptimizationStatistics()` 打印 RTK 边 |
| `core/localization/pose_graph/pgo.h` | 新增 `ProcessRTK(const RTKData&)` 公开接口 |
| `core/localization/pose_graph/pgo.cc` | `ProcessRTK()` 加锁后委托给 `impl_->ProcessRTK()` |
| `core/localization/localization.cpp` | `ProcessRTKMsg` 中新增 `pgo_->ProcessRTK(rtk)` 转发 |
| `config/default*.yaml`（6 个） | `pgo:` 段替换旧 RTK 参数为 `rtk_pos_noise`/`rtk_ang_noise`/`rtk_outlier_th` |

#### 13.5.3 核心实现要点

**AssignRTKPoseIfNeeded()**：
- 使用 `math::PoseInterp<RTKData>()` 在队列中按帧时间戳插值
- `take_pose_func` 从 `RTKData::position` + `orientation` 构造 `SE3`
- 噪声取自 `best_match.pos_std`/`rot_std`（最近帧，不参与插值以避免过度平滑）
- 时间窗口 0.5s，超时返回 false

**AddRTKFactors()**：
- 检查 `current_frame_->rtk_set_ && rtk_valid_`
- 使用 per-frame 噪声 `rtk_pos_std_`/`rtk_rot_std_` 构建信息矩阵（`1/σ²`）
- 噪声 < `1e-4` 时回退到 `options_.rtk_pos_noise`/`rtk_ang_noise`（YAML 默认值）
- Cauchy 鲁棒核，delta = `options_.rtk_outlier_th`（默认 10.0）

**RunOptimization() chi2 回写**：
- 优化后在 `rtk_edges_` 上遍历 `ComputeError()`
- 通过 `frames_by_id_` 查找对应帧，写入 `PGOFrame::rtk_chi2_`
- 供后续 outlier 判断使用

#### 13.5.4 YAML 配置

```yaml
pgo:
  # ... 现有配置 ...
  rtk_pos_noise: 0.1         # RTK 位置噪声标准差 (m)，仅在 per-frame 噪声 < 1e-4 时作为回退值
  rtk_ang_noise: 0.0052      # RTK 姿态噪声标准差 (rad)，≈0.3°，仅在 per-frame 噪声无效时回退
  rtk_outlier_th: 10.0       # RTK 异常值卡方阈值（Cauchy delta）
```

> **注意**：当前 PGO 参数通过 `PGOImpl::Options` 默认值初始化，YAML 中的 pgo 段为文档性质（与现有 `lidar_loc_pos_noise` 等参数一致）。运行时调参需修改 Options 默认值或新增 YAML 加载逻辑。

#### 13.5.5 关键设计决策

| 决策 | 理由 |
|------|------|
| 使用 `std::deque<RTKData>` 而非 `TimedPose` | RTKData 自带 per-measurement 噪声（pos_std/rot_std），PGO 因子可直接使用实际协方差 |
| 时间插值而非降采样 | 100Hz RTK → PGO 帧（1-5Hz），`math::PoseInterp` 位置线性插值 + 姿态 slerp |
| 噪声回退：per-frame → Options 默认 | 与 LoopClosing::AddRTKFactors() 一致 |
| Cauchy 鲁棒核 + `SetDelta()` | RTK 偶发 fix 丢失导致跳变，Cauchy 自动压制；构造函数不继承，必须用 `SetDelta` |
| `rtk_edges_` 向量追踪 + 优化后回写 chi2 | 与 LidarLoc 的 `lidar_loc_edges_` 模式一致，优化后 `ComputeError` 写回 `PGOFrame::rtk_chi2_` |
| `AddRTKFactors` 在 `is_in_map_` 下调用 | 确保地图加载、NDT 已启动后 RTK 才生效，避免初始化阶段的误约束（代价：NDT 持续失败时 RTK 不生效） |
| `AssignRTKPoseIfNeeded` 在 `is_in_map_` 通过后 | 避免为被拒绝的帧浪费 RTK 插值 |
| `PGO::ProcessRTK` 加锁，`PGOImpl::ProcessRTK` 不加 | 遵循现有 PGO 模式（锁在公开接口层） |

#### 13.5.6 设计审查修复（实现时发现并修复）

| 编号 | 严重度 | 问题 | 修复方式 |
|------|--------|------|----------|
| P1 | 🔴 | `make_shared<RobustKernelCauchy>(delta)` 不会编译（构造函数不继承） | 改用 `SetDelta(rtk_outlier_th)` |
| P2 | 🔴 | 无 `rtk_edges_` 向量 → `rtk_chi2_` 无处填充，outlier 检测不工作 | 添加 `rtk_edges_` 向量 + `RunOptimization` 后回写 chi2 |
| P3 | 🟡 | `AssignRTKPoseIfNeeded` 调用位置在 `is_in_map_` 之前 | 移到 `is_in_map_` 通过后、`RunOptimization` 前 |
| P5 | 🟡 | RTKData 插值的噪声提取逻辑未明确 | 噪声取自 `best_match`（最近帧），不参与 PoseInterp |
| P6 | 🟡 | `ProcessRTK` 返回类型不一致 | PGO 层 `bool`，PGOImpl 层 `void`，符合现有模式 |
| P8 | 🟢 | 缺少 `#include "common/rtk_data.h"` | 已在 `pgo_impl.h` 和 `pgo.h` 中添加 |
| P9 | 🟢 | `CleanProblem` 缺少 `rtk_edges_` 清理 | 已添加 `rtk_edges_.clear()` |
| P10 | 🟢 | YAML 配置更新（6 个文件） | 旧 `rtk_fix_*`/`rtk_other_*` 替换为 `rtk_pos_noise`/`rtk_ang_noise`/`rtk_outlier_th` |

---

### 13.6 ④ 定位系统 RTK 订阅 ✅ 已实现（2026-07-22）

> **实现状态**：已根据以下设计方案完成代码实现，并根据审查报告修复了全部问题。
> 实际实现与原始设计的主要差异：
> 1. 坐标转换逻辑提取为 `RTKConverter` 类（`rtk_utils.h/cc`），SLAM 和定位模式共用
> 2. `MapOrigin` 移入 `rtk_utils.h` 作为公共类型
> 3. `SlamSystem::ProcessRTK()` 重构为使用 `RTKConverter`
> 4. 使用带默认值的 `YAML_IO::GetValue(key, default)` 处理可选 RTK 配置
> 5. `ProcessRTKMsg` 不加锁（`lio_->ProcessRTK` 仅做 `deque::emplace_back`，Lock-Free 安全）
> 6. `ProcessRTK` 中转发步骤受 `loc_started_` 守卫

**背景**：`LaserMapping` 已有 `ProcessRTK(RTKData)` 方法，ESKF 的 `SyncPackages()` + `UndistortPcl()` 已支持 RTK 穿插更新。但定位模式的上层（`LocSystem` → `Localization`）没有订阅和传递 RTK 数据，导致定位时 LIO 前端拿不到 RTK 辅助。

**目标**：打通定位模式下的 RTK 数据通路，使 LIO 前端在定位时也能利用 RTK 观测约束 ESKF。

**涉及文件（共 4 个）**：

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `src/core/system/loc_system.h` | 修改 | 新增 RTK 订阅 + 原点管理 + Odometry→RTKData 解析 |
| `src/core/system/loc_system.cc` | 修改 | 订阅 RTK 话题、LLA→UTM→ENU 转换、原点初始化 |
| `src/core/localization/localization.h` | 修改 | 新增 `ProcessRTKMsg(RTKData)` 接口 |
| `src/core/localization/localization.cpp` | 修改 | 实现 `ProcessRTKMsg()`，转发给 `lio_->ProcessRTK()` |

**可选改动（离线模式）**：

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `src/app/run_loc_offline.cc` | 修改 | RosbagIO 加 RTK 话题回调 |

#### 13.6.1 改动 A：loc_system.h — 新增 RTK 相关成员

**关键决策：定位模式原点来源**

定位模式的地图是预先建好的，地理原点在 SLAM 阶段已确定。理想做法是从地图目录的 `georeference.yaml` 读取。当前阶段采用**运行时初始化**：累积前 10 帧 RTK 取中值确定原点（与 `SlamSystem` 逻辑一致），适用于定位 bag 的起始位置与建图 bag 相同的场景。

```cpp
// === 头文件新增 ===
#include "common/rtk_data.h"
#include "common/rtk_utils.h"
#include "nav_msgs/msg/odometry.hpp"

// RTK 订阅
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rtk_sub_ = nullptr;

// 方法声明
void ProcessRTK(const nav_msgs::msg::Odometry::SharedPtr& msg);
```

#### 13.6.2 改动 B：loc_system.cc — 实现 RTK 订阅和解析

**Init() 中新增**（在 cloud_sub_ 之后）：

```cpp
// 订阅 RTK 话题
std::string rtk_topic = yaml.GetValue<std::string>("common", "rtk_topic");
if (!rtk_topic.empty()) {
    rtk_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        rtk_topic, qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            ProcessRTK(msg);
        });
    LOG(INFO) << "loc: subscribed to RTK topic: " << rtk_topic;
} else {
    LOG(INFO) << "loc: no RTK topic configured, skipping";
}

// 读取姿态噪声配置
rtk_rot_noise_ = yaml.GetValue<double>("fasterlio", "rtk_rot_noise");
```

**ProcessRTK() 实现**：从 `SlamSystem::ProcessRTK()` 复制完整逻辑，包含：
1. LLA→UTM→ENU 坐标转换（通过 `RTKConverter`）
2. 中值滤波原点初始化（10 帧）
3. UTM 跨带检查 + 位置跳变检测（>10 m/s）
4. `loc_->ProcessRTKMsg(rtk)` 送入定位模块

> **重要**：loc 模式下 ProcessRTK 只调 `loc_->ProcessRTKMsg(rtk)`，**不调** `lio_->ProcessRTK()`。RTK 通过 Localization 中转后送达 LIO。

#### 13.6.3 改动 C：localization.h — 新增 ProcessRTKMsg 接口

```cpp
/// 处理 RTK/INS 观测数据（已转换至 ENU 坐标系）
void ProcessRTKMsg(const RTKData& rtk);
```

#### 13.6.4 改动 D：localization.cpp — 实现 ProcessRTKMsg

```cpp
void Localization::ProcessRTKMsg(const RTKData& rtk) {
    // 无需加锁：lio_->ProcessRTK() 仅做 deque::emplace_back
    if (lio_ != nullptr) {
        lio_->ProcessRTK(rtk);
    }
}
```

逻辑极简：仅转发给 LIO 前端。`LaserMapping::ProcessRTK()` 会将 RTK 入队 `rtk_buffer_`，之后由 `SyncPackages()` 收集到 `measures_.rtk_`，再由 `UndistortPcl()` 按时间线穿插 ESKF Update(GPS)。

#### 13.6.5 改动 E（可选）：run_loc_offline.cc — 离线模式 RTK 支持

在 `RosbagIO` 链式调用中新增 RTK 话题回调：

```cpp
rosbag
    .AddImuHandle(imu_topic, ...)
    .AddPointCloud2Handle(lidar_topic, ...)
    .AddOdometryHandle(rtk_topic,
                       [&loc](nav_msgs::msg::Odometry::SharedPtr msg) {
                           // 解析 Odometry → RTKData → loc.ProcessRTKMsg()
                           ...
                           return true;
                       })
    .Go();
```

#### 13.6.6 数据流对比

**改动前**（定位模式，RTK 缺失）：
```
Odometry msg → ❌ 无人订阅
```

**改动后**（定位模式，RTK 贯通）：
```
Odometry msg (nav_msgs)
    │  LocSystem::ProcessRTK()
    │  LLA→UTM→ENU → RTKData
    ▼
Localization::ProcessRTKMsg(rtk)
    │
    ▼
LaserMapping::ProcessRTK(rtk)     ← 已有方法，无需改动
    │  rtk_buffer_ 入队
    ▼
SyncPackages() → measures_.rtk_  ← 已有逻辑，无需改动
    │
    ▼
UndistortPcl() → ESKF.Update(GPS) ← 已有逻辑，无需改动
```

#### 13.6.7 注意事项

1. **原点一致性**：当前方案定位时独立初始化原点（10 帧中值滤波），要求定位 bag 起始位置与建图 bag 相同。若不同，ENU 坐标会有固定偏移。长期方案是在地图中保存 `georeference.yaml`，定位时直接读取建图时确定的原点。
2. **空话题处理**：若 YAML 中 `rtk_topic` 为空或未配置，跳过 RTK 订阅，定位正常退化为纯 LIO + LidarLoc 模式。
3. **RosbagIO 兼容**：`AddOdometryHandle` 需要确认 `wrapper/bag_io.h` 是否已实现。若未实现，用通用 topic 回调 + 手动反序列化替代。
4. **LaserMapping 无需改动**：`ProcessRTK()`、`SyncPackages()`、`UndistortPcl()` 中 RTK 逻辑已经完备，定位模式和 SLAM 模式共用同一套代码路径。

---

### 13.7 ⑤ georeference.yaml ✅ 已实现（2026-07-22）

> **实现状态**：已实现。`SaveMap()` 中通过 `rtk_converter_.GetOrigin()` 获取原点信息并写入 YAML；
> `LocSystem::Init()` 中通过 `rtk_converter_.LoadOrigin()` 加载（`RTKConverter` 内部实现）。

**背景**：建图时 `SlamSystem` 通过中值滤波确定了 RTK 地理原点，所有 ENU 坐标都是相对于该原点。但 `SaveMap()` 不会保存这个原点信息，导致定位时无法知道 ENU 坐标系对应的地球位置。

**目标**：建图时保存 `georeference.yaml`，定位时读取并使用同一原点，实现 ENU ↔ 地理坐标的双向映射。

**涉及文件（共 2 个）**：

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `src/core/system/slam.cc` | 修改 | `SaveMap()` 末尾写 `georeference.yaml` |
| `src/core/system/loc_system.cc` | 修改 | `Init()` 中读取 `georeference.yaml`，有则直接设原点 |

**与 TiledMap 的关系**：完全正交，互不影响。TiledMap 的所有操作（`index.txt`、瓦片加载、NDT 匹配）都在 ENU 米制坐标下，不知道也不关心地理原点。`georeference.yaml` 只影响 RTK 数据的 LLA→ENU 转换那一步。

#### 13.7.1 改动 A：slam.cc — SaveMap() 末尾写 georeference.yaml

```cpp
// 保存地理参考信息
if (rtk_converter_.IsOriginReady()) {
    const auto& origin = rtk_converter_.GetOrigin();
    std::ofstream geo_out(save_path + "georeference.yaml");
    if (geo_out.is_open()) {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "latitude" << YAML::Value << origin.latitude_;
        emitter << YAML::Key << "longitude" << YAML::Value << origin.longitude_;
        emitter << YAML::Key << "altitude" << YAML::Value << origin.altitude_;
        emitter << YAML::EndMap;
        emitter << YAML::Key << "utm" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "easting" << YAML::Value << origin.utm_.easting;
        emitter << YAML::Key << "northing" << YAML::Value << origin.utm_.northing;
        emitter << YAML::Key << "altitude" << YAML::Value << origin.utm_.altitude;
        emitter << YAML::Key << "zone" << YAML::Value << origin.utm_.zone;
        emitter << YAML::Key << "band" << YAML::Value << origin.utm_.band;
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
        geo_out << emitter.c_str();
        geo_out.close();
        LOG(INFO) << "georeference saved to " << save_path + "georeference.yaml";
    }
}
```

#### 13.7.2 改动 B：loc_system.cc — Init() 中读取 georeference.yaml

```cpp
// 尝试从地图目录加载地理原点信息
std::string geo_path = map_path + "georeference.yaml";
if (!rtk_converter_.LoadOrigin(geo_path) && std::filesystem::exists(geo_path)) {
    LOG(WARNING) << "loc: georeference.yaml 存在但解析失败，将使用运行时原点初始化";
}
```

#### 13.7.3 完整定位流程（georeference.yaml 存在时）

```
Init()
  │  读 georeference.yaml → rtk_origin_ 直接赋值
  │  读 index.txt → TiledMap 加载瓦片 (ENU 坐标)
  ▼
ProcessRTK(msg)
  │  LLA → UTM
  │  跳过原点初始化（rtk_origin_.valid_ == true）
  │  ENU = UTM - rtk_origin_.utm_   ← 和建图时完全一致的转换
  ▼
RTKData (ENU) → lio_->ProcessRTK()
  │
  ▼
ESKF Update(GPS) → ENU 坐标和 TiledMap 自动对齐 ✅
```

#### 13.7.4 与 ④ 的联动

④（定位系统 RTK 订阅）中描述的 `LocSystem::ProcessRTK()` 的中值滤波原点初始化在 `georeference.yaml` 存在时会被跳过。
两个方案是渐进增强的关系：

- 阶段 1（④ 单独实现）：定位时独立中值滤波初始化原点 → 要求定位起点≈建图起点
- 阶段 2（④ + ⑤ 一起）：从地图文件读取原点 → 定位可任意起点启动

---

### 13.8 ⑥ IMU→base_link 坐标系统一 ✅ 已实现（2026-07-22）

**动机**：INS 模块（ins5715）输出的 `ins_raw` RTK 位姿在 base_link（车体中心），但 ESKF 应在 imu_link（IMU 安装点）下解算。IMU 测量的是安装点处的加速度/角速度，若 ESKF 状态放在 base_link 上，转弯时需要补偿离心力 ω×(ω×r) 和切向加速度 α×r，不如直接把状态放在 imu_link 上干净。

**当前问题**：系统隐式假设 imu_link ≈ base_link，当 IMU 偏离车体中心较远时，RTK 观测存在固定偏移，且 IMU 数据（尤其转弯时）存在未补偿的误差。

**核心原则**：ESKF 全程在 imu_link 下工作，地图（LIO 输出）天然就是 imu_link，不转换。只在输入侧（RTK）和输出侧（TF 发布）做坐标变换。

#### 13.8.1 全链路坐标流

```
                 ┌── IMU 原始数据 ──→ ESKF Predict ──→ 状态在 imu_link ✅
                 │
RTK (base_link) ─┼──→ 杆臂补偿 ──→ ESKF Update(GPS) ──→ 约束 imu_link 状态
                 │   p_imu = p_base - R * t_imu_in_base
                 │
LiDAR (lidar)  ──┼──→ T_imu_lidar ──→ ICP ──→ 帧间匹配 ✅
                 │
                 └── 地图 (imu_link) ✅
                          │
                          ▼
              建图: 直接保存 (imu_link) ✅
                          │
                          ▼
           定位: NDT 匹配 (imu_link ↔ imu_link) ✅
                          │
                          ▼
          PGO 优化 (imu_link) ✅
                          │
                          ▼
   输出: pose_base = pose_imu * T_imu_base.inverse() → TF 发布 (base_link)
```

**对已有功能的影响**：

| 环节 | 当前 | 改后 | 变化 |
|------|------|------|------|
| ESKF Predict | IMU 直入 | IMU 直入 | 无 |
| LiDAR 预处理 | `T_imu_lidar` → imu_link | `T_imu_lidar` → imu_link | 无 |
| LIO 建图 | imu_link | imu_link | 无 |
| SaveMap | 直接存 | 直接存 | 无 |
| NDT 匹配 | imu_link ↔ imu_link | imu_link ↔ imu_link | 无 |
| RTK→ESKF | base_link 当 imu_link 用 | `T_imu = T_base * T_imu_base`（SE3 右乘，位置+旋转均补偿） | 杆臂补偿 |
| 位姿发布 | imu_link 标成 base_link 发 | `T_base = T_imu * T_imu_base.inverse()`（SE3 右乘逆） | 杆臂补偿 |
| TF 精度 | imu≠base 时有固定偏移 | 正确 | 修正 |

**涉及文件（共 10 个）**：

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `common/rtk_utils.h` | 修改 | 新增 `SetTImuBase()` 接口 + `T_imu_base_` 成员 |
| `common/rtk_utils.cc` | 修改 | `Convert()` 中新增杆臂补偿：`p -= R * T_imu_base.translation()` |
| `core/system/slam.cc` | 修改 | 从 YAML common 块加载 `T_imu_base`，设置到 `rtk_converter_` |
| `core/system/loc_system.cc` | 修改 | 加载 `T_imu_base`，设置到 `rtk_converter_` 和 `loc_->SetTImuBase()` |
| `core/localization/localization.h` | 修改 | 新增 `SetTImuBase()` + `T_imu_base_` 成员 |
| `core/localization/localization.cpp` | 修改 | TF 回调中传递 `T_imu_base_` 给 `ToGeoMsg()`；新增 LEISHEN lidar_type |
| `core/localization/localization_result.h` | 修改 | `ToGeoMsg()` 新增可选参数 `T_imu_base`（默认 Identity） |
| `core/localization/localization_result.cc` | 修改 | `ToGeoMsg()` 内实现输出侧杆臂补偿：`pose_ * T_imu_base.inverse()` |
| `io/yaml_io.h` | 修改 | 新增 `Exist()` 和 `GetStdVector<T>()` 辅助方法 |
| `config/default*.yaml`（6 个） | 修改 | `common` 块新增 `T_imu_base: [0.0, 0.0, 0.0]` |

#### 13.8.2 杆臂补偿公式（已修正）

**输入侧**（RTKConverter::Convert）：

```cpp
// 将 RTK 位姿从 base_link 转到 imu_link
// T_ENU_imu = T_ENU_base * T_imu_base  (SE3 右乘)
// 展开：R_imu = R_base * R,  p_imu = p_base + R_base * t
// 其中 t = T_imu_base.translation()（IMU 原点在 base_link 系下的位置向量）
if (!T_imu_base_.matrix().isIdentity(1e-10)) {
    rtk.position = rtk.position + rtk.orientation * T_imu_base_.translation();
    rtk.orientation = rtk.orientation * T_imu_base_.so3().unit_quaternion();
}
```

> **为什么是 `p + R*t` 而非 `p - R*t`？**运动链为 ENU ← base ← IMU，即 `T_ENU_imu = T_ENU_base · T_imu_base`。t 是 IMU 在 base_link 系下的位置向量，在 ENU 系下该向量的投影为 `R_base * t`。IMU 位置 = base_link 位置 + 旋转后的杆臂，故为加号。
>
> 例如：IMU 在 base_link 后方 2m（t = [-2, 0, 0]），车辆朝北时 IMU_ENU = base_ENU + [-2, 0, 0] = base 南侧 2m；车辆朝东时 IMU_ENU = base_ENU + [0, -2, 0] = base 西侧 2m。【参数名暗示：`T_imu_base` 变换方向为 IMU→base，所以从 base 推 IMU 用右乘 `T_ENU_base * T_imu_base`】

**输出侧**（LocalizationResult::ToGeoMsg）：

```cpp
// T_ENU_base = T_ENU_imu * T_imu_base.inverse()  (SE3 右乘逆)
// 展开：R_base = R_imu * R^T,  p_base = p_imu - R_imu * R^T * t
// 即 p_base = p_imu - R_imu * t（当 R=I 时）
SE3 pose_out = pose_;
if (!T_imu_base.matrix().isIdentity(1e-10)) {
    pose_out = pose_ * T_imu_base.inverse();
}
```

#### 13.8.3 YAML 配置

```yaml
common:
  T_imu_base: [0.0, 0.0, 0.0]  # IMU→base_link 杆臂 (m)，[x, y, z]，前/左/上，默认零杆臂
                               # 假设 IMU 和 base_link 方向已由双天线 INS 标定对齐（旋转分量 = Identity）
```

#### 13.8.4 关键设计决策

| 决策 | 理由 |
|------|------|
| 地图不转换 | LIO 地图天然在 imu_link，建图和定位都在同一坐标系下，无需变换 |
| T_imu_base 支持完整 SE3 | 位置和旋转分量均参与链式变换，当前旋转=Identity，但公式已覆盖一般情况 |
| 默认零杆臂 | 向后兼容现有配置，不设 `T_imu_base` 时行为不变 |
| ESKF 状态不动 | 只在观测输入和位姿输出两个边界做变换，核心算法完全透明 |
| 杆臂用 SE3 右乘 | 输入 `T_imu = T_base * T_imu_base`，输出 `T_base = T_imu * T_imu_base⁻¹` |
| TF 直接输出 base_link | 在 `ToGeoMsg()` 内完成补偿，`child_frame_id` 保持 `base_link`，下游无感知 |
| SLAM 模式无 TF 发布 | SlamSystem 不发布 TF（仅 LocSystem 发布），建图不受影响 |

#### 13.8.5 SLAM vs 定位模式

| 模式 | T_imu_base 生效位置 | 说明 |
|------|---------------------|------|
| SLAM 在线/离线 | `RTKConverter::Convert()`（输入侧） | LIO 前端 RTK 观测补偿杆臂后注入 ESKF |
| 定位在线 | `RTKConverter::Convert()`（输入侧）+ `ToGeoMsg()`（输出侧） | RTK 观测补偿 + TF 发布补偿，两端对齐 |

#### 13.8.6 与 `extrinsic_T/R` 的关系（重要）

`extrinsic_T/R`（fasterlio 块）代表 **sensor 在 IMU 系下的位姿**，用于点云去畸变时将 sensor 系点云变换到 IMU 系做运动补偿。

| 场景 | sensor | extrinsic 语义 | 与 T_imu_base 的关系 |
|------|--------|---------------|---------------------|
| 标准 LiDAR 直连 | LiDAR 坐标系 | LiDAR 在 IMU 下的位姿 (`T_imu_lidar`) | 独立配置，与 T_imu_base 无关 |
| LeiShen/merge_cloud | base_link 坐标系 | base_link 在 IMU 下的位姿 | `extrinsic = T_imu_base⁻¹`，即 `extrinsic_T = -Rᵀ*t`，`extrinsic_R = Rᵀ` |

> **注意**：merge_cloud 已将多雷达融合并转到 base_link 系，lightning-lm 拿到的点云 sensor 坐标系就是 base_link。去畸变流程中 `R_lidar_imu_ * P_i + t_lidar_mu_` 将点从 sensor 系转到 IMU 系再做运动补偿，当 sensor=base_link 时，此变换必须是 `T_imu_base⁻¹`，否则去畸变会因坐标系不匹配产生误差。
>
> **配置示例**（IMU 在 base_link 前方 2m，左侧 0.3m）：
> ```yaml
> common:
>   T_imu_base: [2.0, 0.3, 0.0]   # 前/左/上
> fasterlio:
>   extrinsic_T: [-2.0, -0.3, 0.0]  # = -T_imu_base
>   extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # unit matrix
> ```

---

### 13.9 与 fusions_slam 的对比

| | fusions_slam | lightning-lm（当前） |
|---|---|---|
| EKF 类型 | IESKF (18维) | ESKF (12维) |
| RTK 协方差 | 独立 P_ 矩阵 | 共用 P 矩阵 |
| RTK 穿插方式 | `mapDatas` 排序后按序处理 | `SyncPackages` 合并 + `UndistortPcl` 按序迭代 |
| 后端优化 | Ceres LIO-GPS 轨迹对齐 | miao 位姿图 |
| 坐标转换 | `rtk_switch.cpp: LLA2Ecef→ecef2Enu` | `rtk_utils.cc: LLA→UTM→ENU`（自实现，零依赖） |
| 杆臂补偿 | `T_imu_ant.inverse()` | ❌ INS 模块内部已做 |
| 时间对齐 | `syncMeasureGroupAdd()` 在 LiDAR 时间窗内对齐 | `SyncPackages` 裁剪 + `UndistortPcl` 按序穿插 |

---

### 13.10 实现顺序 & 待办汇总

| 序号 | 事项 | 章节 | 状态 |
|------|------|------|------|
| ① | ESKF 前端 | §13.3 | ✅ 已完成 |
| ② | 建图后端 LoopClosing RTK 先验边 | §13.4 | ✅ 已完成（2026-07-16） |
| ④ | 定位系统 RTK 订阅 | §13.6 | ✅ 已完成（2026-07-22） |
| ⑤ | georeference.yaml | §13.7 | ✅ 已完成（2026-07-22） |
| ③ | 定位后端 PGO RTK 因子 | §13.5 | ✅ 已完成（2026-07-22） |
| ⑥ | IMU→base_link 坐标系统一 | §13.8 | ✅ 已完成（2026-07-22） |
| — | 杆臂补偿（天线→IMU） | — | ❌ INS 模块内部已做 |
| — | RTK 类型支持 | — | 📋 暂缓 |

---

### 13.11 RTKConverter 重构记录（2026-07-22）

对定位模式 RTK 数据通路设计进行了全面审查，发现 7 个问题并全部修复。同时将 SLAM/定位共用的 RTK 坐标转换逻辑提取为 `RTKConverter` 类，消除代码重复。

#### 13.11.1 改动文件

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `common/rtk_utils.h` | 修改 | 新增公共 `MapOrigin` 结构体 + `RTKConverter` 类声明（含 `Convert()`、`LoadOrigin()`） |
| `common/rtk_utils.cc` | 修改 | 实现 `RTKConverter`：中值滤波原点初始化、UTM 跨带检查、位置跳变检测、georeference.yaml 加载 |
| `io/yaml_io.h` | 修改 | 新增带默认值的 `GetValue(node, key, default)` 重载（2 层 + 3 层），可选配置项不再抛异常 |
| `core/system/slam.h` | 修改 | 删除内部 `MapOrigin` 定义，改用 `rtk_utils.h` 公共类型；用 `RTKConverter` 替换原始缓冲 + 跳变检测成员 |
| `core/system/slam.cc` | 修改 | `ProcessRTK()` 简化为 5 行（调 `rtk_converter_.Convert()` + 转发）；`SaveMap()` 末尾写 `georeference.yaml` |
| `core/system/loc_system.h` | 修改 | 新增 RTK 订阅、`RTKConverter` 成员、`ProcessRTK()` 声明 |
| `core/system/loc_system.cc` | 修改 | 实现 RTK 订阅 + georeference.yaml 加载 + `ProcessRTK()` |
| `core/localization/localization.h` | 修改 | 新增 `ProcessRTKMsg(const RTKData&)` 声明 |
| `core/localization/localization.cpp` | 修改 | 实现 `ProcessRTKMsg()`，**不加锁**（仅 `deque::emplace_back`，避免与 ICP 锁竞争） |

#### 13.11.2 审查发现及修复

| 编号 | 严重度 | 问题 | 修复方式 |
|------|--------|------|----------|
| R1 | 🔴 | `YAML_IO::GetValue` 键不存在时抛异常 → RTK 可选逻辑无法生效 | 新增 `GetValue(key, default)` 重载，`yaml_node_[node] && yaml_node_[node][key]` 检查 |
| R2 | 🔴 | 离线定位模式 LLA→ENU 转换逻辑与在线模式完全重复 | 提取 `RTKConverter` 类，SLAM/定位两种模式共用 |
| R3 | 🟡 | `ProcessRTK` 不检查 `loc_started_`：原点初始化后可转发到未就绪的 LIO | 转发步骤加 `if (loc_started_)` 守卫，原点初始化独立于 `loc_started_` |
| R4 | 🟡 | `ProcessRTKMsg` 中 `global_mutex_` 与 ICP 计算产生不必要锁竞争 | 去掉锁，`lio_->ProcessRTK()` 仅做 `deque::emplace_back` |
| R5 | 🟡 | PGO RTK 噪声参数已定义但未从 YAML 加载 | 标注为技术债，待 PGO RTK 因子实现时处理 |
| R6 | 🟢 | 缺少 `<filesystem>` 头文件 | 已在 `loc_system.cc` 和 `rtk_utils.cc` 中添加 |
| R7 | 🟢 | 中值滤波缓冲在加载 georeference.yaml 后冗余 | RTKConverter 中原点就绪后 `clear()` + `shrink_to_fit()` 释放 |

---

### 13.12 历史审查记录

#### 13.12.1 代码审查修复（2026-07-14）

审查报告 `.hermes/review-suggestions.md` 共 10 条建议，已全部修复并验证编译通过。

| 编号 | 类别 | 问题 | 修复位置 | 修复方式 |
|------|------|------|----------|----------|
| S1 | 🔴 防御 | GPSObsModel 缺少 NaN/Inf 检查 | `laser_mapping.cc` | 入口处 `allFinite()` 校验，非法时 `obs.valid_=false` |
| S2 | 🟡 数值 | SO(3).log() π 附近奇异 | `laser_mapping.cc` | 旋转角 > 0.8π 时退化姿态残差为零 |
| S3 | 🟢 边界 | UTM 跨带坐标跳变 | `slam.cc` | 检测 zone 变化，跨带时 WARNING + return |
| S4 | 🟡 鲁棒 | 原点初始化对坏值敏感 | `slam.h/cc` | 简单平均 → 中值滤波（`std::sort` + 取中位） |
| S5 | 🔴 防御 | 噪声标准差为零时除零 | `laser_mapping.cc` | `1/σ²` 改为 `1/max(σ, 1e-6)²` |
| S6 | 🟢 可配 | 姿态噪声硬编码 0.0052 | `slam.h/cc` + `default.yaml` | 新增 YAML 项 `rtk_rot_noise`，支持不同 INS |
| S7 | 🟢 检测 | RTK 位置跳变无告警 | `slam.cc` | 速度 > 10 m/s 时 WARNING + return |
| S8 | 🔵 性能 | GPS Update 多余迭代 | `eskf.cc` | GPS 观测后强制 `converge_=true`，一次退出 |
| S9 | 🔵 测试 | 新增逻辑无测试覆盖 | `tests/rtk_utils_test.cc` | 6 个 LLA→UTM 测试用例，零外部依赖 |
| S10 | 🟢 注释 | UndistortPcl 注释不准确 | `imu_processing.hpp` | 修正为"近似对齐到 head 时刻" |

涉及文件：7 个（含 1 个新增测试文件），全部编译通过。

---

### 13.13 RTK 缺失行为分析（隧道等场景）

**结论：不会崩溃**，系统在所有层面都有防御：

| 层面 | RTK 缺失时行为 | 防护机制 |
|------|---------------|----------|
| ROS2 回调 | `ProcessRTK` 不被调用 | 无消息 → 无回调 |
| LIO 前端 `rtk_buffer_` | 无新数据入队 | `SyncPackages` 中 `measures_.rtk_` 为空 |
| `UndistortPcl` | ESKF 不调 `Update(GPS)` | `it_rtk == meas.rtk_.end()` → while 循环跳过 |
| `MakeKF` | 不关联过期 RTK | `dt < 0.5s` 时间窗口检查，超时跳过 |
| `LoopClosing::AddRTKFactors` | 不添加 RTK 先验边 | `HasRTK()` 返回 false → 跳过 |
| RTK 恢复后 | 跳变检测防御 | 速度 > 10 m/s 视为异常，跳过该帧；Cauchy 鲁棒核兜底 |

---

### 13.14 验证方式

```bash
# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lightning

# 离线建图验证（使用含 RTK 话题的 bag）
ros2 run lightning run_slam_offline --config ./config/default.yaml --input_bag <bag_with_rtk>

# 关键观察点
# - ESKF 收敛速度（RTK 初始化后协方差是否快速下降）
# - 轨迹与 RTK 真值的偏差
# - 长直路段无回环时是否漂移
# - GPS 短暂丢失后恢复能力
```

---

## 14. LeiShen（镭神）激光雷达适配 ✅ 已实现（2026-07-22）

### 14.1 背景

上游 `merge_cloud` 节点（`~/CS-Y2630-AD/src/localization/match/lidar_match/multi_lidar_merge/`）将 2-3 台激光雷达融合后，输出 `sensor_msgs::msg::PointCloud2`，点云格式为 `lslidar_driver::PointXYZIRT`：

```cpp
namespace lslidar_driver {
struct EIGEN_ALIGN16 PointXYZIRT {
    PCL_ADD_POINT4D;     // x, y, z (float)
    PCL_ADD_INTENSITY;   // intensity (float)
    std::uint16_t ring;  // 线号
    float time;          // 相对时间 (秒)
};
}
```

lightning-lm 需要在 `pointcloud_preprocess` 中新增此格式的反序列化支持。

### 14.2 与现有格式对比

| 字段 | LeiShen (`PointXYZIRT`) | Velodyne (`Point`) | lightning (`PointType`) |
|------|------------------------|---------------------|------------------------|
| x/y/z | float | float | float |
| intensity | float | float | float |
| ring | uint16_t | uint16_t | —（不保留）|
| time | float（秒） | float | double（毫秒）|

格式与 Velodyne 几乎一致（都有 ring + per-point time），区别仅在于 namespace 和时间单位。LeiShen 总是带有时戳，无需 Velodyne 的 `given_offset_time_` 回退逻辑。

### 14.3 实现方案

**涉及文件（共 3 个）**：

| 文件 | 改动类型 | 说明 |
|------|----------|------|
| `common/point_def.h` | 修改 | 新增 `lightning::LeiShenPoint` 结构体 + `POINT_CLOUD_REGISTER_POINT_STRUCT` |
| `core/lio/pointcloud_preprocess.h` | 修改 | 新增 `LidarType::LEISHEN = 5` + `LeiShenHandler` 声明 |
| `core/lio/pointcloud_preprocess.cc` | 修改 | 实现 `LeiShenHandler` + `Process()` switch 新增 LEISHEN case |
| `core/localization/localization.cpp` | 修改 | lidar_type switch 新增 `LEISHEN`（LEISHEN = 5） |

**注意**：`merge_cloud` 已在外部将多雷达融合并转换到车体坐标系，lightning-lm 不需要了解雷达外参，只需直接使用融合后的点云。

#### 改动 A：`point_def.h` — 新增 LeiShen 点云结构体

为避免与官方 `lslidar_driver` SDK 链接时的 ODR 冲突（`POINT_CLOUD_REGISTER_POINT_STRUCT` 生成全局模板特化），使用 `lightning::LeiShenPoint` 而非 `lslidar_driver::PointXYZIRT`。`pcl::fromROSMsg` 按字段名匹配，不依赖 namespace 一致性。

```cpp
/// 镭神（LeiShen）激光雷达点云结构体
/// 注意：放在 lightning 命名空间下而非 lslidar_driver，避免与官方 SDK 的
/// POINT_CLOUD_REGISTER_POINT_STRUCT 产生 ODR 冲突（pcl::fromROSMsg 按字段名匹配）
namespace lightning {
struct EIGEN_ALIGN16 LeiShenPoint {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;  // 相对时间，单位：秒
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace lightning

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(lightning::LeiShenPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))
// clang-format on
```

#### 改动 B：`pointcloud_preprocess.h` — LidarType 枚举

```cpp
enum class LidarType { AVIA = 1, VELO32 = 2, OUST64 = 3, ROBOSENSE = 4, LEISHEN = 5 };
```

新增方法声明：

```cpp
void LeiShenHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
```

#### 改动 C：`pointcloud_preprocess.cc` — 实现 LeiShenHandler

```cpp
void PointCloudPreprocess::LeiShenHandler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    // 镭神激光雷达点云预处理
    // 与 Velodyne 格式类似（x/y/z/intensity/ring/time），但 time 字段单位为 float 秒
    // 注意：不使用 time_scale_（LeiShen 时间单位固定为秒），直接 * 1e3 转为毫秒
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<lightning::LeiShenPoint> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < plsize; i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x +
                       pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        // time: float 秒 → double 毫秒（固定 ×1e3，不依赖 time_scale_）
        added_pt.time = static_cast<double>(pl_orig.points[i].time) * 1e3;

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}
```

#### 改动 D：`Process()` switch 新增分支

```cpp
case LidarType::LEISHEN:
    LeiShenHandler(msg);
    break;
```

#### 改动 E：`localization.cpp` — 定位模块 lidar_type 识别

```cpp
} else if (lidar_type == 5) {
    preprocess_->SetLidarType(LidarType::LEISHEN);
    LOG(INFO) << "Using LeiShen Lidar";
}
```

### 14.4 关键设计决策

| 决策 | 理由 |
|------|------|
| 使用 `lightning::LeiShenPoint` 而非 `lslidar_driver::PointXYZIRT` | 避免与官方 SDK 的 `POINT_CLOUD_REGISTER_POINT_STRUCT` 产生 ODR 冲突；`pcl::fromROSMsg` 按字段名匹配 |
| **不使用** `time_scale_` 配置 | LeiShen 的 `time` 字段单位固定为 float 秒，直接 `× 1e3` 转为毫秒；若复用 Velodyne 的 `time_scale_=1e-3` 会将 0.05s 误转为 0.00005ms |
| 无 `given_offset_time_` 回退 | LeiShen 总是输出 per-point timestamps |
| 不在 `merge_cloud` 中处理外参 | lightning-lm 直接使用融合后点云，雷达间外参对 LIO 透明 |
| 与 VelodyneHandler 保持独立 | 虽然格式相似但时间转换逻辑不同，独立 Handler 保持清晰的代码语义 |
| `extrinsic_T/R` 改为单位阵或 `T_imu_base⁻¹` | merge_cloud 输出已在 base_link 系，sensor 不再是 LiDAR 而是 base_link；去畸变前需将 sensor 系点云转到 IMU 系，当 IMU≠base_link 时需设 `extrinsic = T_imu_base⁻¹`（见 §13.8.6） |
