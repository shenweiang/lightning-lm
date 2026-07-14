//
// RTK/INS 观测数据结构体
// 存储经过坐标转换后的 ENU 位姿及协方差信息
//

#pragma once

#include "common/eigen_types.h"

namespace lightning {

/// RTK 观测数据（已转换至本地 ENU 坐标系）
struct RTKData {
    double timestamp = 0.0;                 ///< 时间戳 (秒)
    Vec3d position = Vec3d::Zero();         ///< ENU 位置 (m)
    Quatd orientation = Quatd::Identity();  ///< ENU 姿态四元数
    Vec3d pos_std = Vec3d::Zero();          ///< 位置标准差 (m), x/y/z
    Vec3d rot_std = Vec3d::Zero();          ///< 姿态标准差 (rad), roll/pitch/yaw
};

}  // namespace lightning
