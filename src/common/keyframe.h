//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_KEYFRAME_H
#define LIGHTNING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/rtk_data.h"
#include "common/std_types.h"

namespace lightning {

/// 关键帧描述
/// NOTE: 在添加后端后，需要加锁
class Keyframe {
   public:
    using Ptr = std::shared_ptr<Keyframe>;

    Keyframe() {}
    Keyframe(unsigned long id, CloudPtr cloud, NavState state)
        : id_(id), cloud_(cloud), state_(state), pose_lio_(state.GetPose()) {
        timestamp_ = state_.timestamp_;
        pose_opt_ = pose_lio_;
    }

    unsigned long GetID() const { return id_; }
    CloudPtr GetCloud() const { return cloud_; }

    SE3 GetLIOPose() {
        UL lock(data_mutex_);
        return pose_lio_;
    }

    void SetLIOPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_ = pose;

        // also set opt
        pose_opt_ = pose_lio_;
    }

    SE3 GetOptPose() {
        UL lock(data_mutex_);
        return pose_opt_;
    }

    void SetOptPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_ = pose;
    }

    void SetState(NavState s) {
        UL lock(data_mutex_);
        state_ = s;
    }

    NavState GetState() {
        UL lock(data_mutex_);
        return state_;
    }

    /// 设置 RTK 观测（在 MakeKF 时调用，设置后不可变）
    void SetRTKData(const RTKData& rtk) {
        UL lock(data_mutex_);
        rtk_data_ = rtk;
        rtk_valid_ = true;
    }

    /// 是否有关联的 RTK 观测（加锁防止 ThreadSanitizer 告警）
    bool HasRTK() {
        UL lock(data_mutex_);
        return rtk_valid_;
    }

    /// 获取 RTK 位姿（将 position + orientation 打包为 SE3）
    SE3 GetRTKPose() {
        UL lock(data_mutex_);
        return SE3(rtk_data_.orientation, rtk_data_.position);
    }

    /// 获取 RTK 位置噪声标准差 (m)
    Vec3d GetRTKPosStd() {
        UL lock(data_mutex_);
        return rtk_data_.pos_std;
    }

    /// 获取 RTK 姿态噪声标准差 (rad)
    Vec3d GetRTKRotStd() {
        UL lock(data_mutex_);
        return rtk_data_.rot_std;
    }

   protected:
    unsigned long id_ = 0;

    double timestamp_ = 0;
    CloudPtr cloud_ = nullptr;  /// 降采样之后的点云

    std::mutex data_mutex_;
    SE3 pose_lio_;  // 前端的pose
    SE3 pose_opt_;  // 后端优化后的pose

    NavState state_;  // 卡尔曼滤波器状态

    RTKData rtk_data_;          // 该关键帧关联的最近 RTK 观测
    bool rtk_valid_ = false;    // RTK 观测是否有效
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_H
