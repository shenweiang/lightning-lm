//
// Created by xiang on 24-4-11.
//

#include "localization_result.h"
#include "core/lightning_math.hpp"

namespace lightning::loc {

geometry_msgs::msg::TransformStamped LocalizationResult::ToGeoMsg(const SE3& T_imu_base) const {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = math::FromSec(timestamp_);
    msg.child_frame_id = "base_link";

    // 杆臂补偿：imu_link → base_link（SE3 右乘）
    // pose_base = pose_imu * T_imu_base.inverse()
    // 即 p_base = p_imu - R_imu * t（t = IMU 在 base_link 系下的位置向量）
    SE3 pose_out = pose_;
    if (!T_imu_base.matrix().isIdentity(1e-10)) {
        pose_out = pose_ * T_imu_base.inverse();
    }

    msg.transform.translation.x = pose_out.translation().x();
    msg.transform.translation.y = pose_out.translation().y();
    msg.transform.translation.z = pose_out.translation().z();

    msg.transform.rotation.x = pose_out.so3().unit_quaternion().x();
    msg.transform.rotation.y = pose_out.so3().unit_quaternion().y();
    msg.transform.rotation.z = pose_out.so3().unit_quaternion().z();
    msg.transform.rotation.w = pose_out.so3().unit_quaternion().w();

    return msg;
}

NavState LocalizationResult::ToNavState() const {
    NavState ret;
    ret.timestamp_ = timestamp_;
    ret.confidence_ = confidence_;
    ret.pos_ = pose_.translation();
    ret.rot_ = pose_.so3();
    ret.pose_is_ok_ = status_ == LocalizationStatus::GOOD;

    ret.vel_ = (pose_.so3() * vel_b_);

    return ret;
}

}  // namespace lightning::loc
