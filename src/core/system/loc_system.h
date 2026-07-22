//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef USE_LIVOX
#include "livox_ros_driver2/msg/custom_msg.hpp"
#endif

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/rtk_utils.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
#ifdef USE_LIVOX
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);
#endif

    /// 处理RTK/INS观测数据（LLA 解码 → ENU 转换 → 送入定位模块）
    void ProcessRTK(const nav_msgs::msg::Odometry::SharedPtr& msg);

    /// 实时模式下的spin
    void Spin();

   private:
    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;

    std::string imu_topic_;
    std::string cloud_topic_;
#ifdef USE_LIVOX
    std::string livox_topic_;
#endif

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
#endif

    std::string rtk_topic_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rtk_sub_ = nullptr;

    /// RTK 坐标转换器（LLA→UTM→ENU + 原点初始化 + 跳变检测）
    RTKConverter rtk_converter_;

    double rtk_rot_noise_ = 0.0052;  ///< RTK 姿态观测噪声 (rad)，默认 0.3°，从 YAML 读取
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
