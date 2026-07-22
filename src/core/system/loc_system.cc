//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"

#include <filesystem>

#include "common/rtk_data.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
#ifdef USE_LIVOX
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");
#endif

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

#ifdef USE_LIVOX
    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });
#endif

    // 尝试从地图目录加载地理原点（建图时保存的 georeference.yaml）
    std::string geo_path = map_path + "georeference.yaml";
    if (!rtk_converter_.LoadOrigin(geo_path) && std::filesystem::exists(geo_path)) {
        LOG(WARNING) << "loc: georeference.yaml 存在但解析失败，将使用运行时原点初始化";
    }

    // 订阅 RTK 话题（可选，未配置时跳过）
    rtk_topic_ = yaml.GetValue<std::string>("common", "rtk_topic", std::string(""));
    rtk_rot_noise_ = yaml.GetValue<double>("fasterlio", "rtk_rot_noise", 0.0052);
    rtk_converter_.SetRotNoise(rtk_rot_noise_);

    // 读取 IMU→base_link 杆臂外参（可选，默认零杆臂）
    Vec3d T_imu_base_vec(0, 0, 0);
    if (yaml.Exist("common", "T_imu_base")) {
        auto t = yaml.GetStdVector<double>("common", "T_imu_base");
        if (t.size() == 3) {
            T_imu_base_vec = Vec3d(t[0], t[1], t[2]);
            rtk_converter_.SetTImuBase(SE3(SO3(), T_imu_base_vec));
            LOG(INFO) << "loc: T_imu_base = " << T_imu_base_vec.transpose();
        }
    }

    if (!rtk_topic_.empty()) {
        rtk_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            rtk_topic_, qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) { ProcessRTK(msg); });
        LOG(INFO) << "loc: subscribed to RTK topic: " << rtk_topic_;
    } else {
        LOG(INFO) << "loc: no RTK topic configured, skipping";
    }

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) { tf_broadcaster_->sendTransform(pose); });
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        // 传递 IMU→base_link 杆臂外参到定位模块（用于 TF 输出补偿）
        loc_->SetTImuBase(SE3(SO3(), T_imu_base_vec));
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

#ifdef USE_LIVOX
void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}
#endif

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

void LocSystem::ProcessRTK(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    RTKData rtk;
    if (rtk_converter_.Convert(*msg, rtk)) {
        // 仅定位已启动时才转发 RTK 到 LIO（避免 ESKF 未初始化时误注入）
        // 原点初始化由 RTKConverter 内部独立完成，不受 loc_started_ 影响
        if (loc_started_) {
            loc_->ProcessRTKMsg(rtk);
        }
    }
}

}  // namespace lightning