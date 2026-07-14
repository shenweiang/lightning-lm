//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "lightning/srv/save_map.hpp"
#ifdef USE_LIVOX
#include "livox_ros_driver2/msg/custom_msg.hpp"
#endif

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/rtk_data.h"
#include "common/rtk_utils.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    using SaveMapService = srv::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
#ifdef USE_LIVOX
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);
#endif

    /// 处理RTK/INS观测数据
    void ProcessRTK(const nav_msgs::msg::Odometry::SharedPtr& msg);

    /// 实时模式下的spin
    void Spin();

   private:
    /// ros端保存地图的实现
    void SaveMap(const SaveMapService::Request::SharedPtr request, SaveMapService::Response::SharedPtr response);

    Options options_;
    std::atomic_bool running_ = false;

    rclcpp::Service<SaveMapService>::SharedPtr savemap_service_ = nullptr;

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
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

    /// RTK 地理原点（积累多帧取平均后确定，提高原点稳定性）
    struct MapOrigin {
        bool valid_ = false;
        double latitude_ = 0.0;   ///< 原点纬度 (度)
        double longitude_ = 0.0;  ///< 原点经度 (度)
        double altitude_ = 0.0;   ///< 原点高程 (m)
        UTMCoordinate utm_;       ///< 原点 UTM 坐标
    };
    MapOrigin rtk_origin_;

    /// RTK 原点初始化 —— 中值滤波（对单帧坏值不敏感）
    static constexpr int kRTKOriginInitFrames_ = 10;  ///< 原点初始化所需的 RTK 帧数
    int rtk_origin_init_count_ = 0;                   ///< 已累积帧数
    std::vector<double> rtk_origin_lats_;              ///< 纬度样本
    std::vector<double> rtk_origin_lons_;              ///< 经度样本
    std::vector<double> rtk_origin_alts_;              ///< 高程样本

    double rtk_rot_noise_ = 0.0052;  ///< RTK 姿态观测噪声 (rad)，默认 0.3°，从 YAML 读取

    /// RTK 位置跳变检测
    RTKData last_valid_rtk_;  ///< 上一帧有效 RTK，用于跳变检测
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
