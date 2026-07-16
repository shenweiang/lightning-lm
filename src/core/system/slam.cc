//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
#ifdef USE_LIVOX
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
#endif
        rtk_topic_ = yaml["common"]["rtk_topic"].as<std::string>();
        rtk_rot_noise_ = yaml["fasterlio"]["rtk_rot_noise"].as<double>();

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

        rtk_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            rtk_topic_, qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) { ProcessRTK(msg); });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

#ifdef USE_LIVOX
void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}
#endif

void SlamSystem::ProcessRTK(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    if (running_ == false) {
        return;
    }

    // 1. 从 Odometry 消息中提取经纬高
    //    注意：ins5715 驱动将 position.x/y/z 分别编码为 lat/lon/alt
    double lat = msg->pose.pose.position.x;  // 纬度 (度)
    double lon = msg->pose.pose.position.y;  // 经度 (度)
    double alt = msg->pose.pose.position.z;  // 高程 (m)

    // 2. 提取姿态（驱动已转换为 ENU 系）
    Quatd orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z);

    // 3. 提取协方差（ins5715 驱动自定义的索引位置）
    double var_x = msg->pose.covariance[0];  // 东向方差 (lon_std²)
    double var_y = msg->pose.covariance[7];  // 北向方差 (lat_std²)
    double var_z = msg->pose.covariance[8];  // 高程方差 (alt_std², 非标准 Odometry 索引)

    // 4. LLA → UTM 坐标转换
    UTMCoordinate utm = LLAtoUTM(lat, lon, alt);

    // S3: UTM 跨带边界检查 —— 跨越 zone 边界时 ENU 差分产生数百米跳变
    if (rtk_origin_.valid_ && utm.zone != rtk_origin_.utm_.zone) {
        LOG(WARNING) << "[RTK] UTM zone changed: " << rtk_origin_.utm_.zone << " -> " << utm.zone
                     << ", 跨带场景暂不支持，跳过此帧";
        return;
    }

    // 5. 累积多帧 RTK 数据，取中值后初始化地理原点（S4: 中值滤波对单帧坏值不敏感）
    if (!rtk_origin_.valid_) {
        rtk_origin_lats_.push_back(lat);
        rtk_origin_lons_.push_back(lon);
        rtk_origin_alts_.push_back(alt);
        rtk_origin_init_count_++;

        if (rtk_origin_init_count_ >= kRTKOriginInitFrames_) {
            std::sort(rtk_origin_lats_.begin(), rtk_origin_lats_.end());
            std::sort(rtk_origin_lons_.begin(), rtk_origin_lons_.end());
            std::sort(rtk_origin_alts_.begin(), rtk_origin_alts_.end());
            double avg_lat = rtk_origin_lats_[kRTKOriginInitFrames_ / 2];
            double avg_lon = rtk_origin_lons_[kRTKOriginInitFrames_ / 2];
            double avg_alt = rtk_origin_alts_[kRTKOriginInitFrames_ / 2];

            rtk_origin_.utm_ = LLAtoUTM(avg_lat, avg_lon, avg_alt);
            rtk_origin_.latitude_ = avg_lat;
            rtk_origin_.longitude_ = avg_lon;
            rtk_origin_.altitude_ = avg_alt;
            rtk_origin_.valid_ = true;

            LOG(INFO) << "[RTK] 地理原点已初始化 (中值滤波, " << rtk_origin_init_count_ << " 帧): "
                      << "lat=" << avg_lat << " lon=" << avg_lon << " alt=" << avg_alt
                      << " zone=" << rtk_origin_.utm_.zone;
        }
        return;  // 原点未就绪前，不向后端发送 RTK 数据
    }

    // 6. 计算相对于原点的 ENU 坐标
    RTKData rtk;
    rtk.timestamp = ToSec(msg->header.stamp);
    rtk.position = Vec3d(utm.easting - rtk_origin_.utm_.easting, utm.northing - rtk_origin_.utm_.northing,
                         utm.altitude - rtk_origin_.utm_.altitude);
    rtk.orientation = orientation;
    rtk.pos_std =
        Vec3d(std::sqrt(std::max(var_x, 1e-6)), std::sqrt(std::max(var_y, 1e-6)), std::sqrt(std::max(var_z, 1e-6)));
    // S6: 姿态噪声从 YAML 读取（默认 0.0052 rad ≈ 0.3°），支持不同 INS 模块配置
    rtk.rot_std = Vec3d(rtk_rot_noise_, rtk_rot_noise_, rtk_rot_noise_);

    // S7: RTK 位置跳变检测 —— fix 丢失时 INS 积分漂移可产生数米跳变
    if (last_valid_rtk_.timestamp > 0) {
        double jump = (rtk.position - last_valid_rtk_.position).norm();
        double dt = rtk.timestamp - last_valid_rtk_.timestamp;
        if (dt > 0 && jump / dt > 10.0) {  // >10 m/s 视为异常
            LOG(WARNING) << "[RTK] 位置跳变: " << jump << "m in " << dt << "s, skipping";
            return;
        }
    }
    last_valid_rtk_ = rtk;

    // 7. 送入激光前端
    lio_->ProcessRTK(rtk);
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
