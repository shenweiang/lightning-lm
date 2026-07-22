//
// LLA → UTM 转换实现
// 基于 WGS84 椭球参数，使用标准 UTM 投影公式
//

#include "common/rtk_utils.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

#include "common/eigen_types.h"
#include "common/rtk_data.h"
#include "nav_msgs/msg/odometry.hpp"

namespace lightning {

UTMCoordinate LLAtoUTM(double lat, double lon, double alt) {
    // WGS84 椭球参数
    constexpr double a = 6378137.0;            // 半长轴 (m)
    constexpr double f = 1.0 / 298.257223563;  // 扁率
    constexpr double k0 = 0.9996;              // UTM 比例因子
    constexpr double e2 = 2.0 * f - f * f;     // 第一偏心率平方
    constexpr double ep2 = e2 / (1.0 - e2);    // 第二偏心率平方 e'²

    // 计算 UTM 带号
    int zone = static_cast<int>((lon + 180.0) / 6.0) + 1;

    // 计算中央子午线经度
    double lon0 = (zone - 1) * 6.0 - 180.0 + 3.0;

    // 转为弧度
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double lon0_rad = lon0 * M_PI / 180.0;

    // 预计算三角函数
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double tan_lat = std::tan(lat_rad);

    // 卯酉圈曲率半径
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    double T = tan_lat * tan_lat;
    double C = ep2 * cos_lat * cos_lat;
    double A = (lon_rad - lon0_rad) * cos_lat;

    // 子午线弧长 M
    double M = a * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * lat_rad -
                    (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * lat_rad) +
                    (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * lat_rad) -
                    (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * lat_rad));

    // 东向坐标 (Easting)
    double easting = k0 * N *
                         (A + (1.0 - T + C) * A * A * A / 6.0 +
                          (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A * A * A * A * A / 120.0) +
                     500000.0;

    // 北向坐标 (Northing)
    double northing =
        k0 * (M + N * tan_lat *
                      (A * A / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 +
                       (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A * A * A * A * A * A / 720.0));

    // 南半球修正
    char band = 'N';
    if (lat < 0.0) {
        northing += 10000000.0;
    }

    return {easting, northing, alt, zone, band};
}

// ============================================================================
// RTKConverter 实现
// ============================================================================

RTKConverter::RTKConverter() : options_(Options()) {}
RTKConverter::RTKConverter(Options options) : options_(options) {}

bool RTKConverter::LoadOrigin(const std::string& geo_yaml_path) {
    if (!std::filesystem::exists(geo_yaml_path)) {
        LOG(INFO) << "[RTKConv] georeference.yaml 不存在: " << geo_yaml_path << "，将使用运行时初始化";
        return false;
    }

    try {
        auto geo = YAML::LoadFile(geo_yaml_path);
        origin_.latitude_ = geo["origin"]["latitude"].as<double>();
        origin_.longitude_ = geo["origin"]["longitude"].as<double>();
        origin_.altitude_ = geo["origin"]["altitude"].as<double>();
        origin_.utm_.easting = geo["utm"]["easting"].as<double>();
        origin_.utm_.northing = geo["utm"]["northing"].as<double>();
        origin_.utm_.altitude = geo["utm"]["altitude"].as<double>();
        origin_.utm_.zone = geo["utm"]["zone"].as<int>();
        origin_.valid_ = true;

        LOG(INFO) << "[RTKConv] 从 georeference.yaml 加载原点: "
                  << "lat=" << origin_.latitude_ << " lon=" << origin_.longitude_
                  << " zone=" << origin_.utm_.zone;
        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "[RTKConv] 加载 georeference.yaml 失败: " << e.what();
        return false;
    }
}

bool RTKConverter::Convert(const nav_msgs::msg::Odometry& msg, RTKData& rtk) {
    // 1. 从 Odometry 消息中提取经纬高
    //    注意：ins5715 驱动将 position.x/y/z 分别编码为 lat/lon/alt
    double lat = msg.pose.pose.position.x;  // 纬度 (度)
    double lon = msg.pose.pose.position.y;  // 经度 (度)
    double alt = msg.pose.pose.position.z;  // 高程 (m)

    // 2. 提取姿态（驱动已转换为 ENU 系）
    Quatd orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z);

    // 3. 提取协方差（ins5715 驱动自定义的索引位置）
    double var_x = msg.pose.covariance[0];  // 东向方差 (lon_std²)
    double var_y = msg.pose.covariance[7];  // 北向方差 (lat_std²)
    double var_z = msg.pose.covariance[8];  // 高程方差 (alt_std², 非标准 Odometry 索引)

    // 4. LLA → UTM 坐标转换
    UTMCoordinate utm = LLAtoUTM(lat, lon, alt);

    // 5. UTM 跨带边界检查 —— 跨越 zone 边界时 ENU 差分产生数百米跳变
    if (origin_.valid_ && utm.zone != origin_.utm_.zone) {
        LOG(WARNING) << "[RTKConv] UTM zone changed: " << origin_.utm_.zone << " -> " << utm.zone
                     << "，跨带场景暂不支持，跳过此帧";
        return false;
    }

    // 6. 累积多帧 RTK 数据，取中值后初始化地理原点（对单帧坏值不敏感）
    if (!origin_.valid_) {
        origin_lats_.push_back(lat);
        origin_lons_.push_back(lon);
        origin_alts_.push_back(alt);
        origin_init_count_++;

        if (origin_init_count_ >= options_.origin_init_frames) {
            std::sort(origin_lats_.begin(), origin_lats_.end());
            std::sort(origin_lons_.begin(), origin_lons_.end());
            std::sort(origin_alts_.begin(), origin_alts_.end());
            double med_lat = origin_lats_[options_.origin_init_frames / 2];
            double med_lon = origin_lons_[options_.origin_init_frames / 2];
            double med_alt = origin_alts_[options_.origin_init_frames / 2];

            origin_.utm_ = LLAtoUTM(med_lat, med_lon, med_alt);
            origin_.latitude_ = med_lat;
            origin_.longitude_ = med_lon;
            origin_.altitude_ = med_alt;
            origin_.valid_ = true;

            LOG(INFO) << "[RTKConv] 地理原点已初始化 (中值滤波, " << origin_init_count_ << " 帧): "
                      << "lat=" << med_lat << " lon=" << med_lon << " alt=" << med_alt
                      << " zone=" << origin_.utm_.zone;

            // 释放初始化缓冲区（后续不再需要）
            origin_lats_.clear();
            origin_lons_.clear();
            origin_alts_.clear();
            origin_lats_.shrink_to_fit();
            origin_lons_.shrink_to_fit();
            origin_alts_.shrink_to_fit();
        }
        return false;  // 原点未就绪前，不向后端发送 RTK 数据
    }

    // 7. 计算相对于原点的 ENU 坐标
    rtk.timestamp = static_cast<double>(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9;
    rtk.position = Vec3d(utm.easting - origin_.utm_.easting, utm.northing - origin_.utm_.northing,
                         utm.altitude - origin_.utm_.altitude);
    rtk.orientation = orientation;
    rtk.pos_std = Vec3d(std::sqrt(std::max(var_x, 1e-6)), std::sqrt(std::max(var_y, 1e-6)),
                        std::sqrt(std::max(var_z, 1e-6)));
    rtk.rot_std = Vec3d(options_.rot_noise, options_.rot_noise, options_.rot_noise);

    // 8. RTK 位置跳变检测 —— fix 丢失时 INS 积分漂移可产生数米跳变
    if (last_valid_rtk_.timestamp > 0) {
        double jump = (rtk.position - last_valid_rtk_.position).norm();
        double dt = rtk.timestamp - last_valid_rtk_.timestamp;
        if (dt > 0 && jump / dt > options_.max_jump_speed) {
            LOG(WARNING) << "[RTKConv] 位置跳变: " << jump << "m in " << dt << "s, skipping";
            return false;
        }
    }
    last_valid_rtk_ = rtk;

    return true;
}

}  // namespace lightning
