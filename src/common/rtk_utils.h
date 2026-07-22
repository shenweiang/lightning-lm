//
// LLA ↔ UTM ↔ ENU 坐标转换工具
// 不依赖 geodesy 库，直接实现 WGS84 椭球下的 UTM 投影
//

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>

#include "common/rtk_data.h"

namespace lightning {

/// UTM 坐标结构体
struct UTMCoordinate {
    double easting = 0.0;   ///< 东向坐标 (m)
    double northing = 0.0;  ///< 北向坐标 (m)
    double altitude = 0.0;  ///< 高程 (m)
    int zone = 0;           ///< UTM 带号 (1-60)
    char band = 'N';        ///< UTM 纬度带 ('C' - 'X')
};

/// 地图地理原点（建图时确定的参考点，所有 ENU 坐标相对于此原点）
struct MapOrigin {
    bool valid_ = false;
    double latitude_ = 0.0;   ///< 原点纬度 (度)
    double longitude_ = 0.0;  ///< 原点经度 (度)
    double altitude_ = 0.0;   ///< 原点高程 (m)
    UTMCoordinate utm_;       ///< 原点 UTM 坐标
};

/**
 * 将 WGS84 经纬高转换为 UTM 坐标
 * @param lat 纬度 (度, 北正)
 * @param lon 经度 (度, 东正)
 * @param alt 高程 (米)
 * @return UTM 坐标
 */
UTMCoordinate LLAtoUTM(double lat, double lon, double alt);

/**
 * RTK 坐标转换器：将 nav_msgs::Odometry (LLA) 转换为 RTKData (ENU)
 *
 * 封装了完整的转换链：LLA→UTM→ENU + 原点初始化 + 跨带检查 + 跳变检测。
 * 在线模式和离线模式可共用，避免代码重复。
 *
 * 使用示例：
 * @code
 *   RTKConverter converter;
 *   converter.SetRotNoise(0.0052);
 *   // 可选：从地图加载预存原点
 *   converter.LoadOrigin("./data/map/georeference.yaml");
 *
 *   RTKData rtk;
 *   if (converter.Convert(*odom_msg, rtk)) {
 *       lio_->ProcessRTK(rtk);
 *   }
 * @endcode
 */
class RTKConverter {
   public:
    struct Options {
        int origin_init_frames = 10;       ///< 原点初始化所需帧数（中值滤波）
        double rot_noise = 0.0052;         ///< 姿态噪声标准差 (rad)，≈0.3°，可从 YAML 覆盖
        double max_jump_speed = 10.0;      ///< 位置跳变阈值 (m/s)，超过视为异常
    };

    RTKConverter();                        ///< 使用默认选项构造
    explicit RTKConverter(Options options);  ///< 使用自定义选项构造

    /// 从 Odometry 消息（LLA 编码）转换为 ENU 坐标系下的 RTKData
    /// @param msg 导航消息，position.x/y/z = lat/lon/alt
    /// @param rtk [out] 转换后的 RTK 数据（仅当原点就绪且数据有效时写入）
    /// @return true 如果转换成功
    bool Convert(const nav_msgs::msg::Odometry& msg, RTKData& rtk);

    /// 从 georeference.yaml 加载地理原点，加载后跳过运行时中值滤波初始化
    /// @return true 如果文件存在且加载成功
    bool LoadOrigin(const std::string& geo_yaml_path);

    /// 原点是否已就绪
    bool IsOriginReady() const { return origin_.valid_; }

    /// 获取原点信息（用于保存 georeference.yaml）
    const MapOrigin& GetOrigin() const { return origin_; }

    /// 设置姿态噪声（从 YAML 读取后调用）
    void SetRotNoise(double noise) { options_.rot_noise = noise; }

    /// 设置 IMU→base_link 杆臂外参（从 YAML 读取后调用）
    /// @param T_imu_base IMU 坐标系到 base_link 坐标系的 SE3 变换（仅平移分量有效，旋转分量假设为 Identity）
    void SetTImuBase(const SE3& T_imu_base) { T_imu_base_ = T_imu_base; }

   private:
    Options options_;
    MapOrigin origin_;
    RTKData last_valid_rtk_;
    SE3 T_imu_base_ = SE3();  ///< IMU→base_link 杆臂外参（默认 Identity，向后兼容）

    // 中值滤波缓冲（原点未就绪时累积，就绪后不再使用）
    int origin_init_count_ = 0;
    std::vector<double> origin_lats_;
    std::vector<double> origin_lons_;
    std::vector<double> origin_alts_;
};

}  // namespace lightning
