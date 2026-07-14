//
// LLA ↔ UTM ↔ ENU 坐标转换工具
// 不依赖 geodesy 库，直接实现 WGS84 椭球下的 UTM 投影
//

#pragma once

#include <cmath>
#include <string>

namespace lightning {

/// UTM 坐标结构体
struct UTMCoordinate {
    double easting = 0.0;   ///< 东向坐标 (m)
    double northing = 0.0;  ///< 北向坐标 (m)
    double altitude = 0.0;  ///< 高程 (m)
    int zone = 0;           ///< UTM 带号 (1-60)
    char band = 'N';        ///< UTM 纬度带 ('C' - 'X')
};

/**
 * 将 WGS84 经纬高转换为 UTM 坐标
 * @param lat 纬度 (度, 北正)
 * @param lon 经度 (度, 东正)
 * @param alt 高程 (米)
 * @return UTM 坐标
 */
UTMCoordinate LLAtoUTM(double lat, double lon, double alt);

}  // namespace lightning
