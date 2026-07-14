//
// LLA → UTM 转换实现
// 基于 WGS84 椭球参数，使用标准 UTM 投影公式
//

#include "common/rtk_utils.h"

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

}  // namespace lightning
