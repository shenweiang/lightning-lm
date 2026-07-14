//
// RTK 工具函数单元测试
// 编译方式（无 ROS 依赖，可直接用 g++ 编译）：
//   cd src/lightning-lm
//   g++ -std=c++17 -I. tests/rtk_utils_test.cc src/common/rtk_utils.cc -o /tmp/rtk_utils_test && /tmp/rtk_utils_test
//

#include "common/rtk_utils.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace lightning;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name)                                         \
    do {                                                   \
        printf("  [TEST] %s ... ", name);                  \
    } while (0)

#define PASS()                                             \
    do {                                                   \
        printf("PASSED\n");                                \
        tests_passed++;                                    \
    } while (0)

#define TO_STRING(x) std::string(x)
#define FAIL(msg)                                          \
    do {                                                   \
        printf("FAILED: %s\n", TO_STRING(msg).c_str());    \
        tests_failed++;                                    \
    } while (0)

#define ASSERT_NEAR(val, expected, tol, name)              \
    do {                                                   \
        if (std::fabs((val) - (expected)) > (tol)) {       \
            FAIL(name " 期望 " #expected " 实际 " +        \
                 std::to_string(val));                      \
            return;                                        \
        }                                                  \
    } while (0)

#define ASSERT_TRUE(cond, msg)                             \
    do {                                                   \
        if (!(cond)) {                                     \
            FAIL(msg);                                     \
            return;                                        \
        }                                                  \
    } while (0)

// ====== LLAtoUTM 正确性测试 ======

/// 测试 1：北半球 UTM 转换正确性
/// 上海 (31.23°N, 121.47°E) 预期 zone=51, easting≈354000, northing≈3456000
static void test_lla_to_utm_north() {
    TEST("LLAtoUTM 北半球(上海)");
    UTMCoordinate utm = LLAtoUTM(31.23, 121.47, 10.0);

    ASSERT_TRUE(utm.zone == 51,
                ("zone 期望 51 实际 " + std::to_string(utm.zone)).c_str());
    ASSERT_TRUE(utm.band == 'N',
                ("band 期望 N 实际 " + std::string(1, utm.band)).c_str());
    ASSERT_NEAR(utm.easting, 354000.0, 500.0, "easting");
    ASSERT_NEAR(utm.northing, 3456000.0, 500.0, "northing");
    ASSERT_NEAR(utm.altitude, 10.0, 1e-9, "altitude");
    PASS();
}

/// 测试 2：南半球 UTM 转换（northing 应加 10,000,000）
/// 悉尼 (-33.87°S, 151.21°E)
static void test_lla_to_utm_south() {
    TEST("LLAtoUTM 南半球(悉尼)");
    UTMCoordinate utm = LLAtoUTM(-33.87, 151.21, 0.0);

    ASSERT_TRUE(utm.zone == 56,
                ("zone 期望 56 实际 " + std::to_string(utm.zone)).c_str());
    ASSERT_TRUE(utm.band == 'N',
                ("band 期望 N 实际 " + std::string(1, utm.band)).c_str());
    // 南半球 northing 应加 10,000,000
    ASSERT_TRUE(utm.northing > 5000000.0,
                ("南半球 northing 应 > 5e6, 实际 " + std::to_string(utm.northing)).c_str());
    PASS();
}

/// 测试 3：赤道附近
static void test_lla_to_utm_equator() {
    TEST("LLAtoUTM 赤道(0°, 15°E)");
    UTMCoordinate utm = LLAtoUTM(0.0, 15.0, 0.0);

    ASSERT_TRUE(utm.zone == 33,
                ("zone 期望 33 实际 " + std::to_string(utm.zone)).c_str());
    // 赤道 northing 应为 ≈0（北半球不补偿）
    ASSERT_NEAR(utm.northing, 0.0, 100.0, "northing 赤道");
    PASS();
}

/// 测试 4：边界值 — 接近日期变更线（179°E）
static void test_lla_to_utm_dateline() {
    TEST("LLAtoUTM 日期变更线(0°, 179°E)");
    UTMCoordinate utm = LLAtoUTM(0.0, 179.0, 0.0);
    ASSERT_TRUE(utm.zone == 60,
                ("日期变更线 zone 异常: " + std::to_string(utm.zone)).c_str());
    PASS();
}

/// 测试 5：高纬度地区
static void test_lla_to_utm_high_lat() {
    TEST("LLAtoUTM 高纬度(80°N, 0°E)");
    UTMCoordinate utm = LLAtoUTM(80.0, 0.0, 0.0);

    ASSERT_TRUE(utm.zone == 31,
                ("zone 期望 31 实际 " + std::to_string(utm.zone)).c_str());
    // 高纬度 UTM 仍然有效（虽然 UPS 更合适）
    ASSERT_TRUE(std::isfinite(utm.easting),
                "高纬度 easting 应为有限值");
    ASSERT_TRUE(std::isfinite(utm.northing),
                "高纬度 northing 应为有限值");
    PASS();
}

/// 测试 6：输入 NaN 应产生 NaN 输出（不崩溃）
static void test_lla_to_utm_nan_input() {
    TEST("LLAtoUTM NaN 输入不崩溃");
    // 仅验证不崩溃
    volatile double nan_val = std::nan("");
    UTMCoordinate utm = LLAtoUTM(nan_val, 121.0, 0.0);
    ASSERT_TRUE(std::isnan(utm.easting),
                "NaN lat 输入应产生 NaN easting");
    PASS();
}

int main() {
    printf("=== RTK Utils 单元测试 ===\n\n");

    test_lla_to_utm_north();
    test_lla_to_utm_south();
    test_lla_to_utm_equator();
    test_lla_to_utm_dateline();
    test_lla_to_utm_high_lat();
    test_lla_to_utm_nan_input();

    printf("\n=== 结果: %d 通过, %d 失败 ===\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
