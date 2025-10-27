#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

const double pi = 3.141592653589793;
const double RAD_TO_DEG = 180.0 / pi;
const double DEG_TO_RAD = pi / 180.0;
const double RPM_TO_RADS = 0.10471975511965977;
const double RADS_TO_RPM = 9.549296585513721;

struct Point {
    double x{0};
    double y{0};

    double point_distance (const Point& p2) const noexcept {
        return std::sqrt(std::pow(x - p2.x, 2) + std::pow(y - p2.y, 2));
    }
};

struct Pose{
    double x{0};
    double y{0};
    double theta{0}; // rad
};

struct Vec2d {
    double x{0}, y{0};

    constexpr Vec2d() noexcept = default;
    constexpr Vec2d(double x_, double y_) noexcept : x(x_), y(y_) {}

    // 基本算术
    constexpr Vec2d operator+(const Vec2d& o) const noexcept { return {x + o.x, y + o.y}; }
    constexpr Vec2d operator-(const Vec2d& o) const noexcept { return {x - o.x, y - o.y}; }
    constexpr Vec2d operator-() const noexcept { return {-x, -y}; }
    constexpr Vec2d operator*(double s) const noexcept { return {x * s, y * s}; }
    friend constexpr Vec2d operator*(double s, const Vec2d& v) noexcept { return v * s; }
    constexpr Vec2d operator/(double s) const noexcept { return {x / s, y / s}; }

    Vec2d& operator+=(const Vec2d& o) noexcept { x += o.x; y += o.y; return *this; }
    Vec2d& operator-=(const Vec2d& o) noexcept { x -= o.x; y -= o.y; return *this; }
    Vec2d& operator*=(double s) noexcept { x *= s; y *= s; return *this; }
    Vec2d& operator/=(double s) noexcept { x /= s; y /= s; return *this; }

    // 长度，归一化，角度
    double length2() const noexcept { return x*x + y*y; }
    double length() const noexcept { return std::sqrt(length2()); }
    double angle() const noexcept { // 返回向量与 y 轴正方向的夹角（顺时针为正，度数 0-360）
        double deg = std::atan2(x, y) * RAD_TO_DEG;
        if (deg < 0.0) { deg += 360.0; }
        return deg;
    }

    // 返回归一化向量（若为零向量则返回原向量）
    Vec2d normalized() const noexcept {
        double len = length();
        if (len == 0.0) return *this;
        return { x / len, y / len };
    }

    // 点乘、伪叉乘（返回 z 分量大小）
    double dot(const Vec2d& o) const noexcept { return x * o.x + y * o.y; }
    double cross(const Vec2d& o) const noexcept { return x * o.y - y * o.x; }

    // 旋转（以原点为中心，angle 单位为弧度）
    Vec2d rotated(double angle) const noexcept {
        double c = std::cos(angle), s = std::sin(angle);
        return { x * c - y * s, x * s + y * c };
    }

    // 投影到其他向量
    Vec2d project_onto(const Vec2d& onto) const noexcept {
        double denom = onto.length2();
        if (denom == 0.0) return {0,0};
        double t = dot(onto) / denom;
        return onto * t;
    }

    /* 
    用法演示：

    Vec2d a { 3 , 4 };
    Vec2d b { 1 , 2 };

    functions:                        outputs:
    (a + b);                          Vec2d{4.0, 6.0}
    (a - b);                          Vec2d{2.0, 2.0}
    (a * 2.0);                        Vec2d{6.0, 8.0}
    (a / 2.0);                        Vec2d{1.5, 2.0}
    a.dot(b);                         11.0
    a.cross(b);                       2.0
    a.length();                       5.0
    a.normalized();                   Vec2d{0.6, 0.8}
    a.rotated(M_PI/2);                Vec2d{-4.0, 3.0}
    a.project_onto(b);                 Vec2d{2.2, 4.4}
    a.angle();                        53.1301

    */
};

inline double Sin(double deg){
    return sin((deg * pi) / 180.0);
}

inline double Cos(double deg){
    return cos((deg * pi) / 180.0);
}

inline double Tan(double deg){
    return tan((deg * pi) / 180.0);
}

inline double arctan(double tan_val){
    return atan(tan_val) * 180.0 / pi;
}

inline double arcsin(double sin_val){
    return asin(sin_val) * 180.0 / pi;
}

inline int sign(double input){
    return (input >= 0 ? (input == 0 ? 0 : 1) : -1);
}

inline double max_value_limit(double limit , double input){
    return (fabs(input) > limit ? sign(input) * limit : input);
}

inline double min_value_limit(double limit , double input){
    return (fabs(input) < limit ? sign(input) * limit : input);
}

inline double rpm_to_mm_s(double rpm, double R) {
    return rpm * 2.0 * R * pi / 60.0;
}

inline double dps_to_mm_s(double dps, double R) {
    return dps * R * pi / 180.0;
}