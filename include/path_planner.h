#pragma once
#include "math_lib.h"

class straight_line { //直线
public:
    Point p0, p1;

    //直接用两个控制点构造
    straight_line(const Point& _p0, const Point& _p1)
        : p0(_p0), p1(_p1)
    {}

    //插值计算
    Point getPoint(double t) const {
        Point point;
        point.x = p0.x + (p1.x - p0.x) * t;
        point.y = p0.y + (p1.y - p0.y) * t;
        return point;
    }

    //根据需要的采样点数生成离散点序列
    std::vector<Point> generateCurve(int segments) const {
        std::vector<Point> curve;
        curve.reserve(segments + 1);
        for (int i = 0; i <= segments; i++) {
            double t = static_cast<double>(i) / segments;
            curve.push_back(getPoint(t));
        }
        return curve;
    }
};

class quadratic_bezier { //二阶贝塞尔曲线
public:
    Point p0, p1, p2;

    //直接用三个控制点构造
    quadratic_bezier(const Point& _p0, const Point& _p1, const Point& _p2)
        : p0(_p0), p1(_p1), p2(_p2)
    {}

    //插值计算
    Point getPoint(double t) const {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        Point point;
        point.x = uu * p0.x + 2 * u * t * p1.x + tt * p2.x;
        point.y = uu * p0.y + 2 * u * t * p1.y + tt * p2.y;
        return point;
    }

    //根据需要的采样点数生成离散点序列
    std::vector<Point> generateCurve(int segments) const {
        std::vector<Point> curve;
        curve.reserve(segments + 1);
        for (int i = 0; i <= segments; i++) {
            double t = static_cast<double>(i) / segments;
            curve.push_back(getPoint(t));
        }
        return curve;
    }
};

class cubic_bezier { //三阶贝塞尔曲线
public:
    Point p0, p1, p2, p3;

    //直接用四个控制点构造
    cubic_bezier(const Point& _p0, const Point& _p1, const Point& _p2, const Point& _p3)
        : p0(_p0), p1(_p1), p2(_p2), p3(_p3)
    {}

    //插值计算
    Point getPoint(double t) const {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        Point point;
        point.x = uuu * p0.x 
                  + 3 * uu * t * p1.x 
                  + 3 * u * tt * p2.x 
                  + ttt * p3.x;
        point.y = uuu * p0.y 
                  + 3 * uu * t * p1.y 
                  + 3 * u * tt * p2.y 
                  + ttt * p3.y;
        return point;
    }

    //根据需要的采样点数生成离散点序列
    std::vector<Point> generateCurve(int segments) const {
        std::vector<Point> curve;
        curve.reserve(segments + 1);
        for (int i = 0; i <= segments; i++) {
            double t = static_cast<double>(i) / segments;
            curve.push_back(getPoint(t));
        }
        return curve;
    }
};

class quartic_bezier { //四阶贝塞尔曲线
public:
    Point p0, p1, p2, p3, p4;

    //直接用五个控制点构造
    quartic_bezier(const Point& _p0, const Point& _p1, const Point& _p2, 
                    const Point& _p3, const Point& _p4)
        : p0(_p0), p1(_p1), p2(_p2), p3(_p3), p4(_p4)
    {}

    //插值计算
    Point getPoint(double t) const {
        double u = 1 - t;
        double b0 = u * u * u * u;
        double b1 = 4 * u * u * u * t;
        double b2 = 6 * u * u * t * t;
        double b3 = 4 * u * t * t * t;
        double b4 = t * t * t * t;
        Point point;
        point.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x + b4 * p4.x;
        point.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y + b4 * p4.y;
        return point;
    }

    //根据给定采样点数生成离散点序列
    std::vector<Point> generateCurve(int segments) const {
        std::vector<Point> curve;
        curve.reserve(segments + 1);
        for (int i = 0; i <= segments; i++) {
            double t = static_cast<double>(i) / segments;
            curve.push_back(getPoint(t));
        }
        return curve;
    }
};

inline std::vector<Point> PathPlan(const std::vector<Point>& controlPoints, int segments) {
    std::vector<Point> curve;
    size_t count = controlPoints.size();
    if(count == 2){
        straight_line line(controlPoints[0], controlPoints[1]);
        curve = line.generateCurve(segments);
    } else if(count == 3){
        quadratic_bezier bezier(controlPoints[0], controlPoints[1], controlPoints[2]);
        curve = bezier.generateCurve(segments);
    } else if(count == 4){
        cubic_bezier bezier(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]);
        curve = bezier.generateCurve(segments);
    } else if(count == 5){
        quartic_bezier bezier(controlPoints[0], controlPoints[1], controlPoints[2],
                               controlPoints[3], controlPoints[4]);
        curve = bezier.generateCurve(segments);
    } else { //不支持的控制点数量
        std::cerr << "Error: Unsupported number of control points." << std::endl;
        vex::thread::interruptAll();
    }
    return curve;
}