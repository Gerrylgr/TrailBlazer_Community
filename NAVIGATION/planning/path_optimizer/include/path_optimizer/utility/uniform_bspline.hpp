#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

namespace path_optimizer
{

class UniformBspline
{
public:
    UniformBspline() = default;

    UniformBspline(
        const Eigen::MatrixXd & points,
        const int & order,
        const double & interval);

    ~UniformBspline();

    // 原始路径点 -> 控制点
    static bool parameterizeToBspline(
        const double & ts,
        const std::vector<Eigen::Vector2d> & point_set,
        const std::vector<Eigen::Vector2d> & start_end_derivative,
        Eigen::MatrixXd & ctrl_pts);

    // 用 knot 参数 u 求曲线点
    Eigen::VectorXd evaluateDeBoor(const double & u);

    // 用时间参数 t 求曲线点（从“时间 t”映射到 knot 空间，改变 interval 不会改变曲线）
    /*
    这份代码的实现中 u_(3) 永远是 0，所以这里其实是“保留了兼容非零起点 knot 的能力”
    */
    inline Eigen::VectorXd evaluateDeBoorT(const double & t)
    {
        return evaluateDeBoor(t + u_(p_));
    }

    // 导数相关
    UniformBspline getDerivative();
    Eigen::MatrixXd getDerivativeControlPoints();

    // 获取有效时间范围
    bool getTimeSpan(double & um, double & um_p);

    // 设置 B 样条
    void setUniformBspline(
        const Eigen::MatrixXd & points,
        const int & order,
        const double & interval);

    void setKnot(const Eigen::VectorXd & knot)
    {
        u_ = knot;
    }

    // 便于直接采样成路径点
    std::vector<Eigen::Vector2d> sampleByParameter(double du);

    // 控制点变为曲线点，同时按照 spatial_resolution 重采样并输出
    std::vector<Eigen::Vector2d> sampleByArcLength(double spatial_resolution, double parameter_step = 0.02);
    static std::vector<Eigen::Vector2d> resamplePolylineByArcLength(
        const std::vector<Eigen::Vector2d> & input,
        double resolution);                   // 对路径弧长均匀重采样，得到的路径点之间间距相等

    // getter
    inline int getOrder() const { return p_; }
    inline double getInterval() const { return interval_; }
    inline const Eigen::MatrixXd & getControlPoints() const { return control_points_; }

private:
    int p_{0};          // degree
    int n_{-1};         // n+1 control points
    int m_{-1};         // m = n + p + 1
    Eigen::VectorXd u_; // knot vector
    double interval_{0.3};

    Eigen::MatrixXd control_points_;
};

} // namespace path_optimizer