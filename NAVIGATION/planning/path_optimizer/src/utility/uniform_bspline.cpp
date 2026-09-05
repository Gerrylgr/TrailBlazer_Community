/*
均匀 B 样条实现；
E.g.
    控制点数：n+1 = 6  （即 n = 5）
    阶数：p = 3（3次B样条）
    interval = 1.0

    m = n + p + 1 = 5 + 3 + 1 = 9（节点向量最大索引）

    节点向量 u_ 长度 = m+1 = 10

    计算 knot 向量 u_：
        u_ = [-3, -2, -1, 0, 1, 2, 3, 4, 5, 6]

    则根据 getTimeSpan 函数，有效参数区间为 u ∈ [0, 3]，因为前边/后边的三个不足以生成样条点（p+1 个控制点生成一个点）

    对于生成曲线点的过程，举一个具体采样例子，dt = 0.5，sample 会生成这些 t：t = 0 t = 0.5 t = 1.0 t = 1.5 t = 2.0 t = 2.5 t = 3.0
    对应传入 evaluateDeBoor 的 u： u = t + u_(p) = t + 0 → u = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

    之后看每个 u 对应哪个区间，比如 u = 0，u_3 = 0 ≤ u < u_4 = 1 → k = 3，则用控制点：P0, P1, P2, P3

*/

#include "path_optimizer/utility/uniform_bspline.hpp"

#include <cmath>
#include <stdexcept>
#include <iostream>

namespace path_optimizer
{

UniformBspline::UniformBspline(
    const Eigen::MatrixXd & points,
    const int & order,
    const double & interval)
{
    setUniformBspline(points, order, interval);
}

UniformBspline::~UniformBspline() {}


void UniformBspline::setUniformBspline(
    const Eigen::MatrixXd & points,
    const int & order,
    const double & interval)
{
    control_points_ = points;       // 控制点矩阵
    p_ = order;             // 阶数，通常是 3（三次 B样条）
    interval_ = interval;           // 节点间隔 (ts）

    n_ = static_cast<int>(points.cols()) - 1;           // 控制点最大索引
    m_ = n_ + p_ + 1;               // 节点向量最大索引，满足 B样条数学定理：m=n+p+1

    u_ = Eigen::VectorXd::Zero(m_ + 1);         // 节点向量（knot）

    for (int i = 0; i <= m_; ++i)
    {
        // B 样条每个点的计算需要 p+1 个控制点参与，因此要在开头和结尾多留出 p 个节点
        if (i <= p_)
        {
            u_(i) = static_cast<double>(-p_ + i) * interval_;
        }
        else if (i > p_ && i <= m_ - p_)
        {
            u_(i) = u_(i - 1) + interval_;
        }
        else
        {
            u_(i) = u_(i - 1) + interval_;
        }
    }
}

/*
Esp. 整个路径由一条B样条曲线表示，但是一条B样条曲线由很多个多项式曲线段拼接而成（这里是3次多项式，片段之间在节点处平滑连接）

B 样条中，基函数的计算结果代表了控制点的“影响力”权重；
E.g. 3次 B 样条中一个点由4个控制点决定，
     即 Ci​(s) = B0​(s)*Pi ​+ B1​(s)*Pi+1 ​+ B2​(s)*Pi+2 ​+ B3​(s)*Pi+3​    其中 s ∈ [0,1]

    三次均匀 B 样条的四个基函数为：
        B0​(s) = (1−s)^3 / 6
        B1​(s) = (3s^3−6s^2+4) / 6
        B2​(s) = (−3s^3+3s^2+3s+1) / 6
        B3(s) = s^3 / 6
    取段首（因为每个路径点 Qi 都被看作某一小段 B 样条曲线的段起点），也就是 s=0，则：
	​   B0(0) = 1 / 6
       B1(0)= 4 / 6
       B2(0) = 1 / 6
       B3(0) = 0
    所以：
        Ci​(0) = 1/6*​Pi ​+ 4/6*​Pi+1 ​+ 1/6*​Pi+2​ + 0*Pi+3​
    也就是：
        Qi ​= (Pi ​+ 4Pi+1 ​+ Pi+2) / 6
        
    同样的，对上边的式子求一阶导/二阶导，并代入 s=0 可以得到
        B0′​(0) = −1/2​     B1′​(0) = 0     B2′​(0) = 1/2​      B3′​(0) = 0
        B0′′​(0) = 1       B1′′​(0) = −2​   B2′′​(0) = 1       B3′′​(0) = 0​
​​​
*/
// 使用上边的公式根据轨迹点/起始终点速度/起始终点加速度构建线性方程，利用最小二乘法计算控制点
/*
设输入点数量为 K：
    控制点数量是 K+2
    三次 B 样条有效段数是 ((K+2)-3 = K-1
    K 个输入点刚好对应这 K-1 段的 K 个边界
    再加起点、终点两条速度约束，正好得到 K+2 条约束，对应 K+2 个控制点
*/
/*
为什么选择 s=0：
首先，K 个中点需要对应 K 个样条段，当前 k+2 个控制点只能生成 k-1 个样条段，因此需要 k+3 个控制点，
    然后现在约束是 K 个位置约束+2 个速度约束=K+2，也就是说还需要加上一个约束才是 k+3。
然后就是约束的速度语义对不上，现在的速度约束是起点/终点的速度，而非每一段中点的速度。
最后是改变s可能导致解出的控制点振荡，详细解释见 image/why_s=0.png、why_s=0_1.png、why_s=0_1.png
*/
bool UniformBspline::parameterizeToBspline(
    const double & ts,
    const std::vector<Eigen::Vector2d> & point_set,
    const std::vector<Eigen::Vector2d> & start_end_derivative,
    Eigen::MatrixXd & ctrl_pts)
{
    if (ts <= 0.0)
    {
        std::cerr << "[UniformBspline] time step error." << std::endl;
        return false;
    }

    if (point_set.size() <= 3)
    {
        std::cerr << "[UniformBspline] point set has only "
                  << point_set.size() << " points." << std::endl;
        return false;
    }

    if (start_end_derivative.size() != 2)
    {
        std::cerr << "[UniformBspline] derivatives error." << std::endl;
        return false;
    }

    const int K = static_cast<int>(point_set.size());

    Eigen::Vector3d prow, vrow, arow;
    prow << 1.0, 4.0, 1.0;          // 位置模板（每个粗路径点是对应 3 个控制点加权平均）
    vrow << -1.0, 0.0, 1.0;         // 速度模板
    arow << 1.0, -2.0, 1.0;         // 加速度模板

    // K + 2：控制点数量
    // K + 4：约束数量，K 个位置约束，加上起点终点各两个速度约束、加速度约束
    // Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 2, K + 2);

    for (int i = 0; i < K; ++i)
    {
        // A.block(i, i, 1, 3)：在矩阵 A 的第 i 行、第 i 列开始，取一个 1×3 的小块
        A.block(i, i, 1, 3) = (1.0 / 6.0) * prow.transpose();
    }

    // (P(i+1) - P(i-1)) / 2*ts = v     （一阶差分）
    A.block(K, 0, 1, 3) = (1.0 / (2.0 * ts)) * vrow.transpose();            // 起点速度约束
    A.block(K + 1, K - 1, 1, 3) = (1.0 / (2.0 * ts)) * vrow.transpose();             // 终点速度约束

    // // (Pi - 2*P(i+1) + P(i+2)) / ts^2 = a      （二阶差分）
    // A.block(K + 2, 0, 1, 3) = (1.0 / (ts * ts)) * arow.transpose();         // 起点加速度约束
    // A.block(K + 3, K - 1, 1, 3) = (1.0 / (ts * ts)) * arow.transpose();         // 终点加速度约束

    // Eigen::VectorXd bx(K + 4), by(K + 4);       // x, y 坐标单独算
    Eigen::VectorXd bx = Eigen::VectorXd::Zero(K + 2);
    Eigen::VectorXd by = Eigen::VectorXd::Zero(K + 2);

    for (int i = 0; i < K; ++i)
    {
        // 填充轨迹点坐标
        bx(i) = point_set[i].x();  
        by(i) = point_set[i].y(); 
    }

    // for (int i = 0; i < 4; ++i)
    // {
    //     // 填充起点/终点的速度/加速度约束
    //     bx(K + i) = start_end_derivative[i].x(); 
    //     by(K + i) = start_end_derivative[i].y(); 
    // }

    for (int i = 0; i < 2; ++i)
    {
        // 填充起点/终点的速度约束
        bx(K + i) = start_end_derivative[i].x(); 
        by(K + i) = start_end_derivative[i].y(); 
    }

    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);

    ctrl_pts.resize(2, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();

    return true;
}

// 给定一个参数 u，计算 B 样条曲线在这个参数位置上的点
/*
先判断 u 落在哪个 knot 区间
再用这个区间对应的 p+1 个控制点
通过 de Boor 递推
算出一个曲线点
*/
/*
E.g.
    控制点数 = 6
    控制点索引 = P0, P1, P2, P3, P4, P5

    p_ = 3
    interval_ = 1
    u_ = [-3,-2,-1,0,1,2,3,4,5,6]

    则 n_ = points.cols() - 1 = 5
       m_ = n_ + p_ + 1 = 9
       节点向量长度是：m+1 = 10
            也就是：
                u_(0)=-3 u_(1)=-2 u_(2)=-1 u_(3)=0 u_(4)=1 u_(5)=2 u_(6)=3 u_(7)=4 u_(8)=5 u_(9)=6

    有效参数范围是： [u3​,u6​] = [0,3]
    u=1.25 本来就在 [0,3] 内，则 ub=1.25；
    然后找出 ub 落在哪个区间，2 > 1.25，所以 k = 4（u_(k + 1) = u_(5) = 2）
    之后找到能计算出这个轨迹点对应的 (p_ + 1) 个（这里是4个）控制点：
        控制点索引是：k−p+i = 4−3+i = 1+i
        也就是当 i = 0,1,2,3 时，分别取：
            d[0] = P1
            d[1] = P2
            d[2] = P3
            d[3] = P4
    知道了控制点，之后用递推公式计算出轨迹点的坐标
*/
Eigen::VectorXd UniformBspline::evaluateDeBoor(const double & u)
{
    // 检查有没有控制点
    if (control_points_.cols() == 0)
    {
        throw std::runtime_error("UniformBspline::evaluateDeBoor: empty control points.");
    }

    // B 样条真正有效的参数范围是：[up​, um−p​]
    double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

    int k = p_;
    while (true)
    {
        // 找到 u 落在哪个 knot 区间（uk ​≤ ub ≤ uk+1）
        // 用于确定当前这个参数点，要由哪 p+1 个控制点来参与计算
        if (u_(k + 1) >= ub)
            break;
        ++k;
    }

    std::vector<Eigen::VectorXd> d;
    d.reserve(p_ + 1);                  // d[0]=P0、d[1]=P1……

    // 先把当前相关的 p+1 个控制点取出来
    for (int i = 0; i <= p_; ++i)
    {
        d.push_back(control_points_.col(k - p_ + i));
    }

    // de Boor 递推
    for (int r = 1; r <= p_; ++r)
    {
        for (int i = p_; i >= r; --i)
        {
            double denom = u_(i + 1 + k - r) - u_(i + k - p_);
            double alpha = 0.0;
            if (std::abs(denom) > 1e-12)
            {
                alpha = (ub - u_(i + k - p_)) / denom;      // alpha 是参数 u 在这个窗口内的相对位置（0 = 取左点，1 = 取右点），就是上边的"s"
            }

            d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
        }
    }

    return d[p_];
}

// 返回B样条真正“有意义、可用”的参数区间，就是去除前后的 p 个多余节点区间
bool UniformBspline::getTimeSpan(double & um, double & um_p)
{
    if (p_ >= u_.rows() || m_ - p_ >= u_.rows())
        return false;

    um = u_(p_);
    um_p = u_(m_ - p_);
    return true;
}

// 计算原 B 样条曲线的一阶导数（速度曲线）对应的“控制点”
Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
{
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);

    for (int i = 0; i < ctp.cols(); ++i)
    {
        ctp.col(i) =
            p_ * (control_points_.col(i + 1) - control_points_.col(i)) /
            (u_(i + p_ + 1) - u_(i + 1));
    }

    return ctp;
}

// 构造原 B 样条曲线的一阶导数对应的 B 样条对象（也就是速度曲线）
UniformBspline UniformBspline::getDerivative()
{
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);        // 裁掉 knot 两端”
    derivative.setKnot(knot);

    return derivative;
}

// 已知控制点计算整个B样条曲线的所有点（这里的 dt 越小，sample() 生成的轨迹点就越多，看起来就越“细腻”）
std::vector<Eigen::Vector2d> UniformBspline::sampleByParameter(double du)
{
    std::vector<Eigen::Vector2d> sampled_pts;

    if (du <= 0.0)
        return sampled_pts;

    double tm, tmp;
    if (!getTimeSpan(tm, tmp))
        return sampled_pts;

    for (double t = 0.0; t < (tmp - tm); t += du)
    {
        Eigen::VectorXd pt = evaluateDeBoorT(t);
        sampled_pts.emplace_back(pt(0), pt(1));
    }

    Eigen::VectorXd pt = evaluateDeBoorT(tmp - tm);
    sampled_pts.emplace_back(pt(0), pt(1));

    return sampled_pts;
}

std::vector<Eigen::Vector2d> UniformBspline::sampleByArcLength(double spatial_resolution, double parameter_step)
{
    if (spatial_resolution <= 0.0 || parameter_step <= 0.0)
    {
        return {};
    }

    // 先按参数密集采样，将B样条近似成稠密折线
    const std::vector<Eigen::Vector2d> dense_points =sampleByParameter(parameter_step);

    // 再对稠密折线按真实空间弧长采样
    return resamplePolylineByArcLength(dense_points,spatial_resolution);
}

std::vector<Eigen::Vector2d> UniformBspline::resamplePolylineByArcLength(
    const std::vector<Eigen::Vector2d> & input,
    double resolution)
{
    std::vector<Eigen::Vector2d> output;

    if (input.empty() || resolution <= 0.0)
    {
        return output;
    }

    if (input.size() == 1)
    {
        output.push_back(input.front());
        return output;
    }

    std::vector<Eigen::Vector2d> cleaned;
    cleaned.reserve(input.size());
    cleaned.push_back(input.front());

    for (std::size_t i = 1; i < input.size(); ++i)
    {
        if ((input[i] - cleaned.back()).norm() > 1e-8)      // 只有差距足够大才放入，去除重复点
        {
            cleaned.push_back(input[i]);
        }
    }

    if (cleaned.size() == 1)
    {
        return cleaned;
    }

    std::vector<double> cumulative(cleaned.size(), 0.0);            // 存储的是累积长度

    for (std::size_t i = 1; i < cleaned.size(); ++i)
    {
        cumulative[i] = cumulative[i - 1] + (cleaned[i] - cleaned[i - 1]).norm();
    }

    const double total_length = cumulative.back();

    if (total_length <= 1e-8)
    {
        output.push_back(cleaned.front());
        return output;
    }

    std::size_t segment = 0;

    for (double target = 0.0; target < total_length; target += resolution)
    {
        // cumulative[segment + 1] < target 代表这一段的弧长小于 resolution，需要 ++segment 
        while (segment + 1 < cumulative.size() && cumulative[segment + 1] < target)
        {
            ++segment;
        }

        if (segment + 1 >= cleaned.size())
        {
            break;
        }

        const double segment_start = cumulative[segment];
        const double segment_end = cumulative[segment + 1];
        const double segment_length = segment_end - segment_start;

        if (segment_length <= 1e-8)
            continue;
        
        const double ratio = (target - segment_start) / segment_length;

        // P = P(start) ​+ ratio × (P(end​) − P(start)​) = (1 − ratio) × P(start​) + ratio × P(end)
        output.push_back((1.0 - ratio) * cleaned[segment] + ratio * cleaned[segment + 1]);
    }

    if (output.empty() || (output.back() - cleaned.back()).norm() > 1e-8)
    {
        output.push_back(cleaned.back());           // 推入最后一个点
    }   

    return output;
}

} // namespace path_optimizer