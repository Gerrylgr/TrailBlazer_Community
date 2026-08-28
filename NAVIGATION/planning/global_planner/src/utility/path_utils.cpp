#include "global_planner/utility/path_utils.hpp"

namespace global_planner
{

    std::vector<double> compute_angles(const std::vector<Eigen::Vector2d> &points) {
        int n = points.size();
        std::vector<double> angles(n, M_PI);
        if (n < 3) return angles;

        for (int i = 1; i < n - 1; ++i) {
            // 表示两个相邻向量
            Eigen::Vector2d v1 = points[i] - points[i-1];
            Eigen::Vector2d v2 = points[i+1] - points[i];
            // 计算两个相邻向量的长度
            double n1 = v1.norm();
            double n2 = v2.norm();
            if (n1 < 1e-12 || n2 < 1e-12) { angles[i] = 0.0; continue; }
            // 计算夹角
            double cos_angle = v1.dot(v2) / (n1 * n2);
            cos_angle = std::clamp(cos_angle, -1.0, 1.0);
            angles[i] = acos(cos_angle);
        }
        return angles;
    }

    std::vector<Eigen::Vector2d> simplify_path(const std::vector<Eigen::Vector2d> &points, double corner_deg, int corner_dilate) {
        /*
        路径简化函数，输入：
            points: 路径点向量
            corner_deg: 简化路径的角度阈值
            corner_dilate: 在拐点处上采样的点数
        */
        int n = points.size();
        if (n <= 2) return points;

        std::vector<double> angles = compute_angles(points);
        double theta = corner_deg * M_PI / 180.0;

        std::vector<int> corner_idx;     // 用于保存幸存下来的点（特征点）
        for (int i = 0; i < n; ++i) {
            if (i == 0 || i == n-1 || angles[i] >= theta)
                corner_idx.push_back(i);
        }

        // 膨胀拐角索引
        if (corner_dilate > 0) {
            std::set<int> expanded;
            for (int idx : corner_idx) {
                for (int j = idx - corner_dilate; j <= idx + corner_dilate; ++j)
                    if (j >= 0 && j < n) expanded.insert(j);    // 不越界就加入
            }
            // 用膨胀后的点集替换掉原来的点集
            corner_idx.assign(expanded.begin(), expanded.end());
        }

        std::vector<Eigen::Vector2d> out;
        for (int idx : corner_idx)
            out.push_back(points[idx]);
        return out;
    }

}