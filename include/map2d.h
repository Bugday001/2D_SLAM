#ifndef MAP2D_H_
#define MAP2D_H_

#include "basic.h"
#include <unordered_set>
class Map2d {
public:
    std::vector<double> odds;
    std::unordered_set<int> cell2update_;
    double width_, height_, resolution_;
    double p_hit, p_miss, lofree, looccu;
    typedef std::shared_ptr<Map2d> Ptr;

    Map2d(double width, double height, double resolution);
    void bresenham(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2);
    void update_map(const Eigen::Vector2i& origin, const CloudType::Ptr& scan_w);
    int scan_map_match_score(const CloudType::Ptr& scan ,const Eigen::Vector3d& pose);
    Eigen::Vector2d world2map_d(const Eigen::Vector2d& p);
    Eigen::Vector2i world2map_i(const Eigen::Vector2d& p);
    inline void addMiss(int x, int y) { odds[x*width_+y] *= lofree; }
    inline void addOccupied(int x, int y) { odds[x*width_+y] *= looccu; }
    inline double getOdds(int index) { return odds[index]; }
    inline bool isOccupied(int index) { return getOdds(index) > 1.0;};
    inline int toROSMap(int index) { return odds[index]==1?-1:(isOccupied(index)?100:0); };
    inline double getProbability(int index) { return odds[index]/(odds[index]+1.0); }
    inline void cubic_weight(float xin, float a, float& w, float& dw);
    double bicubic_interpolation(const Eigen::Vector2d& p, Eigen::Vector2d& grad);
    double bilinearInterpolation(const Eigen::Vector2d& p, Eigen::Vector2d& grad);
};

// TODO
// ~~膨胀地图，得到靠近边缘的简单梯度~~，手写双三次插值的梯度
// 实现kdtree，除去PCL库
// karto-slam的匹配方法加上二分查找，对角度二分
// Gicp
// ESDF

//Noted
// shared_ptr也是有引用传值的，不然是新的指针指向目标位置
#endif