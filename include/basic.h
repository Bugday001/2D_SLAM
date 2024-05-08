#ifndef BASIC_H_
#define BASIC_H_

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <iostream>
#include <sys/times.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointType;

class CloudType {
public:
    typedef std::shared_ptr<CloudType> Ptr;
    // 构造函数，初始化内部数组
    CloudType(){}
    CloudType(size_t size) : points(std::vector<Eigen::Vector2d>(size)) {}
    // 重载[]操作符，使其可以通过索引访问数组元素（非const版本）  
    Eigen::Vector2d& operator[](size_t index) {  
        return points.at(index); // 使用at()来检查索引是否有效  
    }  
    // 重载[]操作符的const版本，用于const对象  
    const Eigen::Vector2d& operator[](size_t index) const {  
        return points.at(index);
    }
    size_t size() const { return points.size(); }
    size_t size() { return points.size(); }
    void resize(size_t num) { points.resize(num); }
    std::vector<Eigen::Vector2d> points;
};

inline Eigen::Vector2d point2eigen(PointType p)
{
    Eigen::Vector2d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    return pp;
}

inline PointType eigen2point(Eigen::Vector2d pp)
{
    PointType p;
    p.x = pp(0);
    p.y = pp(1);
    return p;
}


/**
* pose: theta, x, y
*/
inline Eigen::Matrix3d pose2Matrix3d(const Eigen::Vector3d& pose) {
    Eigen::Matrix3d T(Eigen::Matrix3d::Identity());
    double theta = pose(0);
    T(0, 0) = cos(theta);   T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);   T(1, 1) =  cos(theta);
    T(0, 2) = pose(1);      T(1, 2) = pose(2);
    return T;
}

inline void vector2pcl(const CloudType::Ptr& scan, pcl::PointCloud<PointType>& scan_pcl) {
    int points_nums = scan->size();
    scan_pcl.points.resize(points_nums);
    for (auto i = 0; i < points_nums; i++) {
        scan_pcl.points[i] = eigen2point(scan->points[i]);
    }
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(scan_pcl, scan_pcl, indices);
    scan_pcl.width = scan_pcl.points.size();
    scan_pcl.height = 1;
    scan_pcl.is_dense = true;
}

/**
* CloudType类型点云变换
* 左乘
*/
inline void transfromCloudVec(const CloudType::Ptr& cloud, CloudType::Ptr& output, const Eigen::Matrix3d& T) {
    Eigen::Matrix2d R = T.block<2,2>(0,0);
    Eigen::Vector2d t = T.block<2,1>(0,2);
    for(size_t i=0; i<cloud->size(); i++) {
        output->points[i] = R * cloud->points[i] + t;  
    }
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

#endif