#ifndef _LIDAR_POSE_GRAPH_H
#define _LIDAR_POSE_GRAPH_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Eigen>
#include "map2d.h"

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::AngleAxisRotatePoint;
using ceres::CrossProduct;
using ceres::DotProduct;

//parameters: rotation and translation
//rotation in angleAxis format

struct lidar_edge_error
{   
    /**
    * p待计算距离的点
    */
    lidar_edge_error(const Eigen::Vector2d p, const Eigen::Vector2d p1, const Eigen::Vector2d p2)
        : p(p), p1(p1), p2(p2) {}

    template <typename T>
    bool operator()(const T *const pose,
                    T *residuals) const
    {
        // pose[0] = theta
        T pi[2];
        pi[0] = T(p(0));
        pi[1] = T(p(1));

        T R[2][2];
        R[0][0] = cos(pose[0]); R[0][1] = -sin(pose[0]); 
        R[1][0] = sin(pose[0]); R[1][1] =  cos(pose[0]); 
        T pi_proj[2];//project pi to current frame

        pi_proj[0] = R[0][0] * pi[0] + R[0][1] * pi[1];
        pi_proj[1] = R[1][0] * pi[0] + R[1][1] * pi[1];
        // pose[1, 2] are the translation.
        pi_proj[0] += pose[1];
        pi_proj[1] += pose[2];

        //distance between pi_proj to line(p1, p2)
        T d1[2], d12[2];
        d1[0] = pi_proj[0] - T(p1(0));
        d1[1] = pi_proj[1] - T(p1(1));

        d12[0] = T(p1(0) - p2(0));
        d12[1] = T(p1(1) - p2(1));

        T normal[2];
        normal[0] = -d12[1];
        normal[1] = d12[0];

        T norm = sqrt(normal[0] * normal[0] + normal[1] * normal[1]);
        normal[0] /= norm;
        normal[1] /= norm;

        residuals[0] = d1[0] * normal[0] + d1[1] * normal[1];//dot product
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector2d p, const Eigen::Vector2d p1, const Eigen::Vector2d p2)
    {
        return (new ceres::AutoDiffCostFunction<lidar_edge_error, 1, 6>(
            new lidar_edge_error(p, p1, p2)));
    }


    //project point p to line (p1 - p2)
    Eigen::Vector2d p;
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
};

struct lidar_edge_error2
{   
    /**
    * p待计算距离的点
    */
    lidar_edge_error2(const Eigen::Vector2d p, const Eigen::Vector2d param)
        : p_(p), param_(param) {}

    template <typename T>
    bool operator()(const T *const pose,
                    T *residuals) const
    {
        // pose[0] = theta
        T pi[2];
        pi[0] = T(p_(0));
        pi[1] = T(p_(1));

        T R[2][2];
        R[0][0] = cos(pose[0]); R[0][1] = -sin(pose[0]); 
        R[1][0] = sin(pose[0]); R[1][1] =  cos(pose[0]); 
        T pi_proj[2];//project pi to current frame

        pi_proj[0] = R[0][0] * pi[0] + R[0][1] * pi[1];
        pi_proj[1] = R[1][0] * pi[0] + R[1][1] * pi[1];
        // pose[1, 2] are the translation.
        pi_proj[0] += pose[1];
        pi_proj[1] += pose[2];

        T norm = sqrt(T(param_(0))*T(param_(0))+T(param_(1))*T(param_(1)));
        residuals[0] = (T(param_(0))*pi_proj[0] + T(param_(1))*pi_proj[1] - T(1))/norm;//dot product
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector2d p, const Eigen::Vector2d param)
    {
        return (new ceres::AutoDiffCostFunction<lidar_edge_error2, 1, 4>(
            new lidar_edge_error2(p, param)));
    }


    //project point p to line (p1 - p2)
    Eigen::Vector2d p_;
    Eigen::Vector2d param_;
};

/**
* scan-to-map
*/
class OccupiedError : public ceres::CostFunction
{
public:
	// 此处，通过构造函数传入相关参数的维度
    OccupiedError(const Map2d::Ptr& map, const CloudType::Ptr& cloud, double theta) 
    : map_(map), cloud_(cloud), theta_(theta) {
        set_num_residuals(cloud->size());
        mutable_parameter_block_sizes()->push_back(3);
    }
    virtual ~OccupiedError() {}

    // 用户自定义残差计算方法
    virtual bool Evaluate(double const* const* pose, /*输入参数块*/\
        double* residuals, /*输出残差*/\
        double** jacobians /*输出雅克比矩阵*/) const
    {
        Eigen::Matrix2d R;
        R(0,0) = cos(pose[0][0]); R(0,1) = -sin(pose[0][0]); 
        R(1,0) = sin(pose[0][0]); R(1,1) =  cos(pose[0][0]);
        Eigen::Vector2d t;
        t(0,0) = pose[0][1];   t(1,0) = pose[0][2];
        double alpha = map_->resolution_;
        for(size_t i = 0; i < cloud_->size(); i++) {
            Eigen::Vector2d point(cloud_->points[i](0), cloud_->points[i](1));
            point = R * point + t;
            Eigen::Vector2d grad;
            residuals[i] = 1 - map_->bicubic_interpolation(point, grad);
            //梯度 [alpha*delta_x, alpha*delta_y, -alpha*delat_x*r*sin(theta)+alpha*delat_y*r*cos(theta)]
            // angle计算有点问题，应该是原始angle+delat_theta
            if (jacobians != NULL && jacobians[0] != NULL) {
                double angle = cloud_->originData.angles[i] + theta_;
                double r = cloud_->originData.dist[i];
                jacobians[0][i*3] = alpha * grad(0);
                jacobians[0][i*3+1] = alpha * grad(1);
                jacobians[0][i*3+2] = -alpha * grad(0) * r * sin(angle) + 
                                        alpha * grad(1) * r * cos(angle);
            }
        }
        return true;
    }

private:
    Map2d::Ptr map_;
    CloudType::Ptr cloud_;
    double theta_;
};

/**
* 避免优化到很远的地方去
*/
struct tranformationError
{   
    tranformationError(const double weight)
        : weight_(weight) {}

    template <typename T>
    bool operator()(const T *const pose,
                    T *residuals) const
    {
        residuals[0] = weight_*(pose[0])*(pose[0]);
        residuals[1] = weight_*(pose[1])*(pose[1]);
        residuals[1] = weight_*(pose[2])*(pose[2]);
        return true;
    }

    static ceres::CostFunction *Create(const double weight)
    {
        return (new ceres::AutoDiffCostFunction<tranformationError, 2, 3>(
            new tranformationError(weight)));
    }
private:
    double weight_;
};


#endif