#ifndef __SLAM2D_H
#define __SLAM2D_H
#include "slam2d_pose_graph.h"
#include "basic.h"
#include "map2d.h"
#include "kdtree.h"
#include <thread>
#include <execution>

using namespace std;
using namespace Eigen;

typedef struct
{
    double theta;
    Eigen::Vector2d t;
    Eigen::Matrix4f toMatrix4f() {
        Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
        T(0, 0) = cos(theta); T(0, 1) = -sin(theta);
        T(1, 0) = sin(theta); T(1, 1) =  cos(theta);
        T(0, 3) = t(0);       T(1, 3) = t(1);
        return T;
    };
    Eigen::Matrix3d toMatrix3d() {
        Eigen::Matrix3d T3(Eigen::Matrix3d::Identity());
        T3(0, 0) = cos(theta); T3(0, 1) = -sin(theta);
        T3(1, 0) = sin(theta); T3(1, 1) =  cos(theta);
        T3(0, 2) = t(0);       T3(1, 2) = t(1);
        return T3;
    };
    void setZero() {
        theta = 0;
        t.setZero();
    }
} State2d;

typedef struct {
    double s2sMatchT;
    double s2mMatchT;
    double updateMapT;
} TimeLog;

class slam2d
{
private:
    /* data */
public:

    slam2d(int width, int height, double resolution);
    ~slam2d();
    State2d state;
    State2d delta;
    double width_, height_, resolution_;
    double firstTimeStamp_, timestamp_, elapsed_time_;
    double mapScore_;
    
    CloudType::Ptr curr_scan_;
    CloudType::Ptr prev_scan_;

    Map2d::Ptr map2d_vec;

    std::thread debug_thread_;
    std::string cpu_type_;
    std::vector<double> cpu_percents;
    clock_t lastCPU, lastSysCPU, lastUserCPU;
    int numProcessors;
    std::string version_;
    TimeLog timeLog_;

    void setCurrentCloud(CloudType cloud);
    void scan_match();
    void s2sGaussNewton();
    void scan_map_interpolation();
    void scan_map_match_random();
    void scan_map_violent();
    void update();
    void update_transform();
    void update_map();
    void debug();
};

#endif
