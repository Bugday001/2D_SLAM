#include "slam2d.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_broadcaster.h"
#include <Eigen/Eigen>

std::shared_ptr<slam2d> slam;
ros::Publisher pub_pose, pub_path, map_pub;
ros::Publisher pub_laserscan;
ros::Publisher pub_map2d;
nav_msgs::OccupancyGrid map2d;
void publish_pose();
void publish_map2d();

void readin_scan_data(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    CloudType cloud;
    slam->timestamp_ = msg->header.stamp.toSec();
    cloud.resize(msg->ranges.size());
    int valid_cnt = 0;
    double range_max = msg->range_max, range_min = std::max((double)msg->range_min, 1.0);
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        float dist = msg->ranges[i].echoes[0]; //only first echo used for slam2d
        if (!std::isfinite(dist) || dist>=range_max || dist<=range_min) {
            continue;
        }
        float theta = msg->angle_min + i * msg->angle_increment;
        cloud[valid_cnt](0) = dist * cos(theta);
        cloud[valid_cnt](1) = dist * sin(theta);
        valid_cnt++;
    }
    cloud.resize(valid_cnt);
    slam->setCurrentCloud(cloud);
}
void readin_scan_data(const sensor_msgs::LaserScanConstPtr &msg)
{
    CloudType cloud;
    slam->timestamp_ = msg->header.stamp.toSec();
    cloud.resize(msg->ranges.size());
    int valid_cnt = 0;
    double range_max = msg->range_max, range_min = std::max((double)msg->range_min, 1.0);
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {   
        float dist = msg->ranges[i]; //only first echo used for slam2d
        if (!std::isfinite(dist) || dist>=range_max || dist<=range_min) {
            continue;
        }
        float theta = msg->angle_min + i * msg->angle_increment;
        cloud[valid_cnt](0) = dist * cos(theta);
        cloud[valid_cnt](1) = dist * sin(theta);
        valid_cnt++;
    }
    cloud.resize(valid_cnt);
    slam->setCurrentCloud(cloud);
}


void multiecho2laserscan(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    //publish laserscan
    sensor_msgs::LaserScan laserscan;
    laserscan.header.stamp = msg->header.stamp;
    laserscan.header.frame_id = "base_link";
    laserscan.angle_min = msg->angle_min;
    laserscan.angle_max = msg->angle_max;
    laserscan.angle_increment = msg->angle_increment;
    laserscan.time_increment = msg->time_increment;
    laserscan.scan_time = msg->scan_time;
    laserscan.range_min = msg->range_min;
    laserscan.range_max = msg->range_max;
    laserscan.ranges.resize(msg->ranges.size());
    laserscan.intensities.resize(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        laserscan.ranges[i] = msg->ranges[i].echoes[0];
        laserscan.intensities[i] = msg->intensities[i].echoes[0];
    }
    pub_laserscan.publish(laserscan);
}

void laserscan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    readin_scan_data(msg);
    slam->update();
    publish_pose();
    publish_map2d();
}

void multiecho_laserscan_callback(const sensor_msgs::MultiEchoLaserScanConstPtr &msg)
{
    readin_scan_data(msg);
    slam->update();
    publish_pose();
    publish_map2d();
    multiecho2laserscan(msg);
}

void publish_map2d()
{
    map2d.header.stamp = ros::Time(slam->timestamp_);
    for (auto index : slam->map2d_vec->cell2update_) {
        map2d.data[index] = slam->map2d_vec->toROSMap(index);
    }
    pub_map2d.publish(map2d);
}

void publish_pose()
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(slam->timestamp_);
    pose.header.frame_id = "odom";
    double theta = slam->state.theta;
    pose.pose.orientation.w = cos(0.5 * theta);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(0.5 * theta);
    pose.pose.position.x = slam->state.t(0);
    pose.pose.position.y = slam->state.t(1);
    pose.pose.position.z = 0;
    pub_pose.publish(pose);

    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    pub_path.publish(path);

    //send transfrom
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tr;
    tr.header.stamp = ros::Time(slam->timestamp_);
    tr.header.frame_id = "odom";
    tr.child_frame_id = "base_link";
    tr.transform.translation.x = slam->state.t(0);
    tr.transform.translation.y = slam->state.t(1);
    tr.transform.translation.z = 0;
    tr.transform.rotation.x = pose.pose.orientation.x;
    tr.transform.rotation.y = pose.pose.orientation.y;
    tr.transform.rotation.z = pose.pose.orientation.z;
    tr.transform.rotation.w = pose.pose.orientation.w;
    br.sendTransform(tr);
}

void initParams() {
    int width, height;
    double resolution;
    ros::param::param<int>("~/map/width", width, 2000);
    ros::param::param<int>("~/map/height", height, 2000);
    ros::param::param<double>("~/map/resolution", resolution, 0.1);
    map2d.header.frame_id = "odom";
    map2d.info.width = width;
    map2d.info.height = height;
    map2d.info.resolution = resolution;
    map2d.info.origin.orientation.w = 1;
    map2d.info.origin.orientation.x = 0;
    map2d.info.origin.orientation.y = 0;
    map2d.info.origin.orientation.z = 0;
    map2d.info.origin.position.x = -0.5 * map2d.info.width * map2d.info.resolution;
    map2d.info.origin.position.y = -0.5 * map2d.info.height * map2d.info.resolution;
    map2d.info.origin.position.z = 0;
    map2d.data.resize(width * height, -1);

    slam = std::make_shared<slam2d>(width, height, resolution);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam2d");
    ros::NodeHandle nh;
    initParams();
    ros::Subscriber sub_multiecho_laserscan = nh.subscribe<sensor_msgs::MultiEchoLaserScan>("/multiecho_scan", 100, multiecho_laserscan_callback);
    ros::Subscriber sub_laserscan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserscan_callback);
    map_pub = nh.advertise<sensor_msgs::PointCloud>("/mapCloud", 50);
    pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/laserscan", 100);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/est_pose", 100);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 100);
    pub_map2d = nh.advertise<nav_msgs::OccupancyGrid>("/map", 100);
    ros::spin();
    return 0;
}