#ifndef __SUBANDPUB_H__
#define __SUBANDPUB_H__
//ros头文件
#include <ros/ros.h>
//时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
//传感器消息
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h> 
 // ros
#include <sensor_msgs/LaserScan.h>

// pcl_ros
#include <pcl_ros/point_cloud.h>    

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<pcl_conversions/pcl_conversions.h> 
class subscriberANDpublisher{
public:
    subscriberANDpublisher();
    void callback(const geometry_msgs::PoseStamped::ConstPtr &pt,  const sensor_msgs::LaserScan::ConstPtr &msg) ;
private:
    ros::NodeHandle nh;
    ros::Publisher pointcloud_pub;
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub;//雷达订阅
    message_filters::Subscriber<geometry_msgs::PoseStamped> UAVpose_sub;//无人机位置订阅
 
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::LaserScan> syncpolicy;//时间戳对齐规则
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;//时间同步器
 
};
#endif

