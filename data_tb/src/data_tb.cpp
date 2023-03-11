#include "data_tb.h"
 #include <cmath>
 
struct EulerAngles {
    double roll, pitch, yaw;
};

subscriberANDpublisher::subscriberANDpublisher()
{
    
    //订阅话题
    UAVpose_sub.subscribe(nh, "/iris_0/mavros/local_position/pose", 10);
    lidar_sub.subscribe(nh, "/iris_0/scan", 10);
    
    // 发布者
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud>("/pointcloud", 10);
 
    //回调
    sync_.reset(new Sync(syncpolicy(10), UAVpose_sub, lidar_sub));
    sync_->registerCallback(boost::bind(&subscriberANDpublisher::callback, this, _1, _2));
    ROS_INFO("init done!");
}
sensor_msgs::PointCloud cloud;
pcl::PointCloud<pcl::PointXYZ> cloud1;
void subscriberANDpublisher::callback(const geometry_msgs::PoseStamped::ConstPtr &pt,  const sensor_msgs::LaserScan::ConstPtr &msg)   //数据处理
{
    // ROS_INFO("done! ");
    double yaw;
    geometry_msgs::PoseStamped UAVpose;
    UAVpose=*pt;
    cloud.header.frame_id = "map";
    cloud.header.stamp = ros::Time::now();
    // cloud.points.resize(msg->ranges.size());

    double siny_cosp = 2 * (UAVpose.pose.orientation.w * UAVpose.pose.orientation.z + UAVpose.pose.orientation.x * UAVpose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (UAVpose.pose.orientation.y * UAVpose.pose.orientation.y + UAVpose.pose.orientation.z * UAVpose.pose.orientation.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);    //四元数转换，计算偏航角
    // std::cout<<"yaw:"<<yaw<<std::endl;

    for (int i = 0; i < msg->ranges.size(); ++i)
    {
        geometry_msgs::Point32 point;
        point.x = UAVpose.pose.position.x+(cos(yaw+msg->angle_min+i*msg->angle_increment)*msg->ranges[i]);
        point.y = UAVpose.pose.position.y+(sin(yaw+msg->angle_min+i*msg->angle_increment)*msg->ranges[i]);
        point.z = UAVpose.pose.position.z;
        cloud.points.push_back(point);
    }
    pointcloud_pub.publish(cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    subscriberANDpublisher sp;
    ROS_INFO("main done! ");
    ros::spin();
 
}