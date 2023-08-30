#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>
using namespace std;

double position_x ;
double position_y ;
double position_z ;
string imu_frame;

void ImuCallback(const sensor_msgs::ImuConstPtr& imu_data) {
    static tf::TransformBroadcaster br;//Broadcaster
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position_x, position_y, position_z));//Set the translation part
    //Get quaternion data from IMU message packet
    tf::Quaternion q;
    q.setX(imu_data->orientation.x);
    q.setY(imu_data->orientation.y);
    q.setZ(imu_data->orientation.z);
    q.setW(imu_data->orientation.w);
    q.normalized();//Normalization
    transform.setRotation(q);//Set the rotation part
    //broadcast out
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", imu_frame));
}
int main (int argc, char ** argv) {
    ros::init(argc, argv, "imu_data_to_tf");
    ros::NodeHandle node;
    node.param("/imu_tf/position_x", position_x, 0.0);
    node.param("/imu_tf/position_y", position_y, 0.0);
    node.param("/imu_tf/position_z", position_z, 0.0);
    node.param("imu", imu_frame, string("imu_link"));
    ros::Subscriber sub = node.subscribe("/imu_data", 10, &ImuCallback);
    ros::spin();
    return 0;
}
