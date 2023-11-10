#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include "iostream"

using namespace std;

float vehicle_z;
ros::Publisher pub;
ros::Subscriber sub;
geometry_msgs::PointStamped goal_point;
void odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    vehicle_z = odom->pose.pose.position.z;
    goal_point.point.x = 2;
    goal_point.point.y = 0;
    goal_point.point.z = -2;
    pub.publish(goal_point);
}


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"send_point");

    ros::NodeHandle nh;
    sub = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, &odomHandler);
    pub = nh.advertise<geometry_msgs::PointStamped> ("/goal_point",10);
    ros::Rate rate(10);    //定义一个频率
    while(ros::ok())
    {
        goal_point.header.frame_id = "map";
        goal_point.header.stamp = ros::Time::now();
        cout << "goal_point.x = " <<  goal_point.point.x << " goal_point.y =  " << goal_point.point.y << " goal_point.point.z = " << goal_point.point.z << endl;
        
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
