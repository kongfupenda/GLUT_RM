#ifndef PYH_PATHFOLLOWER
#define PYH_PATHFOLLOWER

#include"ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;

class PYH_pathFollower
{
private:
    /* data */
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber odom_sub;
    ros::Subscriber path_sub;

    

public:
    ros::Publisher  cmd_vel_pub;                // 发布cmd_vel话题
    double vx,vy,speed;
    double Current_x, Current_y;
    float target_x, target_y;
    int path_size;
    int pathPointID;
    nav_msgs::Path path;
    bool path_callback;
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener;

    PYH_pathFollower();
    ~PYH_pathFollower();

    void odom_sub_Handler(const nav_msgs::Odometry::ConstPtr& odomIn);
    void path_sub_Handler(const nav_msgs::Path::ConstPtr& pathIn);




    
};


#endif