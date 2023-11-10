// // #include"PYH_pathFollower.h"




// // PYH_pathFollower::PYH_pathFollower(): private_nh("~"),listener(buffer)
// // {
// //     vx = 0;
// //     vy = 0;
// //     speed = 0.5;
// //     Current_x = 0; 
// //     Current_y = 0;
// //     target_x = 0;
// //     target_y = 0;
// //     path_size = 0;
// //     path_callback = false;

// //     odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom",5,&PYH_pathFollower::odom_sub_Handler,this);

// //     path_sub = nh.subscribe<nav_msgs::Path> ("/move_base/DWAPlannerROS/global_plan",5,&PYH_pathFollower::path_sub_Handler,this);

// //     cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);

    
// // }
// // PYH_pathFollower::~PYH_pathFollower()
// // {

// // }

// // void PYH_pathFollower::odom_sub_Handler(const nav_msgs::Odometry::ConstPtr& odomIn) //要快很多
// // {
// //     Current_x = odomIn->pose.pose.position.x;
// //     Current_y = odomIn->pose.pose.position.y;
// //     // cout << "Current_x: " << Current_x << " Current_y: " << Current_y << endl;
// // }

// // void PYH_pathFollower::path_sub_Handler(const nav_msgs::Path::ConstPtr& pathIn) //比较慢
// // {
// //     cout << "path_size = " << pathIn->poses.size() << endl;
// //     path_size = pathIn->poses.size();

// //     geometry_msgs::PoseStamped psAtSon2;
// //     float base_target_x = 0, base_target_y = 0;


// //     if(path_size >= 20)
// //     {
// //         std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

// //         psAtSon2 = buffer.transform(pathIn->poses[4],"base_link",ros::Duration(1));
// //         base_target_x = psAtSon2.pose.position.x;
// //         base_target_y = psAtSon2.pose.position.y;
// //         cout << "base_target_x = " << base_target_x << " base_target_y = " << base_target_y << endl; 
// //         // target_x = pathIn->poses[path_size - 1].pose.position.x;
// //         // target_y = pathIn->poses[path_size - 1].pose.position.y;
// //         target_x = base_target_x;
// //         target_y = base_target_y;



// //         std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
// //         std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
// //         std::cout << "\n转换数据格式用时: " << time_used.count() << " 秒。" << std::endl;
        
// //     }


// //     // cout << "target_x[1] = " << target_x << " target_y[1] = " << target_y << endl;
// // }




// // int main(int argc, char  *argv[])
// // {
// //     ros::init(argc,argv,"PYH_pathFollower");
// //     PYH_pathFollower path_follower;
// //     float hypotenuse = 0;
// //     float diff_x = 0;
// //     float diff_y = 0;
// //     geometry_msgs::Twist twist;
    
    
// //     ros::Rate hz(50);
// //     while (ros::ok)
// //     {

// //         ros::spinOnce();
// //         if(path_follower.target_x != 0 && path_follower.target_y != 0 && path_follower.path_size >= 20)
// //         {
// //             // diff_x = path_follower.target_x - path_follower.Current_x;
// //             // diff_y = path_follower.target_y - path_follower.Current_y;
// //             // hypotenuse = sqrt( pow(diff_x,2) + pow(diff_y,2));
// //             // path_follower.vy = path_follower.speed * (diff_y / hypotenuse);
// //             // path_follower.vx = path_follower.speed * (diff_x / hypotenuse);
            
// //             diff_x = path_follower.target_x;
// //             diff_y = path_follower.target_y;
// //             hypotenuse = sqrt( pow(diff_x,2) + pow(diff_y,2));
// //             path_follower.vy = path_follower.speed * (diff_y / hypotenuse);
// //             path_follower.vx = path_follower.speed * (diff_x / hypotenuse);

// //             twist.linear.x = path_follower.vx;
// //             twist.linear.y = path_follower.vy;
            
// //         }
// //         else{
// //             twist.linear.x = 0;
// //             twist.linear.y = 0;
// //         }

// //         cout << "twist.linear.x = " << twist.linear.x  << "   twist.linear.y = " << twist.linear.y << endl;
// //         path_follower.cmd_vel_pub.publish(twist);
// //         hz.sleep();
// //     }
    
// //     return 0;
// // }

//-------------------------------------------------------测试版：跑仿真使用--------------------------------------------------------------------
//
//  缺点，转换坐标系耗时太长（也不是很长），不知道实际上车效果如何   (想法：不用函数，自己写转换矩阵，速度会不会快一点)
//  订阅/odom和/path（两者的frame_id都是odom）得到小车位置和跟踪路径点，为了算出来以小车为坐标系下的跟踪路径点，然后根据运动学分配速度

#include"PYH_pathFollower.h"
//构造函数：用来初始化函数，和初始化话题
PYH_pathFollower::PYH_pathFollower(): private_nh("~"),listener(buffer)
{
    vx = 0;
    vy = 0;
    speed = 0.5;
    Current_x = 0; 
    Current_y = 0;
    target_x = 0;
    target_y = 0;
    path_size = 0;
    pathPointID = 100;  // > path_size
    path_callback = false;

    odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom",5,&PYH_pathFollower::odom_sub_Handler,this); //odom

    path_sub = nh.subscribe<nav_msgs::Path> ("/move_base/DWAPlannerROS/global_plan",5,&PYH_pathFollower::path_sub_Handler,this);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);
    
    // cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel",5);

}
PYH_pathFollower::~PYH_pathFollower()
{

}
//订阅里程计回调,目的：得到小车现在的位置，frame_id:odom
void PYH_pathFollower::odom_sub_Handler(const nav_msgs::Odometry::ConstPtr& odomIn) //要快很多
{
    Current_x = odomIn->pose.pose.position.x;
    Current_y = odomIn->pose.pose.position.y;
    // cout << "Current_x: " << Current_x << " Current_y: " << Current_y << endl;
}

//订阅路径回调，目的：获得路径          frame_id:odom
//技巧：由于路径发布的慢，每次获取全部路径存起来，然后跟踪，直到路径更新了才去跟踪新的
void PYH_pathFollower::path_sub_Handler(const nav_msgs::Path::ConstPtr& pathIn) //比较慢
{
    cout << "path_size = " << pathIn->poses.size() << endl;
    path_size = pathIn->poses.size();
    path.poses.resize(path_size);

    path.header.frame_id = pathIn->header.frame_id;
    path.header.stamp = pathIn->header.stamp;

    for (int i = 0; i < path_size; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    }

    pathPointID = 0;
    // cout << "target_x[1] = " << target_x << " target_y[1] = " << target_y << endl;
}




int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"PYH_pathFollower");
    PYH_pathFollower path_follower;
    float hypotenuse = 0;
    float diff_x = 0;
    float diff_y = 0;
    float lookAheadDis = 0.1;       //前视距离
    geometry_msgs::Twist twist;
    // geometry_msgs::TwistStamped twist;
    // twist.header.frame_id = "vehicle";
    
    geometry_msgs::PoseStamped psAtSon2;
    float base_target_x = 0, base_target_y = 0;


    ros::Rate hz(50);
    while (ros::ok)
    {

        ros::spinOnce();
        float disX, disY, dis;
        while (path_follower.pathPointID < path_follower.path_size - 1) {      //遍历
            disX = path_follower.path.poses[path_follower.pathPointID].pose.position.x - path_follower.Current_x;//以小车建立坐标系然后算出来路径点的坐标x(前提，小车坐标系和世界坐标系只是平移关系)
            disY = path_follower.path.poses[path_follower.pathPointID].pose.position.y - path_follower.Current_y;//坐标y
            dis = sqrt(disX * disX + disY * disY);              //算出来路径点距小车的距离
            if (dis < lookAheadDis) {         //根据前视距离选择目标点
            path_follower.pathPointID++;
            } else {
            path_follower.path.poses[path_follower.pathPointID].header.frame_id = path_follower.path.header.frame_id;
            break;
            }
        }

        cout << "path_follower.pathPointID = " << path_follower.pathPointID << endl;
        if(path_follower.pathPointID < path_follower.path_size - 1)
        {
            std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
            //此是将路径点从odom坐标系转到小车坐标系base_link，此函数和上面方法一样(好像不一样)，只是为了测试两者不同，才写出来玩玩
            psAtSon2 = path_follower.buffer.transform(path_follower.path.poses[path_follower.pathPointID],"base_link",ros::Duration(10));//vehicle base_link
            base_target_x = psAtSon2.pose.position.x;       //base_target_x，以小车为坐标系的目标点的x值
            base_target_y = psAtSon2.pose.position.y;

            std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
            std::cout << "\n转换坐标系用时: " << time_used.count() << " 秒。" << std::endl;


            diff_x = base_target_x;             //diff_x是中间参数，之前的代码有用到，更新后，懒的删掉了，当中间参数了，意思：x方向的差值
            diff_y = base_target_y;             //
            hypotenuse = sqrt( pow(diff_x,2) + pow(diff_y,2));              //算出来斜边hypotenuse，勾股定理
            path_follower.vy = path_follower.speed * (diff_y / hypotenuse);
            path_follower.vx = path_follower.speed * (diff_x / hypotenuse);

            twist.linear.x = path_follower.vx;
            twist.linear.y = path_follower.vy;

            cout << "path_follower.vy = " << path_follower.vy << endl;

            // twist.twist.linear.x = path_follower.vx;
            // twist.twist.linear.x = path_follower.vy;
            
        }
        else{
            // twist.twist.linear.x = 0;
            // twist.twist.linear.x = 0;
            twist.linear.x = 0;
            twist.linear.y = 0;

        }
        
        cout << "twist.linear.x = " << twist.linear.x  << "   twist.linear.y = " << twist.linear.y << endl;
        // cout << "twist.linear.x = " << twist.twist.linear.x  << "   twist.linear.y = " << twist.twist.linear.y << endl;
        path_follower.cmd_vel_pub.publish(twist);
        hz.sleep();
    }
    
    return 0;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------





//--------------------------------------------------------实际部署版：搭配ground_based_autonomy_basic使用-----------------------------------------------
//  订阅/state_estimation和/path（map,vehicle）得到小车位置和跟踪路径点，为了算出来以小车为坐标系下的跟踪路径点，然后根据运动学分配速度
//  由于/path的frame_id:vehicle，可以直接得到小车坐标系的路径跟踪点，可以直接分配速度
//
//  改进方法：还需要测试

// #include"PYH_pathFollower.h"
// #include <signal.h>
// PYH_pathFollower::PYH_pathFollower(): private_nh("~"),listener(buffer)
// {
//     vx = 0;
//     vy = 0;
//     speed = 0.5;
//     Current_x = 0; 
//     Current_y = 0;
//     target_x = 0;
//     target_y = 0;
//     path_size = 0;
//     pathPointID = 100;  // > path_size
//     path_callback = false;

//     odom_sub = nh.subscribe<nav_msgs::Odometry> ("/state_estimation",5,&PYH_pathFollower::odom_sub_Handler,this); //odom

//     path_sub = nh.subscribe<nav_msgs::Path> ("/path",5,&PYH_pathFollower::path_sub_Handler,this);

//     //cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);
    
//     cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel",5);

// }
// PYH_pathFollower::~PYH_pathFollower()
// {

// }

// void PYH_pathFollower::odom_sub_Handler(const nav_msgs::Odometry::ConstPtr& odomIn) //要快很多
// {
//     Current_x = odomIn->pose.pose.position.x;
//     Current_y = odomIn->pose.pose.position.y;
//     // cout << "Current_x: " << Current_x << " Current_y: " << Current_y << endl;
// }

// void PYH_pathFollower::path_sub_Handler(const nav_msgs::Path::ConstPtr& pathIn) //比较慢
// {
//     cout << "path_size = " << pathIn->poses.size() << endl;
//     path_size = pathIn->poses.size();
//     path.poses.resize(path_size);

//     path.header.frame_id = pathIn->header.frame_id;
//     path.header.stamp = pathIn->header.stamp;

//     for (int i = 0; i < path_size; i++) {
//     path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
//     path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
//     path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
//     }

//     pathPointID = 0;
//     // cout << "target_x[1] = " << target_x << " target_y[1] = " << target_y << endl;
// }






// int main(int argc, char  *argv[])
// {
//     ros::init(argc,argv,"PYH_pathFollower");
//     PYH_pathFollower path_follower;
//     float hypotenuse = 0;
//     float diff_x = 0;
//     float diff_y = 0;
//     float lookAheadDis = 0.2;       //前视距离
//     // geometry_msgs::Twist twist;
//     geometry_msgs::TwistStamped twist;
//     twist.header.frame_id = "vehicle";
    
//     geometry_msgs::PoseStamped psAtSon2;
//     float base_target_x = 0, base_target_y = 0;

    

//     ros::Rate hz(50);
//     while (ros::ok)
//     {

//         ros::spinOnce();
//         float disX, disY, dis;
//         while (path_follower.pathPointID < path_follower.path_size - 1) {      //遍历
//             disX = path_follower.path.poses[path_follower.pathPointID].pose.position.x;
//             disY = path_follower.path.poses[path_follower.pathPointID].pose.position.y;
//             dis = sqrt(disX * disX + disY * disY);
//             if (dis < lookAheadDis) {         //根据前视距离选择目标点
//             path_follower.pathPointID++;
//             } else {
//             path_follower.path.poses[path_follower.pathPointID].header.frame_id = path_follower.path.header.frame_id;
//             break;
//             }
//         }

//         cout << "path_follower.pathPointID = " << path_follower.pathPointID << endl;
//         if(path_follower.pathPointID < path_follower.path_size - 1)
//         {

//             base_target_x = path_follower.path.poses[path_follower.pathPointID].pose.position.x;
//             base_target_y = path_follower.path.poses[path_follower.pathPointID].pose.position.y;

//             diff_x = base_target_x;
//             diff_y = base_target_y;
//             hypotenuse = sqrt( pow(diff_x,2) + pow(diff_y,2));
//             path_follower.vy = path_follower.speed * (diff_y / hypotenuse);
//             path_follower.vx = path_follower.speed * (diff_x / hypotenuse);

//             // twist.linear.x = path_follower.vx;
//             // twist.linear.y = path_follower.vy;

//             twist.twist.linear.x = path_follower.vx;
//             twist.twist.linear.y = path_follower.vy;
            
//         }
//         else{
//             twist.twist.linear.x = 0;
//             twist.twist.linear.x = 0;
//             // twist.linear.x = 0;
//             // twist.linear.y = 0;
//             
//         }
        
//         // cout << "twist.linear.x = " << twist.linear.x  << "   twist.linear.y = " << twist.linear.y << endl;
//         cout << "twist.linear.x = " << twist.twist.linear.x  << "   twist.linear.y = " << twist.twist.linear.y << endl;
//         path_follower.cmd_vel_pub.publish(twist);
//         hz.sleep();
//     }
    
//     return 0;
// }

//---------------------------------------------------------------------------------------------------------------------------------------------------