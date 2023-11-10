#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

#define PLOTPATHSET 1

float angle = 0;  //直接旋转角度
int selectedGroupID = -1;   //paths下目标序号
float dirDiff_temp = 0;
float score_temp = 0;

string pathFolder;
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;  
const int groupNum = 7;    //7
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;  //将雷达转至车体中心
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!useTerrainAnalysis) {      //不使用分析模块的点云就执行
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (useTerrainAnalysis) {     //使用点云分析模块的点云就是执行
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud); ////fromROSMsg函数将laserCloud2中的ROS消息转换为PCL点云，并将结果存储在laserCloud中

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));  //以车体建立坐标系下的相对距离
      //intensity代表物体的反射率强度，但是在地面分割后，该值被定义为点云与分割地面的相对高度。
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {       //adjacentRange点云距离阈值，intensity点云强度可以用来判断坡度和障碍高度
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop); // 输入点云
    terrainDwzFilter.filter(*terrainCloudDwz);      // 输出滤波后的点云到terrainCloudDwz

    newTerrainCloud = true;
  }
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)   //手柄，没啥用
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;

  if (joySpeed > 0) {
    joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / PI;
    if (joy->axes[4] < 0) joyDir *= -1;
  }

  if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void boundaryHandler(const geometry_msgs::PolygonStamped::ConstPtr& boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  int boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (int i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::PointCloud2ConstPtr& addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  int addedObstaclesSize = addedObstacles->points.size();
  for (int i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::Bool::ConstPtr& checkObs)
{
  double checkObsTime = ros::Time::now().toSec();

  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";     //就是一条条路径的点云,用来做路径跟踪的路径的，

  FILE *filePtr = fopen(fileName.c_str(), "r");//打开文件fileName，.c_str()C风格字符串版本，"r"打开文件的模式:只读，fopen函数的返回值是一个FILE *类型的指针。如果文件成功打开，这个指针就指向该文件；如果文件打开失败，这个指针就是NULL
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;         //groupID是startpaths对应的id值

      endDirPathList[pathID] = atan2(endY, endX) * 180 / PI;    //角度制  [-180,180]  2.0 * atan2(endY, endX) * 180 / PI
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        printf ("\nError reading input files, exit.\n\n");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("costHeightThre", costHeightThre);
  nhPrivate.getParam("costScore", costScore);
  nhPrivate.getParam("useCost", useCost);
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("dirWeight", dirWeight);
  nhPrivate.getParam("dirThre", dirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("minPathScale", minPathScale);
  nhPrivate.getParam("pathScaleStep", pathScaleStep);
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
  nhPrivate.getParam("minPathRange", minPathRange);
  nhPrivate.getParam("pathRangeStep", pathRangeStep);
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nhPrivate.getParam("goalClearRange", goalClearRange);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/state_estimation", 5, odometryHandler);    //输入

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/registered_scan", 5, laserCloudHandler);   //输入

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/terrain_map", 5, terrainCloudHandler);   //输入

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);  //手柄

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);   //输入，路径点

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);     //没有 输入

  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5, boundaryHandler); //far_planner 输入

  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2> ("/added_obstacles", 5, addedObstaclesHandler);  //没有输入

  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool> ("/check_obstacle", 5, checkObstacleHandler); //没有输入

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);   //输出，选出来最佳路径
  nav_msgs::Path path;

  #if PLOTPATHSET == 1
  ros::Publisher pubFreePaths = nh.advertise<sensor_msgs::PointCloud2> ("/free_paths", 2);  //输出 可行使路径，黄色路径
  #endif

  //ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/stacked_scans", 2);

  printf ("\nReading path files.\n");

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());  //reset为智能指针重新指向新目标
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());  //reset为智能指针重新指向新目标
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());  //reset为智能指针重新指向新目标
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);                       //reset为智能指针重新指向新目标
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);   //pcl库VoxelGrid容器 设置体素大小

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
  readCorrespondences();

  printf ("\nInitialization complete.\n\n");

  //twoWayDrive = false; //方便测试，为了不和pathFollower冲突----------------------------------------------------------------------------------------------------------------

  ros::Rate rate(100);    //100
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloud || newTerrainCloud) { //进入的总判断，有点云才能进入
      if (newLaserCloud) {    //
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {    //newTerrainCloud重置
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {    
        float pointX1 = plannerCloud->points[i].x - vehicleX;   //以小车建立坐标系下的点云坐标（前提，小车坐标系和世界坐标系只是平移关系）
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;  //二维下坐标系旋转坐标系，小车是会运动的，这个就是算小车旋转后的，点云的坐标
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw; 
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {  //由于我的useTerrainAnalysis设置ture，所以minRelz和maxRelZ没啥用
          plannerCloudCrop->push_back(point);
        }
      }

      int boundaryCloudSize = boundaryCloud->points.size();       //far_planner输入的边界
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw    //二维坐标系下point经过旋转平移的坐标系变换到小车坐标系下的point
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw 
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {        
          plannerCloudCrop->push_back(point);
        }
      }

      int addedObstaclesSize = addedObstacles->points.size();               //自我添加边界，无用
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;        
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;     //pathRangeBySpeed is true 路径范围速度影响
      if (pathRange < minPathRange) pathRange = minPathRange;         //最小路径范围
      float relativeGoalDis = adjacentRange;

      if (autonomyMode) {     //autonomyMode 自主模式 true
        float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);  //转到小车坐标系下的目标点
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);    //frame_id:vehicle下 小车与目标点的距离
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;    //反正切函数（atan2会判断你的象限，通过输入的x,y的正负），范围[-180,180]算斜边角 单位：角度
        // cout << "joyDir = " << joyDir << endl;
        // if (!twoWayDrive) {           //我全向轮不需要这个    
        //   if (joyDir > 90.0) joyDir = 90.0;
        //   else if (joyDir < -90.0) joyDir = -90.0;
        // }
      }

      bool pathFound = false;
      float defPathScale = pathScale;     //路径尺度
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;    //pathScaleBySpeed if true  路径尺度由速度影响
      if (pathScale < minPathScale) pathScale = minPathScale;       //最小路径尺度

      while (pathScale >= minPathScale && pathRange >= minPathRange) {      
        for (int i = 0; i < 36 * pathNum; i++) {    //每次循环开始都初始化
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0); //算对角线长度 /2 应该减比例了
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;  //算斜边的倾斜角  单位：角度

//---------------------------------------------------对边界点云的处理，还没有看,应该能决定会不会上坡---------------------------------------------------------------
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);
          //dis 点云相对小车的距离
          //checkObstacle is ture，pathRange处理的点云范围，pathCropByGoal是根据目标点距离+goalClearRange选取点云，true
          //点云在处理范围内     &&  点云在目标点相对小车距离+goalClearRange之内 && true
          if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;      //每个方向的角度
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));   //每个方向与目标点的角度
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
            //有一个满足即可  dirToVehicle = false    该if对于我们目前环境来讲，一直为false没有作用
            //(1)方向角度与目标点的角度差值大于车体转向角度的阈值（筛除转向目标要求大于我实际转向的假设方向）(由于我最大转向角是180,不会满足)  && true  一直为false
            //(2)筛除那些假设方向大于我的实际转向角的假设方向（也就是尽量不转向）一直为false && 筛掉目标点要转向90度的 && true （没有向后行驶）       一直为false 
            //(3)dirToVehicle = false 所以一直为false
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              // Y方向上的尺度变换是和生成体素网格时保持一致
              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              // 计算体素网格的索引
              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {    //确保不会越界
                int ind = gridVoxelNumY * indX + indY;  // 得到索引序号
                int blockedPathByVoxelNum = correspondences[ind].size();  // 当前序号的体素网格,占据了多少条路径
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  //!useTerrainAnalysis is false
                  if (h > obstacleHeightThre || !useTerrainAnalysis)
                  { //当前激光点的高度(障碍物高度)大于obstacleHeightThre阈值,则累加,累加代表该方向的paths路径被清除
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  }
                  else
                  {
                    // 在使用了地面分割且激光点分割后高度小于障碍物高度阈值obstacleHeightThre时
                    // 并且 当前高度大于原有值,且大于地面高度阈值groundHeightThre
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                    //当相对高度intensity小于一定障碍物高度阈值obstacleHeightThre 且大于地面高度groundHeightThre的情况下，相当于此处存在坡度则 pathPenaltyList 惩罚值将设置为该相对高度
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }
          //checkRotObstacle 为false，不会执行
          if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
              (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }
//-----------------------------------------------------------------------------------------------------------------------------------------------
//两者传输最重要参数：clearPathList[],代表了因为障碍物而被清除掉的路径信息
//------------------------------------------------对36个方向的paths进行评分（筛除了障碍物的路径的）--------------------------------------------------------------------
        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;   
        score_temp = 0;     //全局变量，每次初始化一下
        dirDiff_temp = 0;
        angle = 0;
        selectedGroupID = -1;
        for (int i = 0; i < 36 * pathNum; i++) {    //pathNum = 343   343条路径，36个方向
          int rotDir = int(i / pathNum);            //第几个方向
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));   //每个方向与目标点的角度差    
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;          //范围[0,180],[0,-180]
          }
          //有一个满足即可  dirToVehicle = false
          //(1)方向角度与目标点的角度差值大于车体转向角度的阈值（筛除转向目标要求大于我实际转向的假设方向）  && true
          //(2)筛除那些假设方向大于我的实际转向角的假设方向（也就是尽量不转向） && 筛掉目标点要转向90度的 && true （没有向后行驶）
          //(3)
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }
          //clearPathList代表了因为障碍物而被清除掉的路径信息
          if (clearPathList[i] < pointPerPathThre) {      //pointPerPathThre = 2
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre; //costHeightThre = 0.1  pathPenaltyList代表了路径的惩罚项,惩罚项是根据地面阈值和障碍物阈值之间的点云高度
            if (penaltyScore < costScore) penaltyScore = costScore; //costscore = 0.02      penaltyScore惩罚得分

            //------------------------------------------------------------旧版-------------------------------------------------------------
            // float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0)); //i % pathNum一条路径下第几个路径点   endDirPathList该条路径末端点与当前位置x轴的角度
            // if (dirDiff > 360.0) {
            //   dirDiff -= 360.0;
            // }
            // if (dirDiff > 180.0) {
            //   dirDiff = 360.0 - dirDiff;  //范围[0,180],[0,-180]
            // }
            //-------------------------------------------------------------------------------------------------------------------------

            //------------------------------------------------------------新版-------------------------------------------------------------
            float angle_temp = endDirPathList[i % pathNum] + (10.0 * rotDir - 180.0);
            if (angle_temp > 180.0) {     //大于180就是负的的范围, -(360 - θ)
              angle_temp -= 360.0;
            }
            if (angle_temp < -180.0) {     //
              angle_temp += 360.0;  //范围[0,180],[0,-180]
            }
            // cout << "angle_temp  = " << angle_temp << endl;
            float dirDiff = fabs(joyDir - angle_temp);
            //重要bug记录:小车向后走时会发现路径会一直左右徘徊，前进是没有问题的
            //bug原因：在算角度范围在[-180,180]的角度的差值时，joyDir在+-180附近时，选取附近点末端路径点，dirDiff算出来会超过180，[-180,180]角度范围的原因：数学的魅力
            //重要bug修复：将过大角度用360减去即可，就是[0,360]之间的差值了
            if (dirDiff > 180.0) {     //大于180就是负的的范围, -(360 - θ)
              dirDiff = 360.0 - dirDiff;
            }

            

            // cout << "dirDiff  = " << dirDiff << endl;
            //-------------------------------------------------------------------------------------------------------------------------

            float rotDirW;  //W代表了该条路径的方向与y轴负方向的角度差
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            //penaltyScore 地面阈值和障碍物阈值之间的点云高度

            //这个 1- 是为了排除一些角度差太大的路径的：重要公式
            // float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;   //有约束版本

            float score = (2.5 - sqrt(sqrt(dirWeight * dirDiff))* penaltyScore);   //无约束版本    修改评分规则，以最短路径作为评分标准,或者最小角度

            // cout << "dirDiff = " << dirDiff << endl;
            
            //pathList[i % pathNum]的值是paths下路径对应startpaths的id值，也就是paths扇形路径偏左边那一块对应startpaths的id值0，以此类推到右边
            //groupNum: 7       rotDir第几个方向               i % pathNum paths一个方向的路径组下的第几条路径
            //groupNum * rotDir + pathList[i % pathNum]第几个方向第几条startpaths路径


            if (score > 0) {
              //clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;     //得到startpaths各个每一个路径点的得分,pathList[pathNum]一组路径的最后路径点的集合
              if(score_temp < score)
              {
                score_temp = score;           //用来选最高评分的
                selectedGroupID = i;          //序号  没啥用
                angle = angle_temp;           //路径点角度
                dirDiff_temp = dirDiff;       //dirDiff最高分数的路径与目标点的角度差值，用来测试的

              }
              // cout << "有得分" << endl;
            } //通过paths得到startPaths的路径得分
            // if(selectedGroupID < 0)
            // {
            //   cout << "selectedGroupID = " << selectedGroupID << " penaltyScore = " << penaltyScore << " score = " << score <<endl;
            // }
            // cout << "有这条路径说明就是有可行路径 penaltyScore = "<< penaltyScore << " score = "<<  score << endl;
          }

        }
        // cout << "score_temp = " << score_temp <<" angle = " << angle << " joyDir = "<< joyDir <<" dirDiff_temp = " << dirDiff_temp << endl;
        
//------------------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------startpaths选择得分最高的路径----------对36个方向的startpaths进行选取---------------------------------------
//不冷不热知识：一组paths分7个扇形区域对应startpaths的1,2,3,4,5,6,7路径，所以某个方向下的paths的那一个扇形区域的路径的评分都会被累加到startpaths对应的路径的分数上（clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score）
//在原版算法中：是在36个paths方向评分后，也就是startpaths36个方向每组路径的评分后，选出来分数maxScore最大的路径，然后给selectedGroupID赋值i,其信息包含第几个方向的startpaths第几条路径，
//最后就让小车跟着这条路径走就行
//
//PYH版算法：修改了评分规则（末端路径点角度与目标点角度最小的分数高），没有了路径约束（全向轮）,只对paths评分，不把分数加到对应的startpaths路径上了（clearPathPerGroupScore注释了）（原因：startpaths路径不够密集）
//直接在paths评分的时候就将分最高的路径选出来selectedGroupID和他的角度angle，然后利用startpaths的路径4（是直线行驶），将路径4旋转，旋转角度为最高分路径末端路径点的角度，所以下面对startpaths选择得分最高的路径可以注释了
//
        // float maxScore = 0;
        // int selectedGroupID = -1;
        // for (int i = 0; i < 36 * groupNum; i++) {   //groupNum = 7  36个方向  七组路径     
        //   int rotDir = int(i / groupNum);     //第几组路径
        //   float rotAng = (10.0 * rotDir - 180.0) * PI / 180;    // 以x方向为0，-180起始逆时针顺序，该组方向的角度
        //   float rotDeg = 10.0 * rotDir;   //第几组路径的x方向的角度值
        //   if (rotDeg > 180.0) rotDeg -= 360.0;

        //   //minObsAngCCW 180.0，minObsAngCW	-180.0保持不变
        //   // 下一个得分大于上一个（是目前最高得分）     &&     ( 注意有括号
        //   // rotAng的角度制 > -180 && rotAng的角度制 < 180（rotAng在正常值内）||   || true
        //   //下面这个也就是选择满足目前最高得分的，因为 后面括号|| 了一个true，所以可以忽略该结果，反正都是true

        //   if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) ||
        //       (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
        //     maxScore = clearPathPerGroupScore[i];
        //     selectedGroupID = i;

        //   }

        // }
//---------------------------------------------------------------------------------------------------------------------------------------------------

        //                  selectedGroupID             选已有路径的第几条                            控制我已经选择路径的方向
        //  筛选最优路径模块 ---------------------> startPaths[selectedGroupID] --------------->cos(rotAng) * x - sin(rotAng) * y
        //
        // cout << "selectedGroupID_1 = " << selectedGroupID  << endl;
        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);       //，前进18，后退0
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;      //rotAng 第几个方向
          // joyDir = joyDir * PI / 180;      //直接用joyDir没有避障了
          angle = angle * PI / 180;           //转弧度制

          selectedGroupID = selectedGroupID % groupNum;       //第几个方向
          // cout << "selectedGroupID = " << selectedGroupID  << endl;
          int selectedPathLength = startPaths[3]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[3]->points[i].x;
            float y = startPaths[3]->points[i].y;
            float z = startPaths[3]->points[i].z;
            float dis = sqrt(x * x + y * y);

            //路径在我们点云处理范围之内，和在小车与目标点的距离之内即可满足iif
            //relativeGoalDis:vehicle下 小车与目标点的距离
            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(angle) * x - sin(angle) * y);    //joyDir 方案①：合适的旋转,利用3来移动   他是先选择路径，然后才根据角度旋转路径的
              path.poses[i].pose.position.y = pathScale * (sin(angle) * x + cos(angle) * y);    //方案②：直接做一个无约束离散点路径
              path.poses[i].pose.position.z = pathScale * z;                                    //方案③（在用）：直接改一个评分规则
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);

          #if PLOTPATHSET == 1
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || 
                !((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
              continue;
            }

            if (clearPathList[i] < pointPerPathThre) {    //clearPathList代表了因为障碍物而被清除掉的路径信息， 小于pointPerPathThre符合
              int freePathLength = paths[i % pathNum]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = ros::Time().fromSec(odomTime);
          freePaths2.header.frame_id = "vehicle";
          pubFreePaths.publish(freePaths2);
          #endif
        }

        
        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      // cout << "pathFound = " << pathFound << " selectedGroupID = " << selectedGroupID << endl;
      if (!pathFound) {       //没有发现路径 pathFound = false时执行
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths.publish(freePaths2);
        // cout << "没有路径了，循环之内" << endl;
        #endif
        
      }

      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
