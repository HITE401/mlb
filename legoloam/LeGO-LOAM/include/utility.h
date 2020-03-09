#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h" // 用于声明发布到"/segmented_cloud_info"话题的消息

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <Eigen/Core>     // MLB added
#include <Eigen/Geometry>
#include <Eigen/Geometry>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud.h> // 与PointCloud2有什么区别？

#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

//#include "PinholeCamera.h"    // MLB added

#define PI 3.14159265

using namespace std;

// 1. 系统参数 **************************************************************************************
// 1.1 使用的PCL点类型
typedef pcl::PointXYZI PointType;

// 1.2 订阅的原始点云和IMU消息的话题名称
// 运行legoloam官方数据集的订阅话题
// extern const string pointCloudTopic = "/velodyne_points"; // 订阅的原始点云话题
extern const string imuTopic = "/imu/data"; // 订阅的原始IMU话题

// 运行kitti数据集的订阅话题
extern const string pointCloudTopic = "/kitti/velo/pointcloud"; // 订阅的原始点云话题
// extern const string imuTopic = "/kitti/oxts/imu";

// 1.3 保存建图结果的pcd文件路径
extern const string fileDirectory = "/home/lingbo/workspace/tmp/"; // 保存当前关键帧设定半径范围内（500m）的全局点云pcd文件的路径

// 1.4 激光雷达参数
// VLP-16
// extern const int N_SCAN = 16;             // 激光雷达竖直线数
// extern const int Horizon_SCAN = 1800;     // 激光雷达水平点数=360/水平分辨率
// extern const float ang_res_x = 0.2;       // 激光雷达的水平角度分辨率（度）
// extern const float ang_res_y = 2.0;       // 激光雷达的竖直角度分辨率（度）
// extern const float ang_bottom = 15.0+0.1; // 激光雷达的最下面的线的竖直角度（实际为负值-15.0°，转换为正值并加0.1是为了方便整型除法计算）
// extern const int groundScanInd = 7;       // 认为是位于地面的scan的最大行索引+1

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// HDL-64E (Added by MLB -> can run)
/**官方参数为: 
 * 1> Field of View (Vertical): -24.9° ~ +2°; 
 * 2> Angular Resolution (Vertical): 0.4°; 
 * 3> 64 lines;
 * 4> Field of View (Horizontal): 360°; 
 * 5> Angular Resolution (Horizontal): 0.08° ~ 0.35°
 * 6> Rotation Rate: 5 Hz – 20 Hz
 */
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 2000;
extern const float ang_res_x = 360.0 / float(Horizon_SCAN); // 水平角度分辨率: ~ 0.16°
extern const float ang_res_y = 26.9 / float(N_SCAN - 1);    // 竖直角度分辨率:
extern const float ang_bottom = 24.9 + 0.01;
extern const int groundScanInd = 50; // 提取地面时的最大行索引，一般水平角度不超过0度

// HDL-64E reduced to 16 line (Added by MLB -> can run but not good)
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 2000;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 26.9/float(N_SCAN-1);
// extern const float ang_bottom = 24.9+0.01; // +0.1是为了防止之后取整溢出
// extern const int groundScanInd = 12;

// 2. imageProjection节点的参数 **************************************************************************
extern const int segmentValidMinNum = 30; // 原本值为30
extern const int segmentValidMinNum2 = 5; // 原本值为5
extern const int segmentValidLineNum = 3; // 原本值为3，用于判断当前cluster是否是可行分割，若当前cluster点数>=segmentValidPointNum，且包含激光线数>=segmentValidLineNum，则可行

// MLB添加: 用于imageProjection中的cloudSegmentation()函数，用来按大小为cluster排序
struct cluster_t
{
  int size; // cluster的点数
  int ind;  // 索引
};
bool compareClusterSize(cluster_t left, cluster_t right)
{
  return left.size > right.size;
}

// 相机参数
extern const double M_k1 = 0; // 畸变参数
extern const double M_k2 = 0;
extern const double M_p1 = 0;
extern const double M_p2 = 0;

extern const double M_fx = 7.188560000000e+02; // 内参矩阵
extern const double M_fy = 7.188560000000e+02;
extern const double M_cx = 6.071928000000e+02;
extern const double M_cy = 1.852157000000e+02;
                                      // 激光雷达到相机的外参
extern const double ROT_lc[9] = {7.967514e-03, -9.999679e-01, -8.462264e-04,
              -2.771053e-03, 8.241710e-04, -9.999958e-01, 
               9.999644e-01, 7.969825e-03, -2.764397e-03};
extern const double TRA_lc[3] = {-1.377769e-02, -5.542117e-02, -2.918589e-01}; 

// 激光雷达到相机的外参
/*
Eigen::Matrix3d rotation_lc;
Eigen::Vector3d translation_lc;
rotation_lc << 7.967514e-03, -9.999679e-01, -8.462264e-04,
    -2.771053e-03, 8.241710e-04, -9.999958e-01,
    9.999644e-01, 7.969825e-03, -2.764397e-03;
translation_lc << -1.377769e-02, -5.542117e-02, -2.918589e-01;*/

// 3. mapOptimization节点的参数 *****************************************************************************

// 4. transformFusion节点的参数 *****************************************************************************

// 闭环使能标志
extern const bool loopClosureEnableFlag = true;   // false - 关闭; true - 打开;
extern const double mappingProcessInterval = 0.3; // 向量两次执行mapOptimization的最小时间间隔（单位s）

extern const float scanPeriod = 0.1; // 激光雷达一次扫描周期
extern const int systemDelay = 0;
extern const int imuQueLength = 200; // imu循环队列长度

extern const float sensorMountAngle = 0.0;             // 激光雷达安装时与水平面夹角（这里大概为0即可）
extern const float segmentTheta = 60.0 / 180.0 * M_PI; // 用于点云分割函数cloudSegmentation()的角度阈值 // decrese this value may improve accuracy

extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI; // 激光雷达水平角度分辨率的弧度值
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI; // 激光雷达竖直角度分辨率的弧度值

extern const int edgeFeatureNum = 2;                // 一个section提取的sharp点的数目
extern const int surfFeatureNum = 4;                // 一个section提取的flat点的数目
extern const int sectionsTotal = 6;                 // 特征点提取时，将一个scan水平均分的区块数目
extern const float edgeThreshold = 0.1;             // 提取sharp点和lessSharp点的最小曲率阈值
extern const float surfThreshold = 0.1;             // 提取flat点和lessFlat点的最小曲率阈值
extern const float nearestFeatureSearchSqDist = 25; // 查找特征点匹配时，最近邻搜索点距离查询点的最大距离的平方，此处距离设为5m

// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int surroundingKeyframeSearchNum = 50;        // submap size (when loop closure enabled)  // 闭环匹配时，用于提取地图特征点的时间近邻关键帧数目

// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int historyKeyframeSearchNum = 25;       // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment // 闭环ICP匹配得分，越小越好！

extern const float globalMapVisualizationSearchRadius = 5000.0; // key frames with in n meters will be visualized

// 用于定义点云中的点的平滑度（曲率）和排序索引的结构体
struct smoothness_t
{
  float value; // 曲率
  size_t ind;  // 排序索引
};

struct by_value
{
  bool operator()(smoothness_t const &left, smoothness_t const &right)
  {
    return left.value < right.value;
  }
};

// A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

// PointTypePose用于mapOptimization节点的saveKeyFramesAndFactor()函数，用于表示6D位姿
typedef PointXYZIRPYT PointTypePose; // xyz对应平移量，I对应该变量因子图索引，RPY对应欧拉角，T对应该关键帧时间

#endif
