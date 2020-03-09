#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

// 使用extern关键字声明的全局变量，使其可从外部文件引用
extern int ROW;           // 图像高度（行数）
extern int COL;           // 图像宽度（列数）
extern int FOCAL_LENGTH;  // 焦距
const int NUM_OF_CAM = 1; // 相机个数（单目为1）

extern std::string IMAGE_TOPIC;   // 订阅图像的ROS TOPIC
extern std::string IMU_TOPIC;     // 订阅IMU的ROS TOPIC

extern std::string FISHEYE_MASK;  // 鱼眼相机mask图的位置路径
extern std::vector<std::string> CAM_NAMES;  // 相机参数配置yaml文件的路径（可能有2个相机）

extern int MAX_CNT;         // 单帧图片特征点的最大个数
extern int MIN_DIST;        // 特征点之间的最小间隔（像素距离）
extern int WINDOW_SIZE;     // WINDOW_SIZE是特征点跟踪次数的基准值，用于归一化跟踪次数，来为特征点由红到蓝着色，此处赋值为=20
extern int FREQ;            // 控制图像光流跟踪的频率
extern double F_THRESHOLD;  // RANSAC算法的阈值
extern int SHOW_TRACK;      // 是否发布跟踪点按跟踪次数由多到少标记后的图像（对应由红到蓝）
extern int STEREO_TRACK;    // 双目跟踪则为1
extern int EQUALIZE;        // 如果光太亮或太暗则为1，进行直方图均衡化
extern int FISHEYE;         // 如果是鱼眼相机则为1
extern bool PUB_THIS_FRAME; // 是否需要发布当前帧的标志，用于控制发布频率

// 声明的函数
void readParameters(ros::NodeHandle &n);

