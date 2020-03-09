#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"  // 相机模型类
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

/**
* @class FeatureTracker
* @Description 视觉前端预处理：对每个相机进行角点LK光流跟踪
*/
class FeatureTracker
{
public:
  // 构造函数
  FeatureTracker();

  // 成员函数
  void readImage(const cv::Mat &_img, double _cur_time);

  void setMask();

  void addPoints();

  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string &calib_file);

  void showUndistortion(const string &name);

  void rejectWithF();

  void undistortedPoints();

  // 数据域
  cv::Mat mask;         // 用来限定需要提取新特征点的当前帧图像区域的mask（ = 鱼眼相机mask + 特征点均匀分布的mask）
  cv::Mat fisheye_mask; // 鱼眼相机mask，用来去除边缘噪点

  cv::Mat prev_img, cur_img, forw_img;              // 前二帧图片，前一帧图片，当前帧图片

  vector<cv::Point2f> n_pts;                        // 当前帧中新提取的特征点

  vector<cv::Point2f> prev_pts, cur_pts, forw_pts;  // 对应图片的二维像素特征点
  vector<cv::Point2f> prev_un_pts, cur_un_pts;      // 归一化平面上的特征点坐标（un->unified）
  vector<cv::Point2f> pts_velocity;                 // 当前帧相对前一帧的二维归一化特征点的移动速度

  vector<int> ids;        // 当前帧图片forw_img中每个特征点的id
  vector<int> track_cnt;  // 当前帧图片forw_img中每个特征点被追踪的次数

  map<int, cv::Point2f> cur_un_pts_map;   // 带有id信息的当前帧归一化特征点
  map<int, cv::Point2f> prev_un_pts_map;  // 带有id信息的前一帧归一化特征点
                                          // map容器用来储存“键值对”，map格式为<id, 归一化点坐标>
                          
  camodocal::CameraPtr m_camera;  // 相机模型对象

  double cur_time;                // 当前帧时间戳
  double prev_time;               // 前一帧时间戳

  static int n_id;                // 用于一次连续跟踪的特征点id的分配，每检测到一个新的特征点，就将n_id作为该特征点的id，然后n_id+1
};
