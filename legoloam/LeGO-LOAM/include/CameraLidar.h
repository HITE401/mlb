#ifndef PINHOLECAMERA_H
#define PINHOLECAMERA_H

#include "utility.h"

//#include <cmath>
//#include <cstdio>
//#include <eigen3/Eigen/Dense>
//#include <iomanip>
//#include <opencv2/core/eigen.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core.hpp>
//#include <string>

//#include "ceres/rotation.h"   // ceres/rotation.h

namespace legoloam
{

class CameraLidar
{
private:
  double m_k1; // 畸变参数
  double m_k2;
  double m_p1;
  double m_p2;

  double m_fx; // 内参矩阵
  double m_fy;
  double m_cx;
  double m_cy;

  double m_inv_K11; // 逆内参矩阵，用于将二维像素坐标逆投影为畸变后的归一化坐标
  double m_inv_K13;
  double m_inv_K22;
  double m_inv_K23;

  Eigen::Matrix3d rotation_lc; // 激光雷达到相机的外参
  Eigen::Vector3d translation_lc;

public:
  // 1. 构造函数和析构函数
  CameraLidar()
  {
    m_k1 = M_k1; // 畸变参数
    m_k2 = M_k2;
    m_p1 = M_p1;
    m_p2 = M_p2;

    m_fx = M_fx; // 内参矩阵
    m_fy = M_fy;
    m_cx = M_cx;
    m_cy = M_cy;

    m_inv_K11 = 1.0 / m_fx; // 逆内参矩阵
    m_inv_K13 = -m_cx / m_fx;
    m_inv_K22 = 1.0 / m_fy;
    m_inv_K23 = -m_cy / m_fy;

    // 激光雷达到相机外参
    rotation_lc << ROT_lc[0], ROT_lc[1], ROT_lc[2],
                   ROT_lc[3], ROT_lc[4], ROT_lc[5],
                   ROT_lc[6], ROT_lc[7], ROT_lc[8];

    translation_lc << TRA_lc[0], TRA_lc[1], TRA_lc[2];
  }
  
  CameraLidar(
      double k1, double k2, double p1, double p2,
      double fx, double fy, double cx, double cy,
      double* rot_lc, double* tra_lc)
  {
    m_k1 = k1; // 畸变参数
    m_k2 = k2;
    m_p1 = p1;
    m_p2 = p2;

    m_fx = fx; // 内参矩阵
    m_fy = fy;
    m_cx = cx;
    m_cy = cy;

    m_inv_K11 = 1.0 / fx; // 逆内参矩阵
    m_inv_K13 = -cx / fx;
    m_inv_K22 = 1.0 / fy;
    m_inv_K23 = -cy / fy;

    // 激光雷达到相机外参
    rotation_lc << rot_lc[0], rot_lc[1], rot_lc[2],
                   rot_lc[3], rot_lc[4], rot_lc[5],
                   rot_lc[6], rot_lc[7], rot_lc[8];

    translation_lc << tra_lc[0], tra_lc[1], tra_lc[2];
  }

  // 2.1 lidarToCamera()
  // 功能: 将激光雷达坐标系下的3D空间点变换到相机坐标系，得到3D坐标
  Eigen::Vector3d lidarToCamera(Eigen::Vector3d P_l)
  {
    return rotation_lc * P_l + translation_lc;
  }

  // 2.2 cameraToNormalized()
  // 功能: 将相机坐标系下的3D空间点投影到归一化平面，得到3D归一化坐标
  Eigen::Vector3d cameraToNormalized(Eigen::Vector3d P_c)
  {
    Eigen::Vector3d p;
    p(0) = P_c(0) / P_c(2);
    p(1) = P_c(1) / P_c(2);
    p(2) = 1;
    return p;
  }

  // 2.3 cameraToImage()
  // 功能: 将相机坐标系下的3D空间点投影到图像平面，得到畸变后的二维像素坐标(Pi function)
  Eigen::Vector2d cameraToImage(Eigen::Vector3d P_c)
  {
    Eigen::Vector3d pi;
    Eigen::Vector2d po;

    // (1) 投影到归一化平面
    pi = cameraToNormalized(P_c);
    double x = pi(0);
    double y = pi(1);

    // (2) 畸变矫正
    // 设归一化坐标系为(x,y,1)，则畸变矫正公式如下:
    // r^2=x^2+y^2; L= 1+k1*r^2+k2*r^4;
    // dx= 2*p1*x*y + p2*(r^2+2*x^2); dy= 2*p2*x*y + p1*(r^2+2*y^2)
    // x_corrected = x*L+dx; y_corrected = y*L+dy;
    double k1 = m_k1;
    double k2 = m_k2;
    double p1 = m_p1;
    double p2 = m_p2;

    double r_square = x * x + y * y;
    double L = 1.0 + k1 * r_square + k2 * r_square * r_square;
    double dx = 2.0 * p1 * x * y + p2 * (r_square + 2.0 * x * x); // 径向畸变矫正+切向畸变矫正
    double dy = 2.0 * p2 * x * y + p1 * (r_square + 2.0 * y * y);

    x = L * x + dx;
    y = L * y + dy;

    // (3) 投影到图像平面，得到像素坐标
    // u = fx*x + cx; v = fy*y + cy
    double fx = m_fx;
    double fy = m_fy;
    double cx = m_cx;
    double cy = m_cy;

    po(0) = fx * x + cx;
    po(1) = fy * y + cy;

    return po;
  }

  // 3. cameraToLidar()
  // 功能: 将激光雷达坐标系下的3D空间点变换到相机坐标系，得到3D坐标
  Eigen::Vector3d cameraToLidar(Eigen::Vector3d P_c)
  {
    return rotation_lc.inverse() * (P_c - translation_lc);
  }
};

// 共享指针，指向PinholeCamera类的实例
typedef boost::shared_ptr<CameraLidar> CameraLidarPtr;            // PinholeCameraPtr
typedef boost::shared_ptr<const CameraLidar> CameraLidarConstPtr; // PinholeCameraConstPtr

} // namespace end

#endif
