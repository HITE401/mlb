#ifndef PINHOLECAMERA_H
#define PINHOLECAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

#include "ceres/rotation.h"   // ceres/rotation.h
#include "Camera.h"           // Camera.h

namespace camodocal           // camodocal命名空间下
{

class PinholeCamera : public Camera
{
public:
  
  // 1. 相机参数类Parameters的声明
  class Parameters : public Camera::Parameters  //继承
  {
  public:
    // 1.1 构造函数
    Parameters();
    Parameters(const std::string &cameraName,
               int w, int h,
               double k1, double k2, double p1, double p2,
               double fx, double fy, double cx, double cy);
    
    // 1.2 数据域的get函数
      // 畸变参数
    double &k1(void);
    double &k2(void);
    double &p1(void);
    double &p2(void);

      // 相机内参矩阵
    double &fx(void);
    double &fy(void);
    double &cx(void);
    double &cy(void);

      // 
    double xi(void) const;  // ? 继承自父类的
    double k1(void) const;  // ?
    double k2(void) const;
    double p1(void) const;
    double p2(void) const;
    double fx(void) const;  // ?
    double fy(void) const;
    double cx(void) const;
    double cy(void) const;

    // 1.3 readFromYamlFile()
    bool readFromYamlFile(const std::string &filename);

    // 1.4 writeToYamlFile()
    void writeToYamlFile(const std::string &filename) const;

    // 1.5 &operator=() 
    Parameters &operator=(const Parameters &other);

    // 1.6 &operator<<()
    friend std::ostream &operator<<(std::ostream &out, const Parameters &params);

  private:
    double m_k1;  // 畸变参数
    double m_k2;
    double m_p1;
    double m_p2;

    double m_fx;  // 内参矩阵
    double m_fy;
    double m_cx;
    double m_cy;
  };

  // 2. 构造函数
  PinholeCamera();
    // Constructor from the projection model parameters
  PinholeCamera(const std::string &cameraName,
                int imageWidth, int imageHeight,
                double k1, double k2, double p1, double p2,
                double fx, double fy, double cx, double cy);
    // Constructor from the projection model parameters
  PinholeCamera(const Parameters &params);

  // 数据域访问函数
  Camera::ModelType modelType(void) const;    // 获取相机类型，如针孔相机的对应值为=PINHOLE
  const std::string &cameraName(void) const;  // 获取相机名称
  int imageWidth(void) const;                 // 获取图片宽度
  int imageHeight(void) const;                // 获取图片高度

  // 3. estimateIntrinsics()
  void estimateIntrinsics(const cv::Size &boardSize,
                          const std::vector<std::vector<cv::Point3f>> &objectPoints,
                          const std::vector<std::vector<cv::Point2f>> &imagePoints);

  // 4. liftSphere()      // Lift points from the image plane to the sphere
    // 功能: 从二维像素坐标p得到去畸变的三维单位球面坐标P（等价于令三维归一化平面坐标的模长为1）
  virtual void liftSphere(const Eigen::Vector2d &p, Eigen::Vector3d &P) const;
    //%output P

  // 5. liftProjective()  // Lift points from the image plane to the projective space
    // 功能: 利用逆内参矩阵和逆畸变模型，从二维像素坐标p得到去畸变的三维归一化平面坐标P（z=1）
  void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P) const;
    //%output P

  // 6.1 spaceToPlane()
    // Projects 3D points to the image plane (Pi function)
  void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p) const;
    //%output p

  // 6.2 spaceToPlane()
    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
  void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p,
                    Eigen::Matrix<double, 2, 3> &J) const;
    //%output p
    //%output J

  // 7. undistToPlane() 
  void undistToPlane(const Eigen::Vector2d &p_u, Eigen::Vector2d &p) const;
    //%output p

  // 6.3 spaceToPlane()
  template <typename T>
  static void spaceToPlane(const T *const params,
                           const T *const q, const T *const t,
                           const Eigen::Matrix<T, 3, 1> &P,
                           Eigen::Matrix<T, 2, 1> &p);
  // 8.1 distortion()
  void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const;
  
  // 8.2 distortion()
  void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                  Eigen::Matrix2d &J) const;

  // 9. initUndistortMap()
  void initUndistortMap(cv::Mat &map1, cv::Mat &map2, double fScale = 1.0) const;

  // 10. initUndistortRectifyMap()
  cv::Mat initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2,
                                  float fx = -1.0f, float fy = -1.0f,
                                  cv::Size imageSize = cv::Size(0, 0),
                                  float cx = -1.0f, float cy = -1.0f,
                                  cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;
  // 11. parameterCount()
  int parameterCount(void) const;

  // 12. getParameters()
  const Parameters &getParameters(void) const;

  // 13. setParameters()
  void setParameters(const Parameters &parameters);
  
  // 14. readParameters()
  void readParameters(const std::vector<double> &parameterVec);
  
  // 15. writeParameters()
  void writeParameters(std::vector<double> &parameterVec) const;

  // 16. writeParametersToYamlFile()
  void writeParametersToYamlFile(const std::string &filename) const;

  // 17. parametersToString()
  std::string parametersToString(void) const;

private:
  Parameters mParameters;                             // Parameters类的实例
  double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;  // 逆内参矩阵，用于将二维像素坐标逆投影为畸变后的归一化坐标
  bool m_noDistortion;                                // 没有畸变的标志，即此相机模型不考虑畸变
};

// 共享指针，指向PinholeCamera类的实例
typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;            // PinholeCameraPtr
typedef boost::shared_ptr<const PinholeCamera> PinholeCameraConstPtr; // PinholeCameraConstPtr

// 6.3 spaceToPlane() ***********************************************************
/**
 * 功能: 将世界坐标系下的三维空间点P，从世界坐标系投影到相机坐标系，再投影到归一化平面，再去畸变，再投影到图像平面得到二维像素坐标p。
 * 输入:
 *  1> params   - 相机参数，包括畸变参数（k1,k2,p1,p2）和内参矩阵（fx,fy,cx,cy）
 *  2> q        - 世界坐标系到相机坐标系的位姿变换的四元数
 *  3> t        - 世界坐标系到相机坐标系的位姿变换的平移量
 *  4> P        - 世界坐标系下空间点的三维坐标
 * 输出:
 *  1> p        - 图像平面上的二维像素坐标
*/
template <typename T>
void PinholeCamera::spaceToPlane(const T *const params,
                                 const T *const q, const T *const t,
                                 const Eigen::Matrix<T, 3, 1> &P,
                                 Eigen::Matrix<T, 2, 1> &p)
{
  // 1. 将输入的3D空间点的三维坐标的数据格式强转为模板类型，赋给变量P_w
  T P_w[3];
  P_w[0] = T(P(0));
  P_w[1] = T(P(1));
  P_w[2] = T(P(2));

  // 2. 将输入的四元数从Eigen类型转换为Ceres类型，赋给变量q_ceres
    // Convert quaternion from Eigen convention (x, y, z, w)
    // to Ceres convention (w, x, y, z)
  T q_ceres[4] = {q[3], q[0], q[1], q[2]};

  // 3. 将3D空间点变换到相机坐标系下
    // 旋转
  T P_c[3]; // P_c为相机坐标系下的3D空间点坐标
  ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);

    // 平移
  P_c[0] += t[0];
  P_c[1] += t[1];
  P_c[2] += t[2];

  // 4. 将3D空间点从相机坐标系投影到图像平面
    // project 3D object point to the image plane
  T k1 = params[0]; // 畸变参数
  T k2 = params[1];
  T p1 = params[2];
  T p2 = params[3];
  T fx = params[4]; // 内参矩阵
  T fy = params[5];
  T alpha = T(0);   // cameraParams.alpha();
  T cx = params[6];
  T cy = params[7];

    // (1) 从相机坐标系投影到归一化平面坐标
  T u = P_c[0] / P_c[2];  // Pc=(x,y,z); u = x/z; v= y/z;
  T v = P_c[1] / P_c[2];

    // (2) 畸变矫正
      // 设归一化坐标系为(x,y,1)，则畸变矫正公式如下:
      // r^2=x^2+y^2; L= 1+k1*r^2+k2*r^4; 
      // dx= 2*p1*x*y + p2*(r^2+2*x^2); dy= 2*p2*x*y + p1*(r^2+2*y^2)
      // x_corrected = x*L+dx; y_corrected = y*L+dy; 
  T rho_sqr = u * u + v * v;
  T L = T(1.0) + k1 * rho_sqr + k2 * rho_sqr * rho_sqr;
  T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * u * u); // 径向畸变矫正+切向畸变矫正
  T dv = p1 * (rho_sqr + T(2.0) * v * v) + T(2.0) * p2 * u * v;

  u = L * u + du;
  v = L * v + dv;

    // (3) 投影到图像平面，得到像素坐标
      // u = fx*x + cx; v = fy*y + cy
  p(0) = fx * (u + alpha * v) + cx;
  p(1) = fy * v + cy;
}

} // namespace camodocal

#endif
