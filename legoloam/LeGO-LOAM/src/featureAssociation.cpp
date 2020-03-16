#include "utility.h"

class FeatureAssociation
{

private:
  // 1. 节点句柄
  ros::NodeHandle nh;

  // 2. 话题订阅器
  ros::Subscriber subLaserCloud;
  ros::Subscriber subLaserCloudInfo;
  ros::Subscriber subOutlierCloud;
  ros::Subscriber subImu;

  // 3. 话题发布器
  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  ros::Publisher pubLaserCloudCornerLast;
  ros::Publisher pubLaserCloudSurfLast;
  ros::Publisher pubLaserOdometry;
  ros::Publisher pubOutlierCloudLast;

  // 4. 存储当前帧订阅的点云消息的变量
  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  // 消息接收成功标志
  bool newSegmentedCloud;
  bool newSegmentedCloudInfo;
  bool newOutlierCloud;

  // 订阅的点云消息的时间戳
  double timeScanCur; 
  double timeNewSegmentedCloud;
  double timeNewSegmentedCloudInfo;
  double timeNewOutlierCloud;

  // 5. 当前帧提取的特征点
  // sharp点、lessSharp点集、flat点集、lessFlat点集
  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  // 用于特征点提取lessFlat点的中间变量，可以放到函数extractFeatures()内！
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  // 体素滤波器
  pcl::VoxelGrid<PointType> downSizeFilter;

  // 6. 系统初始化变量
  int systemInitCount;
  bool systemInited; // 系统初始化标志，false说明系统还未初始化（即系统刚开始运行）

  // 7. 储存一帧点云中点的曲率、曲率排序索引、是否标记为不可选、分类的数组
  std::vector<smoothness_t> cloudSmoothness;      // 储存一帧点云中点的曲率
  float cloudCurvature[N_SCAN * Horizon_SCAN];    // 点云中各点按曲率从大到小排序后对应的索引
  int cloudNeighborPicked[N_SCAN * Horizon_SCAN]; // 点是否标记过的标志：0-未标记过，1-已标记为不可选为特征点的点
  int cloudLabel[N_SCAN * Horizon_SCAN];          // 点分类标号: 2: sharp; 1: lessSharp; -1: flat; 0: lessFlat;（其中1包含了2，0包含了1，0和1构成了点云全部的点）

  // 8. 当前帧点云的起始点IMU状态，当前点IMU状态，以及由加速度产生的两时刻位移变化和速度变化
  // 1> 当前帧点云初始点时刻的IMU欧拉角、速度、位移（通过前后IMU状态线性差值得到）
  float imuRollStart, imuPitchStart, imuYawStart;
  float imuVeloXStart, imuVeloYStart, imuVeloZStart;
  float imuShiftXStart, imuShiftYStart, imuShiftZStart;

  float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;

  // 2> 当前帧点云中当前点时刻的IMU欧拉角、速度和位移（通过前后IMU状态线性差值得到）
  // 当前帧点云去加速度畸变结束后，即为当前帧点云结束点时刻的IMU欧拉角、速度和位移
  float imuRollCur, imuPitchCur, imuYawCur;
  float imuVeloXCur, imuVeloYCur, imuVeloZCur;
  float imuShiftXCur, imuShiftYCur, imuShiftZCur;

  // 3> 当前帧点云中的当前点时刻的loam实时坐标系相对于初始点时刻的loam实时坐标系，由于加速度产生的位移变化、速度变化
  float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
  float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

  // 4> 利用IMU状态线性差值计算的角位移
  float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;    // 线性差值计算的当前帧点云初始点时刻的IMU角位移
  float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast; // 前一帧点云初始点时刻的IMU角位移的差值
  float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;          // 当前帧点云初始点时刻的IMU角位移相对前一帧点云初始点时刻的IMU角位移的差值

  // 用于线性差值计算的IMU数组索引
  int imuPointerFront;         // imu循环数组中时间戳刚好大于当前点云中的当前点的时间戳的位置索引
  int imuPointerLast;          // imu循环数组中最新收到的imu数据在数组中的位置索引（初始值为-1，正向循环取值范围为[0, 1, ..., 199]）
  int imuPointerLastIteration; // 上一次迭代使用的最新帧IMU索引

  // 9. 存储IMU状态的全局循环数组
  // 时间戳
  double imuTime[imuQueLength];

  // 欧拉角（IMU实时坐标系相对于IMU世界坐标系的xyz轴的rpy欧拉角）
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];
  float imuYaw[imuQueLength];

  // IMU真实加速度（沿loam实时坐标系各坐标轴）
  float imuAccX[imuQueLength];
  float imuAccY[imuQueLength];
  float imuAccZ[imuQueLength];

  // 速度（在初始帧loam世界坐标系下，假设初始帧速度为0，利用前一帧速度和当前帧加速度线性积分得到: v_(m+1)=v_m+a_(m+1)*dt）
  float imuVeloX[imuQueLength];
  float imuVeloY[imuQueLength];
  float imuVeloZ[imuQueLength];

  // 位移（在初始帧loam世界坐标系下，假设初始帧速度为0，利用前一帧位移、前一帧速度和当前帧加速度积分得到：x_(m+1) = x_m + v_m*dt + a_(m+1)*dt*dt/2）
  float imuShiftX[imuQueLength];
  float imuShiftY[imuQueLength];
  float imuShiftZ[imuQueLength];

  // 角速度
  float imuAngularVeloX[imuQueLength];
  float imuAngularVeloY[imuQueLength];
  float imuAngularVeloZ[imuQueLength];

  // 角位移
  float imuAngularRotationX[imuQueLength];
  float imuAngularRotationY[imuQueLength];
  float imuAngularRotationZ[imuQueLength];

  // skipFrameNum用于控制发给laserMapping节点的频率为每隔skipFrameNum帧发布一次；frameCount用于计量帧数
  int skipFrameNum;
  bool systemInitedLM;

  // 前一帧点云lessSharp点集的数目，前一帧点云lessFlat点集的数目
  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  // 用于特征匹配查找最近邻点的中间变量
  int pointSelCornerInd[N_SCAN * Horizon_SCAN];
  float pointSearchCornerInd1[N_SCAN * Horizon_SCAN];
  float pointSearchCornerInd2[N_SCAN * Horizon_SCAN];

  int pointSelSurfInd[N_SCAN * Horizon_SCAN];
  float pointSearchSurfInd1[N_SCAN * Horizon_SCAN];
  float pointSearchSurfInd2[N_SCAN * Horizon_SCAN];
  float pointSearchSurfInd3[N_SCAN * Horizon_SCAN];

  // 5. 位姿变换
  float transformCur[6]; // 从当前帧初始时刻loam实时坐标系到当前帧结束时刻loam实时坐标系的位姿变换（依次为利用优化求得的当前帧IMU旋转变换补偿值、匀速运动平移量）；// [pitch(x), yaw(y), roll(z), x, y, z]；
  float transformSum[6]; // 当前帧点云结束点时刻的loam实时坐标系到初始帧点云初始点时刻的loam实时坐标系（世界坐标系）的位姿变换

  // 当前帧点云结束点时刻的IMU欧拉角
  float imuRollLast, imuPitchLast, imuYawLast;
  // 当前帧点云结束点时刻的loam实时坐标系相对于初始点时刻的loam实时坐标系，由于加速度产生的位移变化、速度变化
  float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
  float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;

  // 在L-M优化的一次迭代中，最终筛选后用于优化的的查询特征点，及乘以该特征点权重后的该点到其对应直线或平面的法向量
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  //
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  // LM优化求解的中间变量
  bool isDegenerate;
  cv::Mat matP;

  // 用于函数publishCloudsLast()，控制发布频率
  int frameCount;

  // MLB添加: ************************************
  ros::Subscriber subVisualCloud;
  pcl::PointCloud<PointType>::Ptr visualCloud;
  double timeNewVisualCloud;
  bool newVisualCloud;
  pcl::PointCloud<PointType>::Ptr visualCloudLast;
  ros::Publisher pubVisualCloud;
  ros::Publisher pubVisualCloudLast;

public:
  // Function1: FeatureAssociation() ******************************************
  // 构造函数
  FeatureAssociation() : nh("~")
  {
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
    subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
    subOutlierCloud = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
    subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &FeatureAssociation::imuHandler, this);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

    pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
    pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);

    pubOutlierCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);

    // MLB添加: ********************
    subVisualCloud = nh.subscribe<sensor_msgs::PointCloud2>("/depth_estimation_3d", 1, &FeatureAssociation::viualCloudHandler, this);
    pubVisualCloud = nh.advertise<sensor_msgs::PointCloud2>("/visual_cloud_cur", 1);
    pubVisualCloudLast = nh.advertise<sensor_msgs::PointCloud2>("/visual_cloud_last", 1);

    initializationValue();
  }

  // Function2: initializationValue() ************************************************
  // 功能: 初始化数据域值
  void initializationValue()
  {
    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
    cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
    surfPointsFlat.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

    surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

    timeScanCur = 0;
    timeNewSegmentedCloud = 0;
    timeNewSegmentedCloudInfo = 0;
    timeNewOutlierCloud = 0;

    newSegmentedCloud = false;
    newSegmentedCloudInfo = false;
    newOutlierCloud = false;

    systemInitCount = 0;
    systemInited = false;

    imuPointerFront = 0;
    imuPointerLast = -1;
    imuPointerLastIteration = 0;

    imuRollStart = 0;
    imuPitchStart = 0;
    imuYawStart = 0;
    cosImuRollStart = 0;
    cosImuPitchStart = 0;
    cosImuYawStart = 0;
    sinImuRollStart = 0;
    sinImuPitchStart = 0;
    sinImuYawStart = 0;
    imuRollCur = 0;
    imuPitchCur = 0;
    imuYawCur = 0;

    imuVeloXStart = 0;
    imuVeloYStart = 0;
    imuVeloZStart = 0;
    imuShiftXStart = 0;
    imuShiftYStart = 0;
    imuShiftZStart = 0;

    imuVeloXCur = 0;
    imuVeloYCur = 0;
    imuVeloZCur = 0;
    imuShiftXCur = 0;
    imuShiftYCur = 0;
    imuShiftZCur = 0;

    imuShiftFromStartXCur = 0;
    imuShiftFromStartYCur = 0;
    imuShiftFromStartZCur = 0;
    imuVeloFromStartXCur = 0;
    imuVeloFromStartYCur = 0;
    imuVeloFromStartZCur = 0;

    imuAngularRotationXCur = 0;
    imuAngularRotationYCur = 0;
    imuAngularRotationZCur = 0;
    imuAngularRotationXLast = 0;
    imuAngularRotationYLast = 0;
    imuAngularRotationZLast = 0;
    imuAngularFromStartX = 0;
    imuAngularFromStartY = 0;
    imuAngularFromStartZ = 0;

    for (int i = 0; i < imuQueLength; ++i)
    {
      imuTime[i] = 0;
      imuRoll[i] = 0;
      imuPitch[i] = 0;
      imuYaw[i] = 0;
      imuAccX[i] = 0;
      imuAccY[i] = 0;
      imuAccZ[i] = 0;
      imuVeloX[i] = 0;
      imuVeloY[i] = 0;
      imuVeloZ[i] = 0;
      imuShiftX[i] = 0;
      imuShiftY[i] = 0;
      imuShiftZ[i] = 0;
      imuAngularVeloX[i] = 0;
      imuAngularVeloY[i] = 0;
      imuAngularVeloZ[i] = 0;
      imuAngularRotationX[i] = 0;
      imuAngularRotationY[i] = 0;
      imuAngularRotationZ[i] = 0;
    }

    skipFrameNum = 1;

    for (int i = 0; i < 6; ++i)
    {
      transformCur[i] = 0;
      transformSum[i] = 0;
    }

    systemInitedLM = false;

    imuRollLast = 0;
    imuPitchLast = 0;
    imuYawLast = 0;
    imuShiftFromStartX = 0;
    imuShiftFromStartY = 0;
    imuShiftFromStartZ = 0;
    imuVeloFromStartX = 0;
    imuVeloFromStartY = 0;
    imuVeloFromStartZ = 0;

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/laser_odom";

    laserOdometryTrans.frame_id_ = "/camera_init";      // 父坐标系为"/camera_init"
    laserOdometryTrans.child_frame_id_ = "/laser_odom"; // 子坐标系为"/laser_odom"

    isDegenerate = false;
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    frameCount = skipFrameNum;

    // MLB添加: ****************
    visualCloud.reset(new pcl::PointCloud<PointType>());
    timeNewVisualCloud = 0;
    newVisualCloud = false;
    visualCloudLast.reset(new pcl::PointCloud<PointType>());
  }

  // Function3: updateImuRollPitchYawStartSinCos() ***************************************
  // 功能: 更新当前帧点云起始点的欧拉角的正余弦值，方便之后计算
  void updateImuRollPitchYawStartSinCos()
  {
    cosImuRollStart = cos(imuRollStart);
    cosImuPitchStart = cos(imuPitchStart);
    cosImuYawStart = cos(imuYawStart);
    sinImuRollStart = sin(imuRollStart);
    sinImuPitchStart = sin(imuPitchStart);
    sinImuYawStart = sin(imuYawStart);
  }

  // Function4: ShiftToStartIMU(float pointTime) ******************************************
  // 功能: 计算从当前帧点云初始点时刻到当前点时刻，由于实时加速度导致的IMU位移变化（转化到初始点时刻的loam实时坐标系下）。
  void ShiftToStartIMU(float pointTime)
  {
    imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
    imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
    imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

    float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
    float y1 = imuShiftFromStartYCur;
    float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

    float x2 = x1;
    float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
    float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

    imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
    imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
    imuShiftFromStartZCur = z2;
  }

  // Function5: VeloToStartIMU() ***********************************************************
  // 功能: 计算从当前帧点云初始点时刻到当前点时刻，由于实时加速度导致的IMU速度变化（转化到初始点时刻的loam实时坐标系下）
  void VeloToStartIMU()
  {
    imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
    imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
    imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

    float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
    float y1 = imuVeloFromStartYCur;
    float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

    float x2 = x1;
    float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
    float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

    imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
    imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
    imuVeloFromStartZCur = z2;
  }

  // Function6: TransformToStartIMU(PointType *p) ********************************************
  // 功能: 将当前帧点云的当前点从当前点时刻的loam实时坐标系变换到初始点时刻的loam实时坐标系。旋转变换利用两时刻欧拉角，位移变换仅考虑两时刻间的加速度位移量。
  void TransformToStartIMU(PointType *p)
  {
    float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    float z1 = p->z;

    float x2 = x1;
    float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    float y3 = y2;
    float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
    float y4 = y3;
    float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

    float x5 = x4;
    float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
    float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

    p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
    p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
    p->z = z5 + imuShiftFromStartZCur;
  }

  // Function7: AccumulateIMUShiftAndRotation() ***************************************************
  /**
  * 功能：
  *   该函数被imuHandler()函数调用。实现功能如下：
  *   - 利用当前帧rpy欧拉角，将全局数组中的当前帧IMU加速度从loam实时坐标系变换到y轴与反向重力加速度重合的loam世界坐标系;
  *   - 在初始IMU帧的loam世界坐标系下，假设初始帧速度为0，利用前一帧速度和当前帧加速度，线性积分得到当前帧速度: v_(m+1)=v_m+a_(m+1)*dt;
  *   - 在初始IMU帧的loam世界坐标系下，假设初始帧速度为0，利用前一帧位移、前一帧速度和当前帧加速度积分得到当前帧位移：x_(m+1) = x_m + v_m*dt + a_(m+1)*dt*dt/2;
  *   - 在loam世界坐标系下，利用前一帧IMU消息时刻的角位移、角速度，线性积分求得当前帧IMU消息时刻的角位移: r_(m+1) = r_m + w_m*dt;
  */
  void AccumulateIMUShiftAndRotation()
  {
    float roll = imuRoll[imuPointerLast];
    float pitch = imuPitch[imuPointerLast];
    float yaw = imuYaw[imuPointerLast];
    float accX = imuAccX[imuPointerLast];
    float accY = imuAccY[imuPointerLast];
    float accZ = imuAccZ[imuPointerLast];

    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    if (timeDiff < scanPeriod)
    {

      imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
      imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
      imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

      imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
      imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
      imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

      imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
      imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
      imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
    }
  }

  // Function8: imuHandler() ******************************************************************
  /**
   * 功能：接受原始的当前帧IMU消息，
   *   1> 获得IMU实时坐标系相对于IMU世界坐标系的rpy欧拉角;
   *   2> 利用欧拉角，将重力加速度变换到IMU实时坐标系下，计算得IMU实时坐标系下的真实加速度，并将其转换为loam实时坐标系下;
   *   3> 将当前帧IMU消息的数组索引、时间戳、欧拉角、真实线加速度（loam实时坐标系下）、角速度，保存到全局变量数组中;
   *   4> 调用函数AccumulateIMUShiftAndRotation();
  */
  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn)
  {
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
    float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
    float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();

    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;

    imuAccX[imuPointerLast] = accX;
    imuAccY[imuPointerLast] = accY;
    imuAccZ[imuPointerLast] = accZ;

    imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x; 
    imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
    imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

    AccumulateIMUShiftAndRotation();
  }

  // Function9: laserCloudHandler() ************************************************************
  // 功能:
  // 接收话题"/segmented_cloud"发布的当前帧点云消息，保存消息header至类变量cloudHeader，
  // 保存消息时间戳到类变量timeScanCur和timeNewSegmentedCloud，
  // 保存点云至类变量segmentedCloud，将接收成功标志newSegmentedCloud置为true
  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    cloudHeader = laserCloudMsg->header;

    timeScanCur = cloudHeader.stamp.toSec();
    timeNewSegmentedCloud = timeScanCur;

    segmentedCloud->clear();
    pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

    newSegmentedCloud = true;
  }

  // Function10: outlierCloudHandler() ***********************************************************
  // 功能: 接收话题"/outlier_cloud"发布的点云消息，保存时间戳至timeNewOutlierCloud，
  // 保存点云至outlierCloud，将接收成功标志newOutlierCloud置为true
  void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn)
  {
    timeNewOutlierCloud = msgIn->header.stamp.toSec();

    outlierCloud->clear();
    pcl::fromROSMsg(*msgIn, *outlierCloud);

    newOutlierCloud = true;
  }

  // Function11: laserCloudInfoHandler() **********************************************************
  // 功能: 接收话题"/segmented_cloud_info"发布的cloud_info消息，保存时间戳至timeNewSegmentedCloudInfo，
  // 保存消息变量指针至segInfo，将接收成功标志newSegmentedCloudInfo置为true
  void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr &msgIn)
  {
    timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
    segInfo = *msgIn;
    newSegmentedCloudInfo = true;
  }


  // Function12.2 adjustDistortion() *************************************
  // 注: 此函数为针对此函数为针对kitti数据集的坐标系和时间戳定义进行修改的版本。
  void adjustDistortion()
  {
    bool halfPassed = false;
    int cloudSize = segmentedCloud->points.size();

    PointType point;

    for (int i = 0; i < cloudSize; i++)
    {
      point.x = segmentedCloud->points[i].y;
      point.y = segmentedCloud->points[i].z;
      point.z = segmentedCloud->points[i].x;

      // 方案七：顺时针旋转，起点为-x轴，但取前一帧中间时刻为时间0点，则当前帧相对时间范围为[0.5,1.5]
      float columnInd = (segmentedCloud->points[i].intensity - int(segmentedCloud->points[i].intensity)) * 10000;
      float relTime = (1 - columnInd / Horizon_SCAN) + 0.5;
      point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;

      // 如果收到IMU数据，则使用IMU矫正加速度导致的点云畸变
      if (imuPointerLast >= 0) 
      {
        float pointTime = relTime * scanPeriod;

        imuPointerFront = imuPointerLastIteration;
        while (imuPointerFront != imuPointerLast)
        {
          if (timeScanCur + pointTime < imuTime[imuPointerFront])
          {
            break;
          }
          imuPointerFront = (imuPointerFront + 1) % imuQueLength;
        }

        if (timeScanCur + pointTime > imuTime[imuPointerFront])
        {
          imuRollCur = imuRoll[imuPointerFront];
          imuPitchCur = imuPitch[imuPointerFront];
          imuYawCur = imuYaw[imuPointerFront];

          imuVeloXCur = imuVeloX[imuPointerFront];
          imuVeloYCur = imuVeloY[imuPointerFront];
          imuVeloZCur = imuVeloZ[imuPointerFront];

          imuShiftXCur = imuShiftX[imuPointerFront];
          imuShiftYCur = imuShiftY[imuPointerFront];
          imuShiftZCur = imuShiftZ[imuPointerFront];
        }
 
        else
        {
          int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;

          float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
          float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

          imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
          imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
          if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI)
          {
            imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
          }
          else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI)
          {
            imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
          }
          else
          {
            imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
          }

          //本质:imuVeloXCur = imuVeloX[imuPointerback] + (imuVelX[imuPointerFront]-imuVelX[imuPoniterBack])*ratioFront
          imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
          imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
          imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

          // 按照时间距离线性差值得到当前点的位移
          imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
          imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
          imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
        }

        if (i == 0)
        {
          imuRollStart = imuRollCur;
          imuPitchStart = imuPitchCur;
          imuYawStart = imuYawCur;

          imuVeloXStart = imuVeloXCur;
          imuVeloYStart = imuVeloYCur;
          imuVeloZStart = imuVeloZCur;

          imuShiftXStart = imuShiftXCur;
          imuShiftYStart = imuShiftYCur;
          imuShiftZStart = imuShiftZCur;

          // 线性差值计算当前帧点云初始点的角位移
          if (timeScanCur + pointTime > imuTime[imuPointerFront])
          {
            imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
            imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
            imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
          }
          else
          {
            int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
            imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
            imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
          }

          // 计算当前帧点云初始点的角位移相对于前一帧点云初始点的角位移的差值
          imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
          imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
          imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

          imuAngularRotationXLast = imuAngularRotationXCur;
          imuAngularRotationYLast = imuAngularRotationYCur;
          imuAngularRotationZLast = imuAngularRotationZCur;

          // 更新当前帧点云起始点的欧拉角的正余弦值，方便之后计算
          updateImuRollPitchYawStartSinCos();
        }
        else
        {
          ShiftToStartIMU(pointTime);
          VeloToStartIMU();
          TransformToStartIMU(&point);
        }
      }

      segmentedCloud->points[i] = point;
    }

    imuPointerLastIteration = imuPointerLast;
  }

  // Function13: calculateSmoothness() ***************************************************
  // 遍历点云segmentedCloud，计算各点的曲率，结果分别存入cloudCurvature[]、cloudNeighborPicked[]、cloudLabel[]、cloudSmoothness[]
  void calculateSmoothness()
  {
    int cloudSize = segmentedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {
      float diffRange = segInfo.segmentedCloudRange[i - 5] + segInfo.segmentedCloudRange[i - 4] + segInfo.segmentedCloudRange[i - 3] + segInfo.segmentedCloudRange[i - 2] + segInfo.segmentedCloudRange[i - 1] - segInfo.segmentedCloudRange[i] * 10 + segInfo.segmentedCloudRange[i + 1] + segInfo.segmentedCloudRange[i + 2] + segInfo.segmentedCloudRange[i + 3] + segInfo.segmentedCloudRange[i + 4] + segInfo.segmentedCloudRange[i + 5];

      cloudCurvature[i] = diffRange * diffRange; // 曲率计算

      cloudNeighborPicked[i] = 0; // 初始时，点全未筛选过
      cloudLabel[i] = 0;          // 初始化为lessFlat点

      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind = i; // 记录曲率点的排序索引
    }
  }

  // markOccludedPoints() *******************************************************
  /** 功能: 遍历点云segmentedCloud，剔除不可靠点
   * (1) 排除被遮挡区域边界的点，将其在cloudNeighborPicked中标记为1
   * (2) 排除与激光束接近平行平面上的点，将其在cloudNeighborPicked中标记为1
   */
  void markOccludedPoints()
  {
    int cloudSize = segmentedCloud->points.size();

    for (int i = 5; i < cloudSize - 6; ++i)
    {

      float depth1 = segInfo.segmentedCloudRange[i];
      float depth2 = segInfo.segmentedCloudRange[i + 1];
      int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i + 1] - segInfo.segmentedCloudColInd[i]));

      if (columnDiff < 10)
      {

        if (depth1 - depth2 > 0.3)
        {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
        else if (depth2 - depth1 > 0.3)
        {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }

      float diff1 = std::abs(segInfo.segmentedCloudRange[i - 1] - segInfo.segmentedCloudRange[i]);
      float diff2 = std::abs(segInfo.segmentedCloudRange[i + 1] - segInfo.segmentedCloudRange[i]);

      if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
        cloudNeighborPicked[i] = 1;
    }
  }

  // extractFeatures() *****************************************************
  // 功能: 按照曲率大小，将每条scan等分为6个区块，从每个区块提取2个sharp点，20个lessSharp点，4个flat点，
  //      将所有scan汇总后分别存入cornerPointsSharp、cornerPointsLessSharp、surfPointsFlat；
  //      将除去lessSharp点的当前帧所有点（cloudLabel[k] <= 0）作为lessFlat点，进行体素滤波，再汇总所有scan后存入surfPointsLessFlat；
  void extractFeatures()
  {
    cornerPointsSharp->clear();     // 当前帧点云所有的曲率极大点（sharp），在数组cloudLabel中标记为2
    cornerPointsLessSharp->clear(); // 当前帧点云所有的曲率欠极大点（lessSharp），在数组cloudLabel中标记为1，包括了sharp点集
    surfPointsFlat->clear();        // 当前帧点云所有的曲率极小点（Flat），在数组cloudLabel中标记为-1
    surfPointsLessFlat->clear();    // 当前帧点云所有的曲率欠极小点（lessFlat），在数组cloudLabel中标记为0（初始时均标记为0）；其为除去sharp和lessSharp点集后当前帧点云laserCloud中的所有有曲率点降采样后的点集

    for (int i = 0; i < N_SCAN; i++)
    {
      surfPointsLessFlatScan->clear(); // 当前scan中的lessFlat点集
      for (int j = 0; j < 6; j++)
      {
        int sp = (segInfo.startRingIndex[i] * (6 - j) + segInfo.endRingIndex[i] * j) / 6;
        int ep = (segInfo.startRingIndex[i] * (5 - j) + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

        if (sp >= ep)
          continue;

        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--)
        {
          int ind = cloudSmoothness[k].ind;

          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] > edgeThreshold &&
              segInfo.segmentedCloudGroundFlag[ind] == false)
          {
            largestPickedNum++;
            if (largestPickedNum <= 2) // 2个sharp点
            {
              cloudLabel[ind] = 2;
              cornerPointsSharp->push_back(segmentedCloud->points[ind]);
              cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
            }
            else if (largestPickedNum <= 20) // 20个lessSharp点
            {
              cloudLabel[ind] = 1;
              cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
            }
            else
            {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            for (int l = 1; l <= 5; l++)
            {
              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        int smallestPickedNum = 0;
        for (int k = sp; k <= ep; k++)
        {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 &&
              cloudCurvature[ind] < surfThreshold &&
              segInfo.segmentedCloudGroundFlag[ind] == true)
          {
            cloudLabel[ind] = -1;
            surfPointsFlat->push_back(segmentedCloud->points[ind]);

            smallestPickedNum++;
            if (smallestPickedNum >= 4)
            {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            for (int l = 1; l <= 5; l++)
            {

              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;

              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {

              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;

              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        // 将除去lessSharp点云外的所有点全部归入平面点中lessFlat类别中
        for (int k = sp; k <= ep; k++)
        {
          if (cloudLabel[k] <= 0)
          {
            surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
          }
        }
      }

      surfPointsLessFlatScanDS->clear();
      downSizeFilter.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter.filter(*surfPointsLessFlatScanDS);

      *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    }
  }

  // publishCloud() ******************************************
  // 功能: 发布当前帧提取的sharp、lessSharp、flat和lessFlat特征点集
  void publishCloud()
  {
    sensor_msgs::PointCloud2 laserCloudOutMsg;

    if (pubCornerPointsSharp.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = cloudHeader.stamp; // 时间戳
      laserCloudOutMsg.header.frame_id = "/camera";      // 坐标系
      pubCornerPointsSharp.publish(laserCloudOutMsg);
    }

    if (pubCornerPointsLessSharp.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubCornerPointsLessSharp.publish(laserCloudOutMsg);
    }

    if (pubSurfPointsFlat.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubSurfPointsFlat.publish(laserCloudOutMsg);
    }

    if (pubSurfPointsLessFlat.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubSurfPointsLessFlat.publish(laserCloudOutMsg);
    }
  }

  // TransformToStart() ******************************************************
  void TransformToStart(PointType const *const pi, PointType *const po)
  {
    float s = 10 * (pi->intensity - int(pi->intensity)); // 10为周期0.1s的倒数，s=相对比例时间

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
  }

  // TransformToStart()函数的重载函数，给定时间比例因子s
  void TransformToStart(PointType const *const pi, PointType *const po, float s)
  {
    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
  }

  // TransformToEnd()
  void TransformToEnd(PointType const *const pi, PointType *const po)
  {
    float s = 10 * (pi->intensity - int(pi->intensity));

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    float x3 = cos(ry) * x2 - sin(ry) * z2;
    float y3 = y2;
    float z3 = sin(ry) * x2 + cos(ry) * z2;

    rx = transformCur[0];
    ry = transformCur[1];
    rz = transformCur[2];
    tx = transformCur[3];
    ty = transformCur[4];
    tz = transformCur[5];

    float x4 = cos(ry) * x3 + sin(ry) * z3;
    float y4 = y3;
    float z4 = -sin(ry) * x3 + cos(ry) * z3;

    float x5 = x4;
    float y5 = cos(rx) * y4 - sin(rx) * z4;
    float z5 = sin(rx) * y4 + cos(rx) * z4;

    float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
    float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
    float z6 = z5 + tz;

    float x7 = cosImuRollStart * (x6 - imuShiftFromStartX) - sinImuRollStart * (y6 - imuShiftFromStartY);
    float y7 = sinImuRollStart * (x6 - imuShiftFromStartX) + cosImuRollStart * (y6 - imuShiftFromStartY);
    float z7 = z6 - imuShiftFromStartZ;

    float x8 = x7;
    float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
    float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

    float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
    float y9 = y8;
    float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

    float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
    float y10 = y9;
    float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

    float x11 = x10;
    float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
    float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

    po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
    po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
    po->z = z11;
    po->intensity = int(pi->intensity);
  }

  // PluginIMURotation() ********************
  void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
                         float alx, float aly, float alz, float &acx, float &acy, float &acz)
  {
    float sbcx = sin(bcx);
    float cbcx = cos(bcx);
    float sbcy = sin(bcy);
    float cbcy = cos(bcy);
    float sbcz = sin(bcz);
    float cbcz = cos(bcz);

    float sblx = sin(blx);
    float cblx = cos(blx);
    float sbly = sin(bly);
    float cbly = cos(bly);
    float sblz = sin(blz);
    float cblz = cos(blz);

    float salx = sin(alx);
    float calx = cos(alx);
    float saly = sin(aly);
    float caly = cos(aly);
    float salz = sin(alz);
    float calz = cos(alz);

    float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly) - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
    acx = -asin(srx);

    float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
    float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
    acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

    float srzcrx = sbcx * (cblx * cbly * (calz * saly - caly * salx * salz) - cblx * sbly * (caly * calz + salx * saly * salz) + calx * salz * sblx) - cbcx * cbcz * ((caly * calz + salx * saly * salz) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cblz * salz) + cbcx * sbcz * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * cblx * salz * sblz);
    float crzcrx = sbcx * (cblx * sbly * (caly * salz - calz * salx * saly) - cblx * cbly * (saly * salz + caly * calz * salx) + calx * calz * sblx) + cbcx * cbcz * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * calz * cblx * cblz) - cbcx * sbcz * ((saly * salz + caly * calz * salx) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (cbly * cblz + sblx * sbly * sblz) - calx * calz * cblx * sblz);
    acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
  }

  // AccumulateRotation() ****************************************
  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                          float &ox, float &oy, float &oz)
  {
    float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) - cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
    ox = -asin(srx);

    float srycrx = sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) + cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) + cos(lx) * cos(ly) * cos(cx) * sin(cy);
    float crycrx = cos(lx) * cos(ly) * cos(cx) * cos(cy) - cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) - sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
    oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

    float srzcrx = sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) + cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) + cos(lx) * cos(cx) * cos(cz) * sin(lz);
    float crzcrx = cos(lx) * cos(lz) * cos(cx) * cos(cz) - cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) - sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
    oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
  }

  // rad2deg() ************************************************
  double rad2deg(double radians)
  {
    return radians * 180.0 / M_PI;
  }

  // deg2rad() ***********************************************
  double deg2rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  // findCorrespondingCornerFeatures()
  // 功能: 从前一帧的lessSharp点集中查找当前帧当前sharp特征点的两个最近邻点
  void findCorrespondingCornerFeatures(int iterCount)
  {
    int cornerPointsSharpNum = cornerPointsSharp->points.size(); 
    for (int i = 0; i < cornerPointsSharpNum; i++)
    {
      TransformToStart(&cornerPointsSharp->points[i], &pointSel); // output: pointSel

      if (iterCount % 5 == 0)
      {
        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
        int closestPointInd = -1, minPointInd2 = -1;

        if (pointSearchSqDis[0] < nearestFeatureSearchSqDist)
        {
          closestPointInd = pointSearchInd[0];
          int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

          float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist; // minPointSqDis2是所有遍历点中到查询点的最小距离
          for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
          {
            if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 1.5)
            {
              break;
            }

            pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                             (laserCloudCornerLast->points[j].x - pointSel.x) +
                         (laserCloudCornerLast->points[j].y - pointSel.y) *
                             (laserCloudCornerLast->points[j].y - pointSel.y) +
                         (laserCloudCornerLast->points[j].z - pointSel.z) *
                             (laserCloudCornerLast->points[j].z - pointSel.z);

            if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
            }
          }

          for (int j = closestPointInd - 1; j >= 0; j--)
          {
            if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 1.5)
            {
              break;
            }

            pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                             (laserCloudCornerLast->points[j].x - pointSel.x) +
                         (laserCloudCornerLast->points[j].y - pointSel.y) *
                             (laserCloudCornerLast->points[j].y - pointSel.y) +
                         (laserCloudCornerLast->points[j].z - pointSel.z) *
                             (laserCloudCornerLast->points[j].z - pointSel.z);

            if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
            }
          }
        }

        pointSearchCornerInd1[i] = closestPointInd;
        pointSearchCornerInd2[i] = minPointInd2;
      }

      if (pointSearchCornerInd2[i] >= 0)
      {
        tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
        tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;

        float x1 = tripod1.x;
        float y1 = tripod1.y;
        float z1 = tripod1.z;

        float x2 = tripod2.x;
        float y2 = tripod2.y;
        float z2 = tripod2.z;

        float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
        float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
        float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

        float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

        float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;  // x轴分量
        float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12; // y轴分量
        float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12; //z轴分量

        float ld2 = a012 / l12;

        float s = 1;
        if (iterCount >= 5)
        {
          s = 1 - 1.8 * fabs(ld2);
        }

        if (s > 0.1 && ld2 != 0)
        {
          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          laserCloudOri->push_back(cornerPointsSharp->points[i]);
          coeffSel->push_back(coeff);
        }
      }
    }
  }

  // findCorrespondingSurfFeatures() **************************************
  void findCorrespondingSurfFeatures(int iterCount)
  {
    int surfPointsFlatNum = surfPointsFlat->points.size();

    // 遍历当前帧所有flat特征点
    for (int i = 0; i < surfPointsFlatNum; i++)
    {
      TransformToStart(&surfPointsFlat->points[i], &pointSel);

      if (iterCount % 5 == 0)
      {
        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis); // K=1，只搜一个最近邻点
        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

        if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) // < 5m ^2
        {
          closestPointInd = pointSearchInd[0];
          int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

          float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
          for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
          {
            if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                             (laserCloudSurfLast->points[j].x - pointSel.x) +
                         (laserCloudSurfLast->points[j].y - pointSel.y) *
                             (laserCloudSurfLast->points[j].y - pointSel.y) +
                         (laserCloudSurfLast->points[j].z - pointSel.z) *
                             (laserCloudSurfLast->points[j].z - pointSel.z);

            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
            }
            else
            {
              if (pointSqDis < minPointSqDis3)
              {
                minPointSqDis3 = pointSqDis;
                minPointInd3 = j;
              }
            }
          }
          for (int j = closestPointInd - 1; j >= 0; j--)
          {
            if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                             (laserCloudSurfLast->points[j].x - pointSel.x) +
                         (laserCloudSurfLast->points[j].y - pointSel.y) *
                             (laserCloudSurfLast->points[j].y - pointSel.y) +
                         (laserCloudSurfLast->points[j].z - pointSel.z) *
                             (laserCloudSurfLast->points[j].z - pointSel.z);

            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2 = j;
              }
            }
            else
            {
              if (pointSqDis < minPointSqDis3)
              {
                minPointSqDis3 = pointSqDis;
                minPointInd3 = j;
              }
            }
          }
        }

        // 3个最近邻点的数组索引
        pointSearchSurfInd1[i] = closestPointInd;
        pointSearchSurfInd2[i] = minPointInd2;
        pointSearchSurfInd3[i] = minPointInd3;
      }

      // 如果3个最近邻点都找到，则构建当前flat特征点的残差laserCloudOri、coeffSel
      if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0)
      {

        tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
        tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
        tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

        float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
        float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
        float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
        float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

        float ps = sqrt(pa * pa + pb * pb + pc * pc);

        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1;
        if (iterCount >= 5)
        {
          s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
        }

        if (s > 0.1 && pd2 != 0)
        {
          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          laserCloudOri->push_back(surfPointsFlat->points[i]);
          coeffSel->push_back(coeff);
        }
      }
    }
  }

  // calculateTransformationSurf() ***************************
  bool calculateTransformationSurf(int iterCount)
  {

    int pointSelNum = laserCloudOri->points.size();

    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    for (int i = 0; i < pointSelNum; i++)
    {
      pointOri = laserCloudOri->points[i];
      coeff = coeffSel->points[i];

      //
      float s = 10 * (pointOri.intensity - int(pointOri.intensity)); // 此处相对时间比例因子s，为MLB添加

      float srx = sin(s * transformCur[0]);
      float crx = cos(s * transformCur[0]);
      float sry = sin(s * transformCur[1]);
      float cry = cos(s * transformCur[1]);
      float srz = sin(s * transformCur[2]);
      float crz = cos(s * transformCur[2]);
      float tx = s * transformCur[3];
      float ty = s * transformCur[4];
      float tz = s * transformCur[5];

      float a1 = crx * sry * srz;
      float a2 = crx * crz * sry;
      float a3 = srx * sry;
      float a4 = tx * a1 - ty * a2 - tz * a3;
      float a5 = srx * srz;
      float a6 = crz * srx;
      float a7 = ty * a6 - tz * crx - tx * a5;
      float a8 = crx * cry * srz;
      float a9 = crx * cry * crz;
      float a10 = cry * srx;
      float a11 = tz * a10 + ty * a9 - tx * a8;

      float b1 = -crz * sry - cry * srx * srz;
      float b2 = cry * crz * srx - sry * srz;
      float b5 = cry * crz - srx * sry * srz;
      float b6 = cry * srz + crz * srx * sry;

      float c1 = -b6;
      float c2 = b5;
      float c3 = tx * b6 - ty * b5;
      float c4 = -crx * crz;
      float c5 = crx * srz;
      float c6 = ty * c5 + tx * -c4;
      float c7 = b2;
      float c8 = -b1;
      float c9 = tx * -b2 - ty * -b1;
      //

      float arx = (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x 
                + (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y 
                + (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

      float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x 
                + (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y 
                + (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

      float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

      float d2 = coeff.intensity;

      // MLB添加
      arx = s * arx;
      arz = s * arz;
      aty = s * aty;

      matA.at<float>(i, 0) = arx;
      matA.at<float>(i, 1) = arz;
      matA.at<float>(i, 2) = aty;
      matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0)
    {
      cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate = false;
      float eignThre[3] = {10, 10, 10};
      for (int i = 2; i >= 0; i--)
      {
        if (matE.at<float>(0, i) < eignThre[i])
        {
          for (int j = 0; j < 3; j++)
          {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        }
        else
        {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
      cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformCur[0] += matX.at<float>(0, 0); // pitch(x) - 对应loam坐标系下的绕水平x轴的转角
    transformCur[2] += matX.at<float>(1, 0); // roll(z)  - 对应loam坐标系下的绕水平z轴的转角
    transformCur[4] += matX.at<float>(2, 0); // y        - 对应loam坐标系下的竖直y轴方向的位移

    for (int i = 0; i < 6; i++)
    {
      if (isnan(transformCur[i]))
        transformCur[i] = 0;
    }

    float deltaR = sqrt(
        pow(rad2deg(matX.at<float>(0, 0)), 2) +
        pow(rad2deg(matX.at<float>(1, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1)
    {
      return false;
    }
    return true;
  }

  // calculateTransformationCorner() ********************************
  // 功能:
  bool calculateTransformationCorner(int iterCount)
  {
    // 当前查找到的特征点匹配的数目
    int pointSelNum = laserCloudOri->points.size();

    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    for (int i = 0; i < pointSelNum; i++)
    {
      pointOri = laserCloudOri->points[i];  
      coeff = coeffSel->points[i];

      // 当前特征点的时间比例因子s
      float s = 10 * (pointOri.intensity - int(pointOri.intensity)); // 此处时间比例因子s，为MLB添加

      // 中间变量
      float srx = sin(s * transformCur[0]);
      float crx = cos(s * transformCur[0]);
      float sry = sin(s * transformCur[1]);
      float cry = cos(s * transformCur[1]);
      float srz = sin(s * transformCur[2]);
      float crz = cos(s * transformCur[2]);
      float tx = s * transformCur[3];
      float ty = s * transformCur[4];
      float tz = s * transformCur[5];

      float b1 = -crz * sry - cry * srx * srz;
      float b2 = cry * crz * srx - sry * srz;
      float b3 = crx * cry;
      float b4 = tx * -b1 + ty * -b2 + tz * b3;
      float b5 = cry * crz - srx * sry * srz;
      float b6 = cry * srz + crz * srx * sry;
      float b7 = crx * sry;
      float b8 = tz * b7 - ty * b6 - tx * b5;

      float c5 = crx * srz; // MLB End

      float ary = (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x + (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

      float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

      float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

      float d2 = coeff.intensity;

      // MLB添加: 新增比例因子s
      ary = s * ary;
      atx = s * atx;
      atz = s * atz;

      matA.at<float>(i, 0) = ary;
      matA.at<float>(i, 1) = atx;
      matA.at<float>(i, 2) = atz;
      matB.at<float>(i, 0) = -0.05 * d2; // 比例0.05
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR); // output: matX

    if (iterCount == 0)
    {
      cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate = false;
      float eignThre[3] = {10, 10, 10};
      for (int i = 2; i >= 0; i--)
      {
        if (matE.at<float>(0, i) < eignThre[i])
        {
          for (int j = 0; j < 3; j++)
          {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        }
        else
        {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
      cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    // 得到更新的全局变量transformCur
    transformCur[1] += matX.at<float>(0, 0);
    transformCur[3] += matX.at<float>(1, 0);
    transformCur[5] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++)
    {
      if (isnan(transformCur[i]))
        transformCur[i] = 0;
    }

    float deltaR = sqrt(
        pow(rad2deg(matX.at<float>(0, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(1, 0) * 100, 2) +
        pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1)
    {
      return false;
    }
    return true;
  }

  // calculateTransformation() ***********************************
  bool calculateTransformation(int iterCount)
  {
    int pointSelNum = laserCloudOri->points.size();

    cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    for (int i = 0; i < pointSelNum; i++)
    {

      pointOri = laserCloudOri->points[i];
      coeff = coeffSel->points[i];

      // 当前特征点的时间比例因子s
      float s = 10 * (pointOri.intensity - int(pointOri.intensity));

      // 中间变量
      float srx = sin(s * transformCur[0]);
      float crx = cos(s * transformCur[0]);
      float sry = sin(s * transformCur[1]);
      float cry = cos(s * transformCur[1]);
      float srz = sin(s * transformCur[2]);
      float crz = cos(s * transformCur[2]);
      float tx = s * transformCur[3];
      float ty = s * transformCur[4];
      float tz = s * transformCur[5];

      float a1 = crx * sry * srz;
      float a2 = crx * crz * sry;
      float a3 = srx * sry;
      float a4 = tx * a1 - ty * a2 - tz * a3;
      float a5 = srx * srz;
      float a6 = crz * srx;
      float a7 = ty * a6 - tz * crx - tx * a5;
      float a8 = crx * cry * srz;
      float a9 = crx * cry * crz;
      float a10 = cry * srx;
      float a11 = tz * a10 + ty * a9 - tx * a8;

      float b1 = -crz * sry - cry * srx * srz;
      float b2 = cry * crz * srx - sry * srz;
      float b3 = crx * cry;
      float b4 = tx * -b1 + ty * -b2 + tz * b3;
      float b5 = cry * crz - srx * sry * srz;
      float b6 = cry * srz + crz * srx * sry;
      float b7 = crx * sry;
      float b8 = tz * b7 - ty * b6 - tx * b5;

      float c1 = -b6;
      float c2 = b5;
      float c3 = tx * b6 - ty * b5;
      float c4 = -crx * crz;
      float c5 = crx * srz;
      float c6 = ty * c5 + tx * -c4;
      float c7 = b2;
      float c8 = -b1;
      float c9 = tx * -b2 - ty * -b1;

      float arx = (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x + (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y + (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

      float ary = (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x + (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

      float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x + (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y + (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

      float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

      float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

      float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

      arx = s * arx;
      ary = s * ary;
      arz = s * arz;
      atx = s * atx;
      aty = s * aty;
      atz = s * atz;

      float d2 = coeff.intensity;

      matA.at<float>(i, 0) = arx;
      matA.at<float>(i, 1) = ary;
      matA.at<float>(i, 2) = arz;
      matA.at<float>(i, 3) = atx;
      matA.at<float>(i, 4) = aty;
      matA.at<float>(i, 5) = atz;
      matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0)
    {
      cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate = false;
      float eignThre[6] = {10, 10, 10, 10, 10, 10};
      for (int i = 5; i >= 0; i--)
      {
        if (matE.at<float>(0, i) < eignThre[i])
        {
          for (int j = 0; j < 6; j++)
          {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        }
        else
        {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
      cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformCur[0] += matX.at<float>(0, 0);
    transformCur[1] += matX.at<float>(1, 0);
    transformCur[2] += matX.at<float>(2, 0);
    transformCur[3] += matX.at<float>(3, 0);
    transformCur[4] += matX.at<float>(4, 0);
    transformCur[5] += matX.at<float>(5, 0);

    for (int i = 0; i < 6; i++)
    {
      if (isnan(transformCur[i]))
        transformCur[i] = 0;
    }

    float deltaR = sqrt(
        pow(rad2deg(matX.at<float>(0, 0)), 2) +
        pow(rad2deg(matX.at<float>(1, 0)), 2) +
        pow(rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(3, 0) * 100, 2) +
        pow(matX.at<float>(4, 0) * 100, 2) +
        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1)
    {
      return false;
    }
    return true;
  }

  // checkSystemInitialization() *********************************************
  // 如过系统尚未初始化，则初始化
  void checkSystemInitialization()
  {
    // 1> 将第一帧提取的lessSharp点集、lessFlat点集保存到上一帧数据中（laserCloudCornerLast，laserCloudSurfLast）
    pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    // 2> 并使用此时的上一帧的lessSharp点集和lessFlat点集构建kd-tree（kdtreeCornerLast，kdtreeSurfLast），用于最近邻搜索
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();

    // 3> 将上一帧（即当前帧）的lessSharp点集和lessFlat点集转换为ROS消息后发布给laserMapping，
    // 话题分别为："/laser_cloud_corner_last"，"/laser_cloud_surf_last"，坐标系为"/camera"
    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
    laserCloudCornerLast2.header.frame_id = "/camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
    laserCloudSurfLast2.header.frame_id = "/camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    // MLB添加:
    // 将第一帧接收的视觉特征点保存到前一帧数据（visualCloudLast），并发布到话题
    laserCloudTemp = visualCloud;
    visualCloud = visualCloudLast;
    visualCloudLast = laserCloudTemp;

    sensor_msgs::PointCloud2 visualCloudLast2;
    pcl::toROSMsg(*visualCloudLast, visualCloudLast2);
    visualCloudLast2.header.stamp = cloudHeader.stamp;
    visualCloudLast2.header.frame_id = "/camera";
    pubVisualCloudLast.publish(visualCloudLast2);

    transformSum[0] += imuPitchStart;
    transformSum[2] += imuRollStart;

    systemInitedLM = true;
  }

  void updateTransformation()
  {
    // 方案一: 视觉特征点+sharp特征点+flat特征点的紧耦合里程计
    // 如果前一帧lessSharp点和lessFlat点数目太少，则不进行当前帧的位姿估计，return结束本函数
    if (laserCloudCornerLast->points.size() < 10 || laserCloudSurfLast->points.size() < 100)
      return;

    // 优化迭代
    for (int iterCount1 = 0; iterCount1 < 25; iterCount1++)
    {
      laserCloudOri->clear();
      coeffSel->clear();
 
      findCorrespondingVisual(iterCount1);
      int num1 = laserCloudOri->points.size();
      ROS_INFO("The number of visual correspondence = %d", num1);
      findCorrespondingSurfFeatures(iterCount1);
      int num2 = laserCloudOri->points.size() - num1;
      ROS_INFO("The number of flat correspondence = %d", num2);
      findCorrespondingCornerFeatures(iterCount1);
      int num3 = laserCloudOri->points.size() - num2;
      ROS_INFO("The number of corner correspondence = %d", num3);
      
      if (laserCloudOri->points.size() < 10)
        continue;

      if (calculateTransformation(iterCount1) == false)
      {
        ROS_INFO("Optimization succeed at iteration: %d", iterCount1);
        break;
      }
          
    }
    
    // 方案二: 先使用flat特征点求解rz, rx, ty；再使用sharp特征点求解tz, tx, ry
    // for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) // iterCount1为迭代次数
    // {
    //   laserCloudOri->clear();
    //   coeffSel->clear();

    //   findCorrespondingSurfFeatures(iterCount1); // iterCount1用于控制每迭代5次，重新查找一次最近邻点
    //   if (laserCloudOri->points.size() < 10)
    //     continue;

    //   if (calculateTransformationSurf(iterCount1) == false)
    //     break;
    // }

    // for (int iterCount2 = 0; iterCount2 < 25; iterCount2++)
    // {
    //   laserCloudOri->clear();
    //   coeffSel->clear();

    //   findCorrespondingCornerFeatures(iterCount2);

    //   if (laserCloudOri->points.size() < 10)
    //     continue;

    //   if (calculateTransformationCorner(iterCount2) == false)
    //     break;
    // }
  }

  // integrateTransformation() *******************************
  // 与原loam代码相同
  // 功能: 利用优化求得的当前帧IMU旋转变换补偿值、匀速运动平移量，和IMU测量得到的当前帧IMU旋转变换、当前帧加速度平移量，
  // 累加得到当前帧结束点时刻的loam实时坐标系的全局位姿，即transformSum。
  void integrateTransformation()
  {
    float rx, ry, rz, tx, ty, tz; // rx, ry, rz是欧拉角；tx, ty, tz是平移量

    AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                       -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz); // output: rx, ry, rz

    // 绕z轴转
    float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX) - sin(rz) * (transformCur[4] - imuShiftFromStartY);
    float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX) + cos(rz) * (transformCur[4] - imuShiftFromStartY);
    float z1 = transformCur[5] - imuShiftFromStartZ;
    // 绕x轴转
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;
    // 绕y轴转，加上负号后（反向），累加到全局位移上
    tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
    ty = transformSum[4] - y2;
    tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

    // 将当前帧点云的IMU旋转变换，累积到当前帧结束点时刻的全局旋转变换
    PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart,
                      imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

    transformSum[0] = rx;
    transformSum[1] = ry;
    transformSum[2] = rz;
    transformSum[3] = tx;
    transformSum[4] = ty;
    transformSum[5] = tz;
  }

  // publishOdometry() ************************************
  // 功能: 
  // 发布世界坐标系下单纯由里程计累加得到的当前帧位姿估计结果transformSum，到里程计话题"/laser_odom_to_init"，并发布其tf消息，
  // 二者父坐标系和子坐标系分别为"/camera_init"（世界坐标系）和"/laser_odom"
  void publishOdometry()
  {
    // 发布transformSum到里程计消息
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

    laserOdometry.header.stamp = cloudHeader.stamp; // 时间戳为segmentedCloud的时间戳

    laserOdometry.pose.pose.orientation.x = -geoQuat.y;
    laserOdometry.pose.pose.orientation.y = -geoQuat.z;
    laserOdometry.pose.pose.orientation.z = geoQuat.x;
    laserOdometry.pose.pose.orientation.w = geoQuat.w;

    laserOdometry.pose.pose.position.x = transformSum[3];
    laserOdometry.pose.pose.position.y = transformSum[4];
    laserOdometry.pose.pose.position.z = transformSum[5];

    pubLaserOdometry.publish(laserOdometry);

    // 发布tf
    laserOdometryTrans.stamp_ = cloudHeader.stamp; // 视间戳为segmentedCloud的时间戳
    laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
    tfBroadcaster.sendTransform(laserOdometryTrans);
  }

  // adjustOutlierCloud() ***********************************
  // 将点云outlierCloud从velodyne坐标轴定义变换到loam坐标轴定义
  void adjustOutlierCloud()
  {
    PointType point;
    int cloudSize = outlierCloud->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
      point.x = outlierCloud->points[i].y;
      point.y = outlierCloud->points[i].z;
      point.z = outlierCloud->points[i].x;
      point.intensity = outlierCloud->points[i].intensity;
      outlierCloud->points[i] = point;
    }
  }

  // publishCloudsLast() ************************************
  void publishCloudsLast()
  {

    updateImuRollPitchYawStartSinCos();

    // 将当前帧的lessSharp点集和lessFlat点集投影到当前帧结束点时刻的loam实时坐标系
    int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++)
    {
      TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
    }

    int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
    for (int i = 0; i < surfPointsLessFlatNum; i++)
    {
      TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
    }

    // 将投影到当前帧结束点时刻的lessSharp点集和lessFlat点集保存到上一帧的lessSharp点集和lessFlat点集，用于下一帧点云的特征匹配
    pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();

    // 点足够多就构建kd-tree，否则弃用此帧，沿用上一帧数据的kd-tree
    // 此处感觉不太对？？？
    if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
    {
      kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
      kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
    }

    // MLB添加: 保存当前帧视觉特征点
    laserCloudTemp = visualCloud;
    visualCloud = visualCloudLast;
    visualCloudLast = laserCloudTemp;

    // 2> 每间隔一帧点云，发布一次已投影到当前帧结束点时刻的loam实时坐标系下的，当前帧的lessSharp点集
    frameCount++;
    if (frameCount >= skipFrameNum + 1)
    {
      frameCount = 0;

      adjustOutlierCloud();
      sensor_msgs::PointCloud2 outlierCloudLast2;
      pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
      outlierCloudLast2.header.stamp = cloudHeader.stamp;
      outlierCloudLast2.header.frame_id = "/camera";
      pubOutlierCloudLast.publish(outlierCloudLast2);

      sensor_msgs::PointCloud2 laserCloudCornerLast2;
      pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
      laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
      laserCloudCornerLast2.header.frame_id = "/camera";
      pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

      sensor_msgs::PointCloud2 laserCloudSurfLast2;
      pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
      laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
      laserCloudSurfLast2.header.frame_id = "/camera";
      pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
    }
  }

  // runFeatureAssociation() ******************************************
  void runFeatureAssociation()
  {
    // 1. 判断如果3个点云消息接收成功，且时间戳一致，则将3个接收标志重置为false；否则return结束；
    if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
        std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05 &&
        std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05 )
        // &&std::abs(timeNewVisualCloud - timeNewSegmentedCloud) < 0.05 && newVisualCloud)  // 新增视觉项
    {
      newSegmentedCloud = false;
      newSegmentedCloudInfo = false;
      newOutlierCloud = false;
      newVisualCloud = false;
      // ROS_INFO("\033[1;32m---->\033[0m MLB Tested!!!");
    }
    else
    {
      return;
    }

    // 2. Feature Extraction
    adjustDistortion();

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishCloud(); // cloud for visualization

    // 3. Feature Association
    // (1) 如果系统还未初始化(systemInitedLM=false)，则调用函数checkSystemInitialization()，并return结束本方法；
    if (!systemInitedLM)
    {
      checkSystemInitialization();
      return;
    }

    // updateInitialGuess();  // 没有IMU数据时，直接使用前一帧位姿估计为当前帧位姿估计的初值，效果更好！！！

    updateTransformation();

    integrateTransformation();

    publishOdometry();

    publishCloudsLast(); // cloud to mapOptimization
  }

  // MLB添加: *****************************************
  void viualCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn)
  {
    timeNewVisualCloud = msgIn->header.stamp.toSec();

    visualCloud->clear();
    pcl::fromROSMsg(*msgIn, *visualCloud); // 其intensity通道为特征点id

    newVisualCloud = true;

    PointType point;
    for (int i = 0; i < visualCloud->points.size(); i++)
    {
      point.intensity = visualCloud->points[i].intensity;
      point.x = visualCloud->points[i].y;
      point.y = visualCloud->points[i].z;
      point.z = visualCloud->points[i].x;

      visualCloud->points[i] = point;
    }
  }

  // findCorrespondingVisual() ************
  // 功能: 从前一帧视觉特征点集中查找当前帧视觉特征点集的匹配点，将找到有效特征匹配的当前特征点存入laserCloudOri，将其到其对应点的单位法向量存入coeffSel。
  void findCorrespondingVisual(int iterCount)
  {
    PointType pointBef, pointCur, pointLast;
    for (size_t i = 0; i < visualCloud->points.size(); i++)
    {
      for (size_t j = 0; j< visualCloudLast->points.size(); j++)
      {
        if (visualCloud->points[i].intensity == visualCloudLast->points[j].intensity) // intensity通道为特征点id
        {
          pointBef = visualCloud->points[i];
          pointBef.intensity = 0.1;
          TransformToStart(&pointBef, &pointCur, 1);
          
          pointLast = visualCloudLast->points[j];

          float x1 = pointCur.x;
          float y1 = pointCur.y;
          float z1 = pointCur.z;

          float x0 = pointLast.x;
          float y0 = pointLast.y;
          float z0 = pointLast.z;

          float distance = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) + (z1-z0)*(z1-z0));
         
          float nx = (x1-x0)/distance;
          float ny = (y1-y0)/distance;
          float nz = (z1-z0)/distance;

          float s=1;
          if (iterCount >= 5)
          {
            s = 1 - 1.8 * fabs(distance);
          }

          if (s > 0.1 && distance > 0 && distance < 0.75)  // MLB添加条件: distance < 1.5，去除外点
          {
            coeff.x = s * nx;
            coeff.y = s * ny;
            coeff.z = s * nz;
            coeff.intensity = s * distance;

            coeffSel->push_back(coeff);

            laserCloudOri->push_back(pointBef);
            // ROS_INFO("MLB Test!");
          }

          break; // 不再继续查找
        }

      }
    }
  }
};

int main(int argc, char **argv)
{
  // 1. 初始化节点和句柄
  ros::init(argc, argv, "lego_loam");

  // 2. 使用ROS_INFO()输出提示
  ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

  // 3. 声明FeatureAssociation类的对象FA
  FeatureAssociation FA;

  // 4. 在while()循环中调用消息回调函数及类函数runFeatureAssociation()
  ros::Rate rate(200); // 设置ros::Rate为200Hz
  while (ros::ok())
  {
    ros::spinOnce();

    FA.runFeatureAssociation();

    rate.sleep();
  }

  ros::spin(); // 可省
  return 0;
}
