#include "utility.h"
#include "CameraLidar.h"
#include "feature_tracker.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>

//using namespace legoloam;

class ImageProjection
{
private:
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud;

  ros::Publisher pubFullCloud;
  ros::Publisher pubFullInfoCloud;

  ros::Publisher pubGroundCloud;
  ros::Publisher pubSegmentedCloud;
  ros::Publisher pubSegmentedCloudPure;
  ros::Publisher pubSegmentedCloudInfo;
  ros::Publisher pubOutlierCloud;

  // 点云
  pcl::PointCloud<PointType>::Ptr laserCloudIn; 

  pcl::PointCloud<PointType>::Ptr fullCloud;    
  pcl::PointCloud<PointType>::Ptr fullInfoCloud;

  pcl::PointCloud<PointType>::Ptr groundCloud;    
  pcl::PointCloud<PointType>::Ptr segmentedCloud;     
  pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
  pcl::PointCloud<PointType>::Ptr outlierCloud;      

  PointType nanPoint; 

  // 当前帧image及其标签矩阵，rangeMat存储对应位置的深度值，groundMat标记是否是地面
  cv::Mat rangeMat; 
  cv::Mat labelMat; 
  int labelCount;

  // 当前帧点云的初始点水平角度（约为-pi/2），结束点水平角度（约为3/2pi）
  float startOrientation; 
  float endOrientation;   

  // 输入的当前帧原始点云消息的header（包含了时间戳和坐标系）
  std_msgs::Header cloudHeader;

  cloud_msgs::cloud_info segMsg; // info of segmented cloud

  // 用于labelComponents()函数的点云分割的中间变
  std::vector<std::pair<uint8_t, uint8_t>> neighborIterator; 
  uint16_t *allPushedIndX;                                   
  uint16_t *allPushedIndY;

  uint16_t *queueIndX; 
  uint16_t *queueIndY;

  // MLB添加: *************************************************************
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorClusters;
  ros::Publisher pubColorClusters;

  pcl::PointCloud<PointType>::Ptr lidar2Normalized;   // 归一化特征点（z=1）
  pcl::PointCloud<PointType>::Ptr lidar2NormalizedDS;
  pcl::PointCloud<PointType>::Ptr lidar2Normalized3D;
  pcl::PointCloud<PointType>::Ptr lidar2Normalized3DDS;
  pcl::PointCloud<pcl::PointXY>::Ptr lidar2Normalized2D; 
  pcl::VoxelGrid<PointType> voxelDS;
  pcl::KdTreeFLANN<pcl::PointXY> kdtree;

  vector<int> lidar2NormalizedLabel;               
  ros::Publisher pubLidar2Normalized;

  pcl::PointCloud<PointType>::Ptr depthEstimationCloud;   // 当前帧视觉特征点深度估计利用的归一化点云，用于Rviz显示
  pcl::PointCloud<PointType>::Ptr depthEstimation3D;      // 当前帧视觉特征点深度估计的计算结果点云，位于激光雷达坐标系下
  ros::Publisher pubDepthEstimationCloud;
  ros::Publisher pubDepthEstimation3D;
  
  pcl::PointCloud<PointType>::Ptr clusterCloud;

  legoloam::CameraLidar camera;

  bool newLaserCloud;
  double timeNewLaserCloud;
  bool newImage;
  double timeNewImage;

  pcl::PointCloud<PointType>::Ptr featurePoints;  // 当前帧跟踪及新增的视觉特征点
  ros::Publisher pubFeaturePoints;

  // 特征跟踪部分的全局变量 ****************************************
  vector<uchar> r_status; // 没有用？
  vector<float> r_err;

  // 输入图片消息的缓存队列
  queue<sensor_msgs::ImageConstPtr> img_buf; // 输入图片的缓存队列

  // ROS话题发布器
  ros::Publisher pub_img;
  ros::Publisher pub_match;
  ros::Publisher pub_restart;
  ros::Subscriber sub_img;

  // FeatureTracker类的实例。每个相机都有一个FeatureTracker实例，即trackerData[i]。
  FeatureTracker trackerData[NUM_OF_CAM];

  // 消息回调函数img_callback()中用到的全局控制变量。
  bool first_image_flag = true; // 当前帧是否是本次连续跟踪的第一帧
  int pub_count = 1;            // 本次连续跟踪已发布帧的次数
  double first_image_time;      // 本次连续跟踪的第一帧图片的时间戳
  double last_image_time = 0;   // 当前帧的上一帧图片的时间戳
  bool init_pub = 0;

public:
  // Function1: ImageProjection() *****************************************************************
  // 构造函数
  ImageProjection() : nh("~")
  {
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);

    pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
    pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

    pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
    pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
    pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
    pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

    // 为无效点nanPoint赋值NaN，intensity通道为-1，用于初始化点云fullCloud
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    // 调用函数allocateMemory()
    allocateMemory();

    // 调用函数resetParameters()
    resetParameters();

    // MLB添加：
    pubColorClusters = nh.advertise<sensor_msgs::PointCloud2>("/color_clusters", 1);
    pubLidar2Normalized = nh.advertise<sensor_msgs::PointCloud2>("/lidar_2_normalized", 1);
    pubFeaturePoints = nh.advertise<sensor_msgs::PointCloud2>("/feature", 1000);
    pubDepthEstimationCloud = nh.advertise<sensor_msgs::PointCloud2>("/depth_estimation_cloud", 1);
    pubDepthEstimation3D = nh.advertise<sensor_msgs::PointCloud2>("/depth_estimation_3d", 1);
    //pubFeatureCloud = nh.advertise<sensor_msgs::PointCloud2>("/feature_cloud", 1);
    //pubLidar2Normalized2 = nh.advertise<sensor_msgs::PointCloud2>("/lidar_2_normalized2", 1);
    //subFeature = nh.subscribe<sensor_msgs::PointCloud2>("/feature_tracker/feature", 1, &ImageProjection::featureHandler, this);

    // 视觉跟踪部分初始化
    featureTrackerInit();
    // 订阅图片话题IMAGE_TOPIC(即"/cam0/image_raw")，执行回调函数img_callback()。
    sub_img = nh.subscribe<sensor_msgs::Image>("/kitti/camera_gray_left/image_raw", 100, &ImageProjection::img_callback, this);
    // 发布话题"/feature_tracker/feature"，消息内容为feature_points，为当前帧的归一化特征点（x,y,z=1），给后端优化用
    //pub_img = nh.advertise<sensor_msgs::PointCloud2>("/feature", 1000);  // 不加"/"则该话题位于当前节点命名空间/feature_tracker下
    // 发布话题"/feature_tracker/feature_img"，消息内容为当前帧图片实例ptr，其按照跟踪次数将特征点标记为红色（次数大）和蓝色，给RVIZ用和调试用
    pub_match = nh.advertise<sensor_msgs::Image>("/feature_img", 1000);
    // 发布话题"/feature_tracker/restart"
    pub_restart = nh.advertise<std_msgs::Bool>("/restart", 1000);
  }

  // Function2： allocateMemory() **********************************************************
  void allocateMemory()
  {
    // 1> 为7个点云变量分配新的动态内存
    laserCloudIn.reset(new pcl::PointCloud<PointType>());

    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullInfoCloud.reset(new pcl::PointCloud<PointType>());

    groundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    // MLB添加：
    colorClusters.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    lidar2Normalized.reset(new pcl::PointCloud<PointType>());
    lidar2NormalizedDS.reset(new pcl::PointCloud<PointType>());
    lidar2Normalized2D.reset(new pcl::PointCloud<pcl::PointXY>());
    lidar2Normalized3D.reset(new pcl::PointCloud<PointType>());
    lidar2Normalized3DDS.reset(new pcl::PointCloud<PointType>());
    clusterCloud.reset(new pcl::PointCloud<PointType>());
    
    //voxelDS.setLeafSize (0.01f, 0.01f, 0.01f);

    featurePoints.reset(new pcl::PointCloud<PointType>());
    depthEstimationCloud.reset(new pcl::PointCloud<PointType>());
    depthEstimation3D.reset(new pcl::PointCloud<PointType>());

    newLaserCloud = false;
    newImage = false;

    //featureCloud.reset(new pcl::PointCloud<PointType>());
    //lidar2Normalized2.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    // 2> 使用PCL的resize()函数，将点云fullCloud和fullInfoCloud的大小设置为N_SCAN * Horizon_SCAN
    fullCloud->points.resize(N_SCAN * Horizon_SCAN);
    fullInfoCloud->points.resize(N_SCAN * Horizon_SCAN);

    // 3> 为消息变量segMsg赋初值
    segMsg.startRingIndex.assign(N_SCAN, 0);
    segMsg.endRingIndex.assign(N_SCAN, 0);
    segMsg.segmentedCloudGroundFlag.assign(N_SCAN * Horizon_SCAN, false);
    segMsg.segmentedCloudColInd.assign(N_SCAN * Horizon_SCAN, 0);
    segMsg.segmentedCloudRange.assign(N_SCAN * Horizon_SCAN, 0);

    // 4> 为用于labelComponents()函数的点云分割的中间变量赋初值
    // 为迭代器向量neighborIterator赋值，其包含了在image中当前点的上下左右四个点相对当前点的索引差值
    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    // 为广度搜索时的已探索队列和待探索队列分配动态内存，大小为N_SCAN * Horizon_SCAN
    allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
  }

  // Function3: resetParameters() ****************************************************************
  void resetParameters()
  {
    // 1> 使用PCL的clear()方法清除除fullCloud和fullInfoCloud的另外5个点云内存
    //laserCloudIn->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();

    // MLB添加：
    colorClusters->clear();
    lidar2Normalized->clear();
    lidar2NormalizedDS->clear();
    lidar2Normalized3D->clear();
    lidar2Normalized3DDS->clear();
    clusterCloud->clear();
    depthEstimationCloud->clear();
    depthEstimation3D->clear();

    //featurePoints->clear();
    //featureCloud->clear();
    //lidar2Normalized2->clear();

    // 2> 初始化3个矩阵的元素，rangeMat(=FLT_MAX)，groundMat(=0)，和labelMat(=0)
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX)); // rangeMat各元素值初始化为FLT_MAX（浮点数最大值）
    // groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));       // groundMat各元素值初始化为0
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1)); // labelMat各元素值初始化为0

    // 3> 初始化当前分割的cluster的标签labelCount为1
    labelCount = 1;

    // 4> 初始化点云fullCloud和fullInfoCloud的所有点为自定义的NaN无效点，其xyz通道值为quiet_NaN，intensity通道置为-1
    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
  }

  // Function4: ~ImageProjection() *******************************************************
  // 析构函数
  ~ImageProjection() {}

  // Function6: cloudHandler() *****************************************************************
  void runImageProjection()
  {
    // 1. Convert ros message to pcl point cloud
    //copyPointCloud(laserCloudMsg);


    // 1. 判断如果当前帧点云消息和视觉归一化特征点消息接收成功，且时间戳一致，则将2个接收标志重置为false；否则return结束；
    if (newLaserCloud && newImage)
    {
      newLaserCloud = false;
      newImage = false;
      ROS_INFO("\033[1;32m---->\033[0m MLB Tested!!!");
    }
    else
    {
      return;
    }

    // 2. Start and end angle of a scan
    findStartEndAngle();

    // 3. Range image projection
    projectPointCloud();

    // 4. Mark ground points
    groundRemoval();

    // 5. Point cloud segmentation
    cloudSegmentation();

    // MLB added
    projectToCameraNormalized();
    featureDepthEstimation(); // 调用此函数会导致程序异常退出，不调用时没事。

    // 6. Publish all clouds
    publishCloud();

    // 7. Reset parameters for next iteration
    resetParameters();
  }

  // Function7: findStartEndAngle() *****************************************************************
  // 功能: 计算当前帧点云的起始点水平角度、结束点水平角度及二者差值，分别保存至类变量segMsg.startOrientation、segMsg.endOrientation和segMsg.orientationDiff
  void findStartEndAngle()
  {
    segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    segMsg.endOrientation = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                   laserCloudIn->points[laserCloudIn->points.size() - 2].x) +
                            2 * M_PI;
    if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI)
    {
      segMsg.endOrientation -= 2 * M_PI;
    }
    else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
      segMsg.endOrientation += 2 * M_PI;

    // 当前帧点云的起始点与结束点的水平角度差值
    segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
  }

  // Function8: projectPointCloud() *************************************************************
  void projectPointCloud()
  { // range image projection

    float verticalAngle, horizonAngle, range;   // 当前点竖直角度，水平角度，深度
    size_t rowIdn, columnIdn, index, cloudSize; // 当前点行索引，列索引，总索引，点云大小
    PointType thisPoint;                        // 当前遍历点

    cloudSize = laserCloudIn->points.size(); // 当前帧点云大小

    // 遍历当前帧原始点云laserCloudIn的所有点，计算当前遍历点的行索引、列索引、深度值
    for (size_t i = 0; i < cloudSize; ++i)
    {
      thisPoint.x = laserCloudIn->points[i].x;
      thisPoint.y = laserCloudIn->points[i].y;
      thisPoint.z = laserCloudIn->points[i].z;

      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI; // 计算当前遍历点的竖直角度
      rowIdn = (verticalAngle + ang_bottom) / ang_res_y;                                                            // rowIdn是行索引，对应激光雷达线数从下到上为0,1,...,15
      if (rowIdn < 0 || rowIdn >= N_SCAN)                                                                           // 判断行索引是否合法
        continue;

      horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

      columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2; // columnIdn是列索引 // round()函数计算四舍五入到最邻近的整数
      if (columnIdn >= Horizon_SCAN)
        columnIdn -= Horizon_SCAN;
      if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        continue;


      range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      if (range < 0.1)
        continue;

      rangeMat.at<float>(rowIdn, columnIdn) = range;
      thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
      index = columnIdn + rowIdn * Horizon_SCAN;

      fullCloud->points[index] = thisPoint;

      fullInfoCloud->points[index] = thisPoint;
      fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
    }
  }

  // Functioin9: groundRemoval() ******************************************************
  void groundRemoval()
  {
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;

    // 1> 遍历当前image所有点(i,j)，如果该点rangeMat值为初始值FLT_MAX，则表示该点为无效NaN点，则将该点的labelMat值置为-2
    for (size_t i = 0; i < N_SCAN; ++i)
    {
      for (size_t j = 0; j < Horizon_SCAN; ++j)
      {
        if (rangeMat.at<float>(i, j) == FLT_MAX)
        {
          labelMat.at<int>(i, j) = -2;
        }
      }
    }

    // 2> 遍历当前image的所有column，并对每一个column从下到上遍历到行索引groundScanInd-1=6(即-3度线，即不超过0度):
    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
      for (size_t i = 0; i < groundScanInd; ++i)
      {
        lowerInd = j + (i)*Horizon_SCAN;
        upperInd = j + (i + 1) * Horizon_SCAN;

        if (labelMat.at<int>(i, j) == -2 || labelMat.at<int>(i + 1, j) == -2)
        {
          continue;
        }

        diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
        diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
        diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

        angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

        if (abs(angle - sensorMountAngle) <= 10)
        {
          labelMat.at<int>(i, j) = 0; // 地面点，labelMat[]=0
          labelMat.at<int>(i + 1, j) = 0;
        }
      }
    }

    if (pubGroundCloud.getNumSubscribers() != 0) 
    {
      for (size_t i = 0; i <= groundScanInd; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (labelMat.at<int>(i, j) == 0)
            groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
        }
      }
    }
  }

  // Function10: cloudSegmentation() *************************************************
  void cloudSegmentation()
  {
    for (size_t i = 0; i < N_SCAN; ++i)
      for (size_t j = 0; j < Horizon_SCAN; ++j)
        if (labelMat.at<int>(i, j) == -1)
          labelComponents(i, j);

    int sizeOfSegCloud = 0; // 所有分割点云合集的点数

    // 遍历当前image中所有点
    PointType point;
    for (size_t i = 0; i < N_SCAN; ++i) // extract segmented cloud for lidar odometry
    {
      // segMsg.startRingIndex[i]表示在第i条激光线中计算点曲率时的开始点在点云segmentedCloud中的索引
      segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5; // =4 ?

      for (size_t j = 0; j < Horizon_SCAN; ++j)
      {
        // 如果当前点是已分割点（labelMat>0），或地面点（labelMat=0）
        if (labelMat.at<int>(i, j) >= 0)
        {
          // 1) 如果当前点labelMat=999999，且行索引>groundScanInd，则为非地面无效分割点，则对非地面无效分割点进行5倍降采样后存入点云outlierCloud
          if (labelMat.at<int>(i, j) == 999999)
          {
            // 如果当前点行索引大于groundScanInd=7，且其列索引为5的倍数，则将当前点加入点云outlierCloud（相当于对非地面的无效分割点降采样）
            if (i > groundScanInd && j % 5 == 0)
            {
              outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
              continue;
            }
            else
            {
              continue;
            }
          }

          // 2) 如果当前点labelMat>=0且<999999，则为地面点或有效分割点，
          // 则令当前点intensity通道为labelMat值，存入点云clusterCloud，用于视觉特征点深度关联
          if (labelMat.at<int>(i, j) >= 0 && labelMat.at<int>(i, j) < 999999)
          {
            point = fullCloud->points[j + i * Horizon_SCAN];
            point.intensity = labelMat.at<int>(i, j);
            clusterCloud->push_back(point);
          }

          // 3) 如果当前点是地面点（labelMat=0），，则continue跳过4/5的主体地面点；
          // 对于剩下的点（包括剩下的1/5地面点和全部有效分割点），在点云消息变量segMsg中的对应数组中依次记录当前分割点的是否地面标志（之后不作为角点）、
          // 列索引（用于计算相对时间）和深度值，并将当前分割点从fullCloud中提取加入点云segmentedCloud；
          if (labelMat.at<int>(i, j) == 0)
          {
            if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
              continue;
          }

          // mark ground points so they will not be considered as edge features later
          segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (labelMat.at<int>(i, j) == 0);

          // mark the points' column index for marking occlusion later
          segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;

          // save range info
          segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);

          // save segmented cloud
          segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);

          // size of seg cloud
          ++sizeOfSegCloud; // 所有分割点云合集的点数+1
        }
      }

      // segMsg.endRingIndex[i]表示在第i条激光线中计算点曲率时的最后一点在点云segmentedCloud中的索引
      segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
    }

    if (pubSegmentedCloudPure.getNumSubscribers() != 0) // extract segmented cloud for visualization
    {
      for (size_t i = 0; i < N_SCAN; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999)
          {
            segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
            segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
          }
        }
      }
    }

    // MLB添加 **************************
    int clusterNumber = 0;
    if (pubColorClusters.getNumSubscribers() != 0)
    {
      // 1> 遍历image，得到有效cluster的数目
      for (size_t i = 0; i < N_SCAN; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (labelMat.at<int>(i, j) > clusterNumber && labelMat.at<int>(i, j) >= 0 && labelMat.at<int>(i, j) < 999999)
            clusterNumber = labelMat.at<int>(i, j); // 不包括地面点的有效cluster的数目
        }
      }
      clusterNumber++;

      // 2> 声明存储所有cluster的点云向量，并为其分配动态内存
      vector<pcl::PointCloud<PointType>::Ptr> clusters(clusterNumber);
      for (int i = 0; i < clusterNumber; i++)
        clusters[i].reset(new pcl::PointCloud<PointType>());
      cluster_t sortIndex[clusterNumber]; // 用于排序，按照每个cluster的点数从大到小排序

      // 3> 遍历image所有点，按照其labelMat标签值将其存入点云向量clusters
      for (size_t i = 0; i < N_SCAN; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (labelMat.at<int>(i, j) >= 0 && labelMat.at<int>(i, j) < 999999)
          {
            int index = labelMat.at<int>(i, j);
            clusters[index]->push_back(fullCloud->points[j + i * Horizon_SCAN]);
            sortIndex[index].ind = index; // 索引
          }
        }
      }

      // 4> 为sortIndex对应的每个cluster的大小赋值
      for (int i = 0; i < clusterNumber; i++)
        sortIndex[i].size = clusters[i]->size();

      // 5> 按照每个cluster的点数对点云集合排序
      std::sort(sortIndex, sortIndex + clusterNumber, compareClusterSize);

      // 6> 按照点数从多到少，将每个cluster着色，再存入点云colorClusters
      pcl::PointXYZRGB point2;
      int numShow = clusterNumber;
      float ratio = 0;
      // 将地面点着色为白色，存入点云colorClusters
      for (int j = 0; j < clusters[0]->size(); j++)
      {
        point2.x = clusters[0]->points[j].x;
        point2.y = clusters[0]->points[j].y;
        point2.z = clusters[0]->points[j].z;
        point2.r = 255;
        point2.g = 255;
        point2.b = 255;
        colorClusters->push_back(point2);
      }
      // 将剩余cluster按点数从大到小对应从着色，再存入点云
      for (int i = 1; i < numShow; i++)
      {
        for (int j = 0; j < clusters[i]->size(); j++)
        {
          point2.x = clusters[i]->points[j].x;
          point2.y = clusters[i]->points[j].y;
          point2.z = clusters[i]->points[j].z;
          ratio = float(i) / numShow;
          point2.r = (int)255 * ratio;
          point2.g = (int)255 * (1 - ratio);
          point2.b = 0;
          colorClusters->push_back(point2);
        }
      }
    }
  }

  // Function11: labelComponents() **************************************************
  void labelComponents(int row, int col)
  {
    // use std::queue, std::vector or std::deque will slow the program down greatly

    float d1, d2, alpha, angle;                 // 当前两点深度最大值d1, 最小值d2
    int fromIndX, fromIndY, thisIndX, thisIndY; // 当前点及上下左右中的一个邻接点
    bool lineCountFlag[N_SCAN] = {false};       // ?

    // 将当前点加入待探索队列
    queueIndX[0] = row; // 将队首元素索引赋值为当前点在image中的行索引和列索引
    queueIndY[0] = col;
    int queueSize = 1;     // 当前队列大小
    int queueStartInd = 0; // 队首元素索引
    int queueEndInd = 1;   // 队尾索引

    // 将当前点加入已探索队列
    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;

    // 若当前待探索队列不为空，则循环:
    while (queueSize > 0)
    {
      // 1> 获得当前队首元素，队列大小-1，队首元素索引+1
      fromIndX = queueIndX[queueStartInd];
      fromIndY = queueIndY[queueStartInd];
      --queueSize;
      ++queueStartInd;

      // 2> 将取出的队首元素在labelMat矩阵中的标签标记为当前的labelCount(>=1)
      labelMat.at<int>(fromIndX, fromIndY) = labelCount;

      // 3> 遍历当前队首元素在image中的上下左右四个邻接点
      // Loop through all the neighboring grids of popped grid
      for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
      {
        // 1) 当前邻接点在image中的索引
        thisIndX = fromIndX + (*iter).first;  // 行索引
        thisIndY = fromIndY + (*iter).second; // 列索引

        // 2) 判断当前邻接点行索引是否在范围内，对列索引进行首尾边界衔接，最后判断当前邻接点的labelMat值是否是-1（初始值）
        // index should be within the boundary
        if (thisIndX < 0 || thisIndX >= N_SCAN)
          continue;
        // at range image margin (left or right side)
        if (thisIndY < 0)
          thisIndY = Horizon_SCAN - 1;
        if (thisIndY >= Horizon_SCAN)
          thisIndY = 0;
        // prevent infinite loop (caused by put already examined point back)
        if (labelMat.at<int>(thisIndX, thisIndY) != -1)
          continue;

        // 3) 计算两点深度最大值和最小值，分别赋给d1和d2
        d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));
        d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));
        // 4) 如果当前两点的行索引相同，则为水平相邻；否则为竖直相邻；分别对应将激光雷达水平和竖直角分辨率的弧度值赋给alpha；
        if ((*iter).first == 0)
          alpha = segmentAlphaX; // 激光雷达水平角度分辨率的弧度值
        else
          alpha = segmentAlphaY; // 激光雷达竖直角度分辨率的弧度值

        // 5) 计算两点连线与深度较大点射线的夹角值
        angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

        // 6) 如果该夹角大于阈值segmentTheta（60°），则在labelMat矩阵中将该邻接点标记为当前点的label（即为同一个cluster），并将当前邻接点加入待探索队列和已探索队列队尾
        if (angle > segmentTheta)
        {
          // 将当前邻接点加到待探索队列队尾，队列大小+1，队尾元素索引+1
          queueIndX[queueEndInd] = thisIndX;
          queueIndY[queueEndInd] = thisIndY;
          ++queueSize;
          ++queueEndInd;

          // 将当前邻接点在labelMat矩阵中标记为当前点标签labelCount
          labelMat.at<int>(thisIndX, thisIndY) = labelCount;
          lineCountFlag[thisIndX] = true; // 将当前邻接点所在行（激光线）标记为true？

          // 将当前邻接点加入已探索队列队尾，队列大小+1
          allPushedIndX[allPushedIndSize] = thisIndX;
          allPushedIndY[allPushedIndSize] = thisIndY;
          ++allPushedIndSize;
        }
      }
    }

    // 如果当前分割得到的cluster点集大小segmentValidMinNum，则认为是可行的分割；
    bool feasibleSegment = false;
    if (allPushedIndSize >= segmentValidMinNum) // segmentMinNum
      feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidMinNum2) // >=5
    {
      // 计算当前cluster包含的激光线数目
      int lineCount = 0; // 当前cluster包含的激光线数目
      for (size_t i = 0; i < N_SCAN; ++i)
        if (lineCountFlag[i] == true)
          ++lineCount;
      // 如果当前cluster包含的激光线数目>=5，则也认为其是可行的分割（因为竖值方向角分辨率低）
      if (lineCount >= segmentValidLineNum)
        feasibleSegment = true;
    }

    if (feasibleSegment == true)
    {
      ++labelCount; // 将labelCount+1，用于标记下一个cluster
    }
    else
    { // segment is invalid, mark these points
      for (size_t i = 0; i < allPushedIndSize; ++i)
      {
        labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999; // outliers的labelMat[]=999999
      }
    }
  }

  // Function12: publishCloud() ******************************************
  void publishCloud()
  {
    // 1. 发布带有信息的cloud_info消息类型的点云segMsg，话题为"/segmented_cloud_info"
    // Publish Seg Cloud Info
    segMsg.header = cloudHeader;
    pubSegmentedCloudInfo.publish(segMsg);

    // 2. 发布点云outlierCloud，话题为"/outlier_cloud"
    sensor_msgs::PointCloud2 laserCloudTemp;

    pcl::toROSMsg(*outlierCloud, laserCloudTemp);
    laserCloudTemp.header.stamp = cloudHeader.stamp; // 时间戳
    laserCloudTemp.header.frame_id = "base_link";    // 坐标系为"base_link"
    pubOutlierCloud.publish(laserCloudTemp);

    // 3. 发布点云segmentedCloud，话题为"/segmented_cloud"
    // segmented cloud with ground
    pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
    laserCloudTemp.header.stamp = cloudHeader.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pubSegmentedCloud.publish(laserCloudTemp);

    // 4. 发布点云fullCloud，话题为"/full_cloud_projected"
    // projected full cloud
    if (pubFullCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*fullCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFullCloud.publish(laserCloudTemp);
    }

    // 5. 发布点云groundCloud，话题为"/ground_cloud"
    // original dense ground cloud
    if (pubGroundCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*groundCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubGroundCloud.publish(laserCloudTemp);
    }

    // 6. 发布点云segmentedCloudPure，话题为"/segmented_cloud_pure"
    // segmented cloud without ground
    if (pubSegmentedCloudPure.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubSegmentedCloudPure.publish(laserCloudTemp);
    }

    // 7. 发布点云fullInfoCloud，话题为"/full_cloud_info"
    // projected full cloud info
    if (pubFullInfoCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFullInfoCloud.publish(laserCloudTemp);
    }

    // MLB添加：
    if (pubColorClusters.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*colorClusters, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubColorClusters.publish(laserCloudTemp);
    }

    if (pubLidar2Normalized.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*lidar2NormalizedDS, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubLidar2Normalized.publish(laserCloudTemp);
    }

    if (pubFeaturePoints.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*featurePoints, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFeaturePoints.publish(laserCloudTemp);

      //ROS_INFO("\033[1;32m---->\033[0m MLB Tested 3!!!");
    }

    if (pubDepthEstimationCloud.getNumSubscribers() != 0 && depthEstimationCloud->points.size() > 0)
    {
      pcl::toROSMsg(*depthEstimationCloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubDepthEstimationCloud.publish(laserCloudTemp);
    }

    if (pubDepthEstimation3D.getNumSubscribers() != 0 && depthEstimation3D->points.size() > 0)
    {
      pcl::toROSMsg(*depthEstimation3D, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubDepthEstimation3D.publish(laserCloudTemp);
    }
  }

  // MLB添加
  // projectToCameraNormalized() ********************
  // 将分割点云clusterCloud投影到相机归一化平面，得到点云lidar2Normalized，其intensity通道为该点labelMat值
  void projectToCameraNormalized()
  {

    int cloudSize = clusterCloud->points.size(); // 当前帧点云大小
    lidar2NormalizedLabel.clear();

    Eigen::Vector3d p;
    PointType point;
    pcl::PointXY point2;
    for (size_t i = 0; i < cloudSize; ++i)
    {
      point = clusterCloud->points[i]; // intensity通道为该点的labelMat
      p << point.x, point.y, point.z;
      p = camera.lidarToCamera(p);

      if (p(2) < 0 || p(2) > 99) // 
        continue;

      if (sqrt(p(0)*p(0) + p(1)*p(1)) < p(2)) // sqrt(x^2+y^2) < z，即位于相机视角90°内
      {
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        lidar2Normalized3D->push_back(point);
      }
    }

    for(size_t i = 0; i < lidar2Normalized3D->points.size(); ++i)
    {
      point = lidar2Normalized3D->points[i]; 
      point.intensity = lidar2Normalized3D->points[i].intensity + lidar2Normalized3D->points[i].z/100.0;

      p << point.x, point.y, point.z;
      p = camera.cameraToNormalized(p);
      point.x = p(0);
      point.y = p(1);
      point.z = p(2);
      
      lidar2Normalized->push_back(point); 
      // lidar2NormalizedLabel.push_back((int)clusterCloud->points[i].intensity); // 当前点所属cluster标签
    }
    

    voxelDS.setInputCloud (lidar2Normalized);
    voxelDS.setLeafSize (0.007f, 0.007f, 0.007f); //voxel size: 0.01m
    voxelDS.filter (*lidar2NormalizedDS);
    ROS_INFO("Original points number is: %d", lidar2Normalized->size());
    ROS_INFO("Downsampled points number is: %d", lidar2NormalizedDS->size());

  }

  void featureDepthEstimation()
  // 特征点深度恢复
  {
    int numFeature = featurePoints->points.size();

    float xLeft[numFeature];
    float xRight[numFeature];
    float yUp[numFeature];
    float yDown[numFeature];
    float boxSize = 0.01; 
    PointType point;
    for (size_t i = 0; i < featurePoints->points.size(); ++i)
    {
      point = featurePoints->points[i];
      xLeft[i] = point.x - boxSize;
      xRight[i] = point.x + boxSize;
      yUp[i] = point.y + boxSize;
      yDown[i] = point.y - boxSize;
    }

    // 遍历激光点云，将每个点分给对应特征点的box
    vector<pcl::PointCloud<PointType>::Ptr> boxCloud(numFeature);
    for (int i = 0; i < numFeature; i++)
      boxCloud[i].reset(new pcl::PointCloud<PointType>());

    for (size_t j = 0; j < lidar2NormalizedDS->points.size(); ++j) 
    { 
      point = lidar2NormalizedDS->points[j];
      for (size_t i = 0; i < featurePoints->points.size(); ++i)
      {
        if(point.x < xRight[i] && point.x > xLeft[i] && point.y < yUp[i] && point.y > yDown[i])
        {
          boxCloud[i]->push_back(point);
        }
      }
    }

    // 遍历每个box
    for (int i = 0; i < numFeature; i++)
    {
      if (boxCloud[i]->points.size() == 0)
        continue;

      set<int> s;
      for (size_t j = 0; j < boxCloud[i]->points.size(); ++j)
        s.insert( (int)boxCloud[i]->points[j].intensity);
      int numClusters = s.size();

      // 声明存储所有cluster的点云向量
      vector<pcl::PointCloud<PointType>::Ptr> clusters(numClusters);
      int label[numClusters]; 
      for (int i = 0; i < numClusters; i++)
        clusters[i].reset(new pcl::PointCloud<PointType>()); 
      
      map<int, int> mapCluster; 
      int curLabel = -1;
      int curIndex = 0;
      for (size_t j = 0; j < boxCloud[i]->points.size(); ++j)
      {
        curLabel =  (int)boxCloud[i]->points[j].intensity;
        if (mapCluster.find(curLabel) == mapCluster.end())
        {
          mapCluster.insert(pair<int, int>(curLabel, curIndex));
          curIndex++;
        }  
      }

      // 遍历当前box，将点按照标签存入对应的cluster
      for (size_t j = 0; j < boxCloud[i]->points.size(); ++j)
      {
        curLabel =  (int)boxCloud[i]->points[j].intensity;
        curIndex = mapCluster.find(curLabel)->second;
        clusters[curIndex]->push_back(boxCloud[i]->points[j]);
      }

      // 遍历每个cluster，得到平均距离最小的cluster的索引minRangeIndex
      int minRangeIndex = 0;  
      double minRange = 200;  
      for (size_t j = 0; j < clusters.size(); ++j)
      {
        double range = 0;
        for (size_t k = 0; k < clusters[j]->points.size(); ++k) 
        {
          int z = clusters[j]->points[k].intensity;
          int x = clusters[j]->points[k].x * z;
          int y = clusters[j]->points[k].y * z;

          range += sqrt(x * x + y * y + z * z);
        }
        range = range / clusters[j]->points.size(); 
        if (range < minRange)
        {
          minRange = range;
          minRangeIndex = j;
        }
      }

      // 利用该cluster点估计该特征点位置；
      if (clusters[minRangeIndex]->points.size() >= 3)
      {
        for (size_t k = 0; k < clusters[minRangeIndex]->points.size(); ++k)
          depthEstimationCloud->points.push_back(clusters[minRangeIndex]->points[k]);
        
        PointType point1;
        PointType point2;
        PointType point3;
        int index1 = 0; 
        int index2 = 1;
        int index3 = 2; 
        int clusterSize = clusters[minRangeIndex]->points.size();
        double maxArea = 0;
        double area = 0;
        for (size_t k = 0; k < clusterSize-2; ++k)
        {
          point1 = clusters[minRangeIndex]->points[k];

          for (size_t m = k+1; m < clusterSize-1; ++m)
          {
            point2 = clusters[minRangeIndex]->points[m];

            for (size_t n = m+1; n < clusterSize; ++n)
            {

              point3 = clusters[minRangeIndex]->points[n];
              area =  0.5 * fabs((point2.x-point1.x)*(point3.y-point1.y) - (point3.x-point1.x)*(point2.y-point1.y));
              
              if (area > maxArea)
              {
                maxArea = area;
                index1 = k;
                index2 = m;
                index3 = n;
              }
            }
          }
        }

        ROS_INFO("The max area of this feature is: %f", maxArea);
        
        // 如果面积大于最小阈值，则使用该三个点来估计平面参数
        double areaThreshhold = 3e-5; // 最小面积阈值
        if (maxArea > areaThreshhold)
        {
          double intensity = clusters[minRangeIndex]->points[index1].intensity; // intensity整数部分为labelMat值，小数部分为深度z/100
          int label = (int)intensity;
          double depth1 = (intensity-label)*100;
          
          intensity = clusters[minRangeIndex]->points[index2].intensity;
          label = (int)intensity;
          double depth2 = (intensity-label)*100;

          intensity = clusters[minRangeIndex]->points[index3].intensity;
          label = (int)intensity;
          double depth3 = (intensity-label)*100;

          ROS_INFO("Depth1, 2, 3 = %f, %f, %f", depth1, depth2, depth3);
          
          double x1 = clusters[minRangeIndex]->points[index1].x * depth1;
          double y1 = clusters[minRangeIndex]->points[index1].y * depth1;
          double z1 = depth1;

          double x2 = clusters[minRangeIndex]->points[index2].x * depth2;
          double y2 = clusters[minRangeIndex]->points[index2].y * depth2;
          double z2 = depth2;

          double x3 = clusters[minRangeIndex]->points[index3].x * depth3;
          double y3 = clusters[minRangeIndex]->points[index3].y * depth3;
          double z3 = depth3;

          // 计算平面模型参数
          double a = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1); // 平面模型参数为: ax + by + cz + d =0
          double b = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1);
          double c = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
          double d = -(a * x1 + b * y1 + c * z1);
          
          // 计算视觉特征点射线与平面交点
          double xx1, yy1, zz1 = 0;
          double xx2 = featurePoints->points[i].x;
          double yy2 = featurePoints->points[i].y;
          double zz2 = 1;

          double n = fabs(d / (a * (xx2 - xx1) + b * (yy2 - yy1) + c * (zz2 - zz1)));

          double xx3 = xx1 + n * (xx2 - xx1); // (xx3,yy3,zz3)即为直线与平面交点
          double yy3 = yy1 + n * (yy2 - yy1);
          double zz3 = zz1 + n * (zz2 - zz1);

          Eigen::Vector3d p;
          p(0) = xx3;
          p(1) = yy3;
          p(2) = zz3;

          p = camera.cameraToLidar(p);

          PointType point0;
          point0.x = p(0);
          point0.y = p(1);
          point0.z = p(2);
          point0.intensity = featurePoints->points[i].intensity; // 视觉特征点id
          depthEstimation3D->points.push_back(point0);
        }

      }
    }
  }


  // 原始点云消息的订阅器subLaserCloud的回调函数
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn)
  {
    cloudHeader = msgIn->header;
    timeNewLaserCloud = msgIn->header.stamp.toSec();

    laserCloudIn->clear();

    pcl::fromROSMsg(*msgIn, *laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices); // Remove Nan points

    newLaserCloud = true;
  }

  // featureTrackerInit() ***************************************
  // 初始化视觉特征跟踪部分
  void featureTrackerInit()
  {
    readParameters(nh);

    trackerData[0].readIntrinsicParameter(CAM_NAMES[0]); // trackerData是FeatureTracker类的一维数组，CAM_NAMES是相机配置文件路径的vector
  }

  // img_callback() ***************************************************
  void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
  {
    if (first_image_flag)
    {
      first_image_flag = false;
      first_image_time = img_msg->header.stamp.toSec(); // 记录本次跟踪第一帧图像的时间戳
      last_image_time = img_msg->header.stamp.toSec();  // 记录上一帧图像的时间戳
      return;                                           // 结束函数
    }

    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
      // 输出ROS_WARN提示消息
      ROS_WARN("image discontinue! reset the feature tracker!");

      // 重置全局控制变量
      first_image_flag = true; // 令flag=true，重新开始下一次连续跟踪
      last_image_time = 0;
      pub_count = 1;

      // 发布消息到话题"/restart"
      std_msgs::Bool restart_flag; // 重启的flag
      restart_flag.data = true;
      pub_restart.publish(restart_flag); // 发布消息“已经重启”
      return;                            // 结束函数
    }

    last_image_time = img_msg->header.stamp.toSec();
    newImage = true;
    timeNewImage = img_msg->header.stamp.toSec();

    // 控制发布频率为全局变量FREQ=10Hz。若当前帧需要发布，则令发布标志PUB_THIS_FRAME = true。
    /*if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ) // - FREQ为frequency缩写，即为发布频率（次/second）
    {
      // 将发布标志（PUB_THIS_FRAME）置为true
      PUB_THIS_FRAME = true;

      // 当发布次数（pub_count）太大时，为防止溢出，更新时间间隔起始时刻，并已发布次数重置为0
      if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
      {
        first_image_time = img_msg->header.stamp.toSec();
        pub_count = 0;
      }
    }
    else
      PUB_THIS_FRAME = false;
    */
    PUB_THIS_FRAME = true;  // kitti已经时间同步为10Hz

    cv_bridge::CvImageConstPtr ptr; // ptr
    if (img_msg->encoding == "8UC1")
    {
      sensor_msgs::Image img;
      img.header = img_msg->header;
      img.height = img_msg->height;
      img.width = img_msg->width;
      img.is_bigendian = img_msg->is_bigendian; // ?
      img.step = img_msg->step;
      img.data = img_msg->data;
      img.encoding = "mono8";

      ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
      ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image; // 转换为cv::Mat格式后的图像，使用OpenCV处理

    // 调用特征点跟踪对象trackerData[0]对象的readImage()函数
    TicToc t_r; // 计时
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
      ROS_DEBUG("processing camera %d", i);

      if (i != 1 || !STEREO_TRACK)
        trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());

      else
      {
        if (EQUALIZE)
        {
          // 如果光太亮或太暗，则进行自适应直方图均衡化处理
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
          clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
        }
        else
          trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
      }
    }

    // (7) 调用特征点跟踪对象trackerData[i]的updateID()方法
    for (unsigned int i = 0;; i++)
    {
      bool completed = false;
      for (int j = 0; j < NUM_OF_CAM; j++)
        if (j != 1 || !STEREO_TRACK)             
          completed |= trackerData[j].updateID(i); 
      if (!completed)
        break;
    }

    // 将当前帧跟踪特征点(track_cnt>1)保存到点云featurePoints，intensity通道为一次连续跟踪中的特征点id；
    if (PUB_THIS_FRAME)
    {
      pub_count++;
      featurePoints->clear();

      vector<set<int>> hash_ids(NUM_OF_CAM); 
      
      for (int i = 0; i < NUM_OF_CAM; i++)
      {
        auto &un_pts = trackerData[i].cur_un_pts;       
        auto &ids = trackerData[i].ids;                  

        PointType p;
        for (unsigned int j = 0; j < ids.size(); j++)
        {
          if (trackerData[i].track_cnt[j] >= 1)  // 跟踪次数>1
          {
            int p_id = ids[j];
            hash_ids[i].insert(p_id);

            p.x = un_pts[j].x;
            p.y = un_pts[j].y;
            p.z = 1;
            p.intensity = p_id * NUM_OF_CAM + i;
            featurePoints->points.push_back(p);
            // ROS_INFO("\033[1;32m---->\033[0m MLB Tested 2!!!");
          }
        }
      }

      // 发布到话题"/feature_img";
      if (SHOW_TRACK)
      {
        ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8); 

        cv::Mat stereo_img = ptr->image; 

        for (int i = 0; i < NUM_OF_CAM; i++)
        {
          cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW); 
          cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);                  

          // 显示当前帧特征点的追踪状态，越红越好，越蓝越不行
          for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
          {
            // 当前特征点按照跟踪次数确定的红色程度
            double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);

            cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255 * len, 255 * (1 - len)), 2); // MLB 修改: 从绿到红
                                                                                                             // cv::circle函数参数: void circle(CV_IN_OUT Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0);
                                                                                                             // CvScalar是一个可以用来存放4个double数值的数组，一般用来存放像素值，通道顺序是BGR

            //draw speed line
            Vector2d tmp_cur_un_pts(trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
            Vector2d tmp_pts_velocity(trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
            Vector3d tmp_prev_un_pts;
            tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
            tmp_prev_un_pts.z() = 1;
            Vector2d tmp_prev_uv;
            trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
            cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255, 0, 0), 1, 8, 0);

          }
        }
        //cv::imshow("vis", stereo_img);
        //cv::waitKey(5);

        pub_match.publish(ptr->toImageMsg());
      }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc()); // 输出时间
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lego_loam");

  ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

  ImageProjection IP;

  ros::Rate rate(200);
  while (ros::ok())
  {
    IP.runImageProjection();

    ros::spinOnce();

    rate.sleep();
  }
  ros::spin();
  return 0;
}