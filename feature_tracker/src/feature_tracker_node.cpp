#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud.h>

#include <sensor_msgs/Imu.h>

#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h> // CvBridge is a ROS library that provides an interface between ROS and OpenCV.
#include <message_filters/subscriber.h>

#include "feature_tracker.h"  // FeatureTracker类的头文件

#define SHOW_UNDISTORTION 0   // 此处暂时无用？？？

// 全局变量 ************************************************************
vector<uchar> r_status;
vector<float> r_err;

  // 输入图片消息的缓存队列
queue<sensor_msgs::ImageConstPtr> img_buf; // 输入图片的缓存队列

  // ROS话题发布器
ros::Publisher pub_img;
ros::Publisher pub_match;
ros::Publisher pub_restart;

  // FeatureTracker类的实例。每个相机都有一个FeatureTracker实例，即trackerData[i]。
FeatureTracker trackerData[NUM_OF_CAM];

  // 消息回调函数img_callback()中用到的全局控制变量。
bool first_image_flag = true; // 当前帧是否是本次连续跟踪的第一帧
int pub_count = 1;            // 本次连续跟踪已发布帧的次数
double first_image_time;      // 本次连续跟踪的第一帧图片的时间戳
double last_image_time = 0;   // 当前帧的上一帧图片的时间戳
bool init_pub = 0;


// Function1: img_callback() *********************************************************************
/**
 * 功能:
 *  (1) 判断当前帧图片是否是本次连续跟踪的第一帧。若是，则更新时间戳并return结束本函数，等待下一帧图片输入。
 *  (2) 判断如果当前帧与上一帧时间间隔超过1s，或者当前帧时间早于上一帧时间，则跟踪异常，则重置全局控制变量，
 *      发布消息到话题"/feature_tracker/restart"，并return结束本函数。
 *  (3) 更新前一帧时间戳（last_image_time）为当前帧时间戳
 *  (4) 控制发布频率为全局变量FREQ=10Hz。若当前帧需要发布，则令发布标志PUB_THIS_FRAME = true。
 *  (5) 将当前帧ROS图像消息转换为cv_bridge格式的图片指针变量（ptr），和cv::Mat格式变量（show_img）。
 *  (6) 调用特征跟踪对象trackerData[i]的readImage()函数，读取当前帧图像并进行特征点跟踪及新增处理。
 *  (7) 调用特征跟踪对象trackerData[i]的updateID()方法，更新当前帧的新提取特征点的ID（一次连续跟踪内每个特征点一个id）。
 *  (8) 发布消息: 
 *      1> 发布当前帧的归一化特征点点云(z=1)到话题 "/feature_tracker/feature"，其5个附加通道分别存储了对应特征点的id、二维像素坐标和归一化坐标速度;
 *      2> 在当前帧图片上绘制特征点的追踪状态，按跟踪次数从多到少由红到蓝分布，将图片发布到话题"/feature_tracker/feature_img";
 * @brief       ROS节点的回调函数，对新来的图像进行特征点追踪，发布
 * @Description readImage()函数对新来的图像使用光流法进行特征点跟踪
 *              追踪的特征点封装成feature_points发布到pub_img的话题下，
 *              图像封装成ptr发布在pub_match下
 * @param[in]   img_msg 当前输入的图像
 * @return      void
*/

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  // (1) 判断当前帧图片是否是本次连续跟踪的第一帧。若是，则更新时间戳并return结束本函数，等待下一帧图片输入。
  if (first_image_flag)
  {
    first_image_flag = false;
    first_image_time = img_msg->header.stamp.toSec(); // 记录本次跟踪第一帧图像的时间戳
    last_image_time = img_msg->header.stamp.toSec();  // 记录上一帧图像的时间戳
    return; // 结束函数
  }

  // (2) 判断如果当前帧与上一帧时间间隔超过1s，或者当前帧时间早于上一帧时间，则跟踪异常，则重置全局控制变量，
    // 发布消息到话题"/feature_tracker/restart"，并return结束本函数。
  if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
  {
    // 输出ROS_WARN提示消息
    ROS_WARN("image discontinue! reset the feature tracker!");
    
    // 重置全局控制变量
    first_image_flag = true;      // 令flag=true，重新开始下一次连续跟踪
    last_image_time = 0;
    pub_count = 1;

    // 发布消息到话题"/feature_tracker/restart"
    std_msgs::Bool restart_flag;  // 重启的flag
    restart_flag.data = true;
    pub_restart.publish(restart_flag);  // 发布消息“已经重启”
    return; // 结束函数
  }

  // (3) 更新前一帧时间戳（last_image_time）为当前帧时间戳
  last_image_time = img_msg->header.stamp.toSec();  

  // (4) 控制发布频率为全局变量FREQ=10Hz。若当前帧需要发布，则令发布标志PUB_THIS_FRAME = true。
  if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ) // - FREQ为frequency缩写，即为发布频率（次/second）
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

  // (5) 将当前帧ROS图像消息转换为cv_bridge格式的图片指针变量（ptr），和cv::Mat格式变量（show_img）。
    // img -> show_img
  cv_bridge::CvImageConstPtr ptr; // ptr
  if (img_msg->encoding == "8UC1")
  {
    // 将输入的"8UC1"格式的图片消息转化为"mono8"格式的图片消息
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian; // ?
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    // 使用cv_bridge，将"mono8"格式的ROS图片消息转化为opencv格式，赋给指针
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat show_img = ptr->image; // 转换为cv::Mat格式后的图像，使用OpenCV处理

  // (6) 调用特征点跟踪对象trackerData[i]对象的readImage()函数，读取当前帧图像并进行特征点跟踪及新增处理
  TicToc t_r; // 计时
  for (int i = 0; i < NUM_OF_CAM; i++)
  {
    ROS_DEBUG("processing camera %d", i);

    // 如果是单目，则使用FeatureTracker::readImage()函数对输入图像进行光流特征点跟踪
    if (i != 1 || !STEREO_TRACK) 
      trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        // opencv函数rowRange(int x, int y)取出原图片的第x+1至第y行，组成一张新图片

    // 双目（略）
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

#if SHOW_UNDISTORTION
    trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
  }

  // (7) 调用特征点跟踪对象trackerData[i]的updateID()方法，更新当前帧的新提取特征点的ID（一次连续跟踪内每个特征点一个id）
    // 遍历当前帧的所有特征点id，如果ids[i]=-1，即为新提取特征点，则令其id=n_id
  for (unsigned int i = 0;; i++)
  {
    bool completed = false; // 特征点id更新完毕的标志
    for (int j = 0; j < NUM_OF_CAM; j++)
      if (j != 1 || !STEREO_TRACK) // 单目
        completed |= trackerData[j].updateID(i); // "|="为C语言中的按位或运算，再赋值
    if (!completed)
      break;
  }

  // (8) 发布消息: 
    // 1> 发布当前帧的归一化特征点点云(z=1)到话题 "/feature_tracker/feature"，其5个附加通道分别存储了对应特征点的id、二维像素坐标和归一化坐标速度;
    // 2> 在当前帧图片上绘制特征点的追踪状态，按跟踪次数从多到少由红到蓝分布，将图片发布到话题"/feature_tracker/feature_img";
  if (PUB_THIS_FRAME)
  {
    // 1> 本次连续跟踪的已发布帧数+1
    pub_count++;

    // 2> 声明待发布的当前帧的归一化特征点云，及其附加5个通道（分别存储了对应特征点的id、二维像素坐标和归一化坐标速度）
    sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud); // 图像特征点构成的点云

    sensor_msgs::ChannelFloat32 id_of_point;          // 特征点id
    sensor_msgs::ChannelFloat32 u_of_point;           // 特征点的二维像素坐标
    sensor_msgs::ChannelFloat32 v_of_point;
    sensor_msgs::ChannelFloat32 velocity_x_of_point;  // 特征点的二维归一化坐标相对前一帧的速度
    sensor_msgs::ChannelFloat32 velocity_y_of_point;

    // 3> 将特征点云的header设为当前帧图片的header，坐标系设为“world”
    feature_points->header = img_msg->header;         // 特征点云的header
    feature_points->header.frame_id = "world";        // 特征点云的坐标系

    // 4> 将当前帧的二维归一化特征点加上一维z=1后，存入特征点云feature_points，同时向其5个通道中依次添加对应的id，二维像素坐标，和归一化坐标速度
    vector<set<int>> hash_ids(NUM_OF_CAM);  // set为C++容器中的集合类型，其中同一个元素只能出现一次
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
      auto &un_pts = trackerData[i].cur_un_pts; // 当前帧特征点的二维归一化坐标
      auto &cur_pts = trackerData[i].cur_pts;   // 当前帧特征点的二维像素坐标
      auto &ids = trackerData[i].ids;           // 当前帧特征点的id
      auto &pts_velocity = trackerData[i].pts_velocity; // 当前帧特征点的归一化坐标相对前一帧的速度

      // 遍历当前帧所有的特征点，将特征点id插入集合hash_ids中，将特征点的二维归一化坐标加上一维z=1后加入feature_points中
      for (unsigned int j = 0; j < ids.size(); j++) 
      {
        if (trackerData[i].track_cnt[j] > 1)
        {
          // 将当前特征点id插入集合hash_ids中
          int p_id = ids[j];
          hash_ids[i].insert(p_id); 

          // 将特征点归一化二维坐标加上一维z=1后加入feature_points中
          geometry_msgs::Point32 p;
          p.x = un_pts[j].x;
          p.y = un_pts[j].y;
          p.z = 1;
          feature_points->points.push_back(p);  

          // 以下5个vector是放入特征点云的5个channel中（整个点云数据类型为sensor_msgs::PointCloud）
          id_of_point.values.push_back(p_id * NUM_OF_CAM + i);  // 特征点id
          u_of_point.values.push_back(cur_pts[j].x);            // 特征点的二维像素坐标
          v_of_point.values.push_back(cur_pts[j].y);
          velocity_x_of_point.values.push_back(pts_velocity[j].x);  // 特征点的归一化坐标的移动速度
          velocity_y_of_point.values.push_back(pts_velocity[j].y);
        }
      }
    }
    feature_points->channels.push_back(id_of_point);
    feature_points->channels.push_back(u_of_point);
    feature_points->channels.push_back(v_of_point);
    feature_points->channels.push_back(velocity_x_of_point);
    feature_points->channels.push_back(velocity_y_of_point);
    ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());

    // 5> 发布当前帧的归一化特征点云（feature_points）到话题 "/feature_tracker/feature"
      // skip the first image; since no optical speed on frist image
    if (!init_pub) // 第一帧不发布（init_pub = 0）
    {
      init_pub = 1;
    }
    else
      pub_img.publish(feature_points);  

    // 6> 在当前帧图片上绘制特征点的追踪状态，按跟踪次数从多到少由红到蓝分布，将图片发布到话题"/feature_tracker/feature_img"
    if (SHOW_TRACK)
    {
      ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8); // 将原输入图片的色彩空间转换为BGR8
        //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
      cv::Mat stereo_img = ptr->image;  // 双目图片是由两单目图片上下拼接而成的一张图片

      for (int i = 0; i < NUM_OF_CAM; i++)
      {
        cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW); // i=0和1时，tmp分别为其中一目相机在双目图片中的对应部分
        cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB); // void cv::CvtColor( const CvArr* src, CvArr* dst, int code );

        // 显示当前帧特征点的追踪状态，越红越好，越蓝越不行
        for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
        {
          // 当前特征点按照跟踪次数确定的红色程度
          double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);  
            // WINDOW_SIZE是特征点跟踪次数的基准值，用于归一化跟踪次数，来为特征点由红到蓝着色，此处赋值为=20
          
          // 在tmp_img上的当前帧特征点处画圆，按照跟踪次数大小，跟踪次数越大越红（即len越大），反之越蓝
          // cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2); 
          cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255 * len, 255 * (1 - len)), 2); // MLB 修改: 从绿到红
              // cv::circle函数参数: void circle(CV_IN_OUT Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0); 
              // CvScalar是一个可以用来存放4个double数值的数组，一般用来存放像素值，通道顺序是BGR
            
          //draw speed line
          
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    
          //char name[10];
          //sprintf(name, "%d", trackerData[i].ids[j]);
          //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        }
      }
      //cv::imshow("vis", stereo_img);
      //cv::waitKey(5);

      // 发布使用颜色标记了的特征点跟踪状态的当前帧图片
      pub_match.publish(ptr->toImageMsg());
    }
  }
  ROS_INFO("whole feature tracker processing costs: %f", t_r.toc()); // 输出时间
}

//********************************************************************************************
// 主函数
int main(int argc, char **argv)
{
  // 1. 初始化ROS节点和句柄
  ros::init(argc, argv, "feature_tracker"); // 节点名称为"feature_tracker"
  ros::NodeHandle n("~");                   // ROS节点句柄为n

  // 2. 设置ROS的logger的级别。只有级别大于或等于该level的日志记录消息才会得到处理。
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  // 3. 调用全局函数readParameters()，读取launch文件中的节点参数及yaml配置文件中的配置参数，保存到全局变量（函数和变量均定义于parameters.h文件）。
  readParameters(n); // n为ROS节点句柄；readParameters()函数定义于“parameters.cpp/parameters.h”中

  // 4. 读取每个相机实例对应的相机内参，NUM_OF_CAM=1为单目
  for (int i = 0; i < NUM_OF_CAM; i++)
    trackerData[i].readIntrinsicParameter(CAM_NAMES[i]); 
    // trackerData是FeatureTracker类的一维数组，CAM_NAMES是相机配置文件路径的vector

  // 5. 判断每个相机是否是鱼眼相机，若是则将鱼眼mask存入该相机实例，用于去除其原图四周边缘部分。
    // kitti不是鱼眼，是针孔相机！
  if (FISHEYE)
  {
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
      trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0); // 导入FISHEYE_MASK图片到当前跟踪中
      
      if (!trackerData[i].fisheye_mask.data)
      {
        ROS_INFO("load mask fail");
        ROS_BREAK();
      }
      else
        ROS_INFO("load mask success");
    }
  }

  // 6. 声明1个话题订阅器并为全局变量中声明的3个话题发布器指定话题
    // 订阅图片话题IMAGE_TOPIC(即"/cam0/image_raw")，执行回调函数img_callback()。
  ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // 发布话题"/feature_tracker/feature"，消息内容为feature_points，为当前帧的归一化特征点（x,y,z=1），给后端优化用
  pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);  // 不加"/"则该话题位于当前节点命名空间/feature_tracker下

    // 发布话题"/feature_tracker/feature_img"，消息内容为当前帧图片实例ptr，其按照跟踪次数将特征点标记为红色（次数大）和蓝色，给RVIZ用和调试用
  pub_match = n.advertise<sensor_msgs::Image>("feature_img", 1000);

    // 发布话题"/feature_tracker/restart"
  pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);
  
  /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
  
  ros::spin();
  return 0;
}

// new points velocity is 0, pub or not?
// track cnt > 1 pub?