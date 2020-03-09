#include "parameters.h"

// 全局变量，已用extern声明，可直接从外部引用
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;


// Function1: readParam() ******************************************************
// 用于从ROS节点参数服务器中读取参数，具体方法为nh.getParam(name, value)
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  T ans;  // 返回结果
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

// Function2: readParameters() **************************************************
void readParameters(ros::NodeHandle &n)
{
  // (1) 调用函数readParam()，在该函数内使用ROS节点方法nh.getParam()读取在launch文件中声明的本节点参数——yaml配置文件路劲。
  std::string config_file;  // 存储yaml配置文件路劲的变量
  config_file = readParam<std::string>(n, "config_file"); // "config_file"是在launch文件中声明的节点参数名称

  // (2) 声明cv::FileStorage类的对象fsSettings，读取指定路径下的yaml配置文件到该对象。
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  // (3) 调用函数readParam()，读取launch文件中的节点参数——VINS代码的根目录路劲。
  //std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

  // (4) 读取yaml配置文件中的对应参数，并赋值给全局变量
  fsSettings["image_topic"] >> IMAGE_TOPIC;
  fsSettings["imu_topic"] >> IMU_TOPIC;

  MAX_CNT = fsSettings["max_cnt"];
  MIN_DIST = fsSettings["min_dist"];
  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  FREQ = fsSettings["freq"];
  F_THRESHOLD = fsSettings["F_threshold"];
  SHOW_TRACK = fsSettings["show_track"];
  EQUALIZE = fsSettings["equalize"];
  FISHEYE = fsSettings["fisheye"];
  //if (FISHEYE == 1)
  //  FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    
  CAM_NAMES.push_back(config_file);

  WINDOW_SIZE = 20; // WINDOW_SIZE是特征点跟踪次数的基准值，用于归一化跟踪次数，来为特征点由红到蓝着色，此处赋值为=20
  STEREO_TRACK = false;
  FOCAL_LENGTH = 460;
  PUB_THIS_FRAME = false;

  if (FREQ == 0)
    FREQ = 100;

  // (5) 关闭文件对象
  fsSettings.release();
}
