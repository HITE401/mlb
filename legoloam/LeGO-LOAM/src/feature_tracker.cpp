#include "feature_tracker.h"

// 将类的static成员变量n_id初始化为0，用于重新更新特征点id
int FeatureTracker::n_id = 0;

// Function: inBorder() ***************************************************************************
// 功能: 判断跟踪的特征点是否在当前图像边界内
bool inBorder(const cv::Point2f &pt)
{
  const int BORDER_SIZE = 1;  // 边界裕值
  // cvRound()：返回跟参数最接近的整数值，即四舍五入；
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  // COL为图片的列数（等于宽度），ROW为图片的行数（等于高度）
  return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

// Function: reduceVector() ************************************************************************
// 功能: 去除当前帧的特征点存储向量forw_pts中的跟踪失败的特征点（特征点类型为cv::point2f）
/* @param v       - 2D图像特征点构成的vector
 * @param status  - 对应特征点是否被成功跟踪的状态，跟踪失败=0
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];  // 直接将特征点vector中status=0的点前移覆盖，最后resize该vector
  v.resize(j);
}

// 去除当前帧的特征点存储向量中的跟踪失败的特征点（特征点相关参数类型为int）
void reduceVector(vector<int> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

// FUnction: FeatureTracker() **********************************************************************
// - 空的构造函数
FeatureTracker::FeatureTracker()
{
}

// Function: setMask() *****************************************************************************
/** 
 * 功能: 
 *  1> 对当前帧跟踪特征点按照跟踪次数从大到小进行排序（forw_pts、ids、track_cnt）；
 *  2> 如果是鱼眼相机，则将鱼眼mask赋给限定当前帧特征点分布范围的mask，
 *  3> 按照跟踪次数多少从多到少依次选取特征点，并在mask中将选取特征点周围半径范围（MIN_DIST）内值设为0，
 *      最后得到鱼眼mask+已跟踪特征点双重限定后用于限定新特征点提取范围的mask。
 * 输入:
 *  1> fisheye_mask             - 鱼眼相机mask图片
 *  2> forw_pts, ids, track_cnt - 排序前的当前帧跟踪特征点的像素坐标、id和跟踪次数向量
 *  3> MIN_DIST                 - 限定特征点均匀分布的半径 = 30 像素
 * 输出: 
 *  1> forw_pts, ids, track_cnt - 按跟踪次数从大到小排序后，并使用鱼眼mask+均匀分布剔除后的当前帧跟踪特征点的像素坐标、id和跟踪次数向量
 *  2> mask - 鱼眼mask+已跟踪特征点双重限定后用于限定新特征点提取范围的mask图片
 */
void FeatureTracker::setMask()
{
  // (1) 如果是鱼眼相机，则令mask为鱼眼mask以去除边缘噪点，否则使用像素值为255的空mask（即无遮挡）
  if (FISHEYE)
    mask = fisheye_mask.clone();
  else
    mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

  // (2) 构造当前帧所有特征点的序列cnt_pts_id，(cnt，pts，id)中元素依次为该特征点的跟踪次数、像素位置、id，所有特征点构成一个vector
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < forw_pts.size(); i++)
    cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

  // (3) 对当前帧的特征点序列cnt_pts_id，按照被跟踪到的次数cnt从大到小排序元素（lambda表达式）
    /* 直接使用C++排序函数sort()对向量cnt_pts_id中的元素排序（降序）*/
  sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b) {
    return a.first > b.first; // 跟踪次数从大到小排列
  });

  // (4) 清空类数据域中当前帧特征点的像素位置、id、跟踪次数3个向量，使用排序后的序列cnt_pts_id重新存入
  forw_pts.clear();
  ids.clear();
  track_cnt.clear();

  // 遍历排序后的序列cnt_pts_id中的所有特征点，
    // 如果当前特征点像素位置对应的mask值为255（即无遮挡），则：
    // 1> 保留当前特征点，将对应特征点的位置pts，id，被追踪次数cnt分别存入类的数据域，
    // 2> 在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的点
  for (auto &it : cnt_pts_id) 
  {
    if (mask.at<uchar>(it.second.first) == 255) // it.second.first为当前特征点的像素坐标
    {
      forw_pts.push_back(it.second.first);  
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);

      // 在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

// Function: addPoints() **************************************************************************
// 功能: 将当前帧新检测到的特征点n_pts，push_back()到当前帧已跟踪的特征点向量（for_pts）中，对应id设为-1，跟踪次数设为1
void FeatureTracker::addPoints()
{
  for (auto &p : n_pts)
  {
    forw_pts.push_back(p);  
    ids.push_back(-1);      //新提取的特征点id初始化为-1
    track_cnt.push_back(1); //新提取的特征点被跟踪的次数初始化为1
  }
}

// Function: readImage() *************************************************************************
// 对当前帧图像使用光流法进行特征点跟踪，使用F矩阵个RANSAC方法去除外点，提取新的特征点
/**
 * 功能: 
 *  1> 如果EQUALIZE=1，表示输入图片太亮或太暗，则对其进行直方图均衡化处理;
 *  2> 从前一帧特征点（cur_pts）进行LK光流跟踪，剔除跟踪失败的点和位于当前图像边界外的点后，得到当前帧特征点fow_pts;
 *  3> 光流追踪完成后，当前帧所有光流跟踪得到的特征点的成功跟踪次数track_cnt加1;
 *  4> 如果PUB_THIS_FRAME=1，则需要发布当前帧的特征点，进行如下操作: 
 *    1) 调用函数rejectWithF()。在其中调用cv::findFundamentalMat()函数，计算fundamental矩阵并利用RANSAC方法，来剔除当前帧跟踪特征点中的外点。
 *    2) 调用setMask()函数。
 *    3) 如果跟踪特征点数目少于MAX_CNT=150，则调用方法cv::goodFeaturesToTrack()，从当前帧图片mask外区域提取新的shi-tomasi特征点补足数目，
         结果存到数据域（n_pts）
 *  5> 更新前二帧的图片、特征点、归一化特征点为前一帧；更新前一帧的图片、特征点为当前帧。
 *  6> 调用类函数undistortedPoints()，将当前帧像素特征点（cur_pts）投影到二维归一化特征点（cur_un_pts和cur_un_pts_map）；
 *     计算归一化特征点速度（pts_velocity）；更新前一帧归一化特征点和时间戳（prev_un_pts_map、prev_time）；
 * 输入:
 *  1> _img       - 当前帧图片，cv::Mat格式
 *  2> _cur_time  - 当前帧图片消息的时间戳
 *  3> EQUALIZE   - 表示输入图片太亮或太暗的标志，若=1则对其进行自适应直方图均衡化
 * 输出:
 * @Description 
 *  - cv::createCLAHE()           对图像进行自适应直方图均衡化，提高对比度
 *  - cv::calcOpticalFlowPyrLK()  使用LK金字塔光流法跟踪特征点
 *  - setMask()                   按照跟踪次数对跟踪点进行排序，设置mask
 *  - rejectWithF()               通过基本矩阵剔除outliers
 *  - cv::goodFeaturesToTrack()   添加特征点(shi-tomasi角点)，确保每帧都有足够的特征点
 *  - addPoints()                 添加新的追踪点
 *  - undistortedPoints()         对角点图像坐标去畸变矫正，并计算每个角点的速度
*/
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
  cv::Mat img;          // 使用自适应直方图均衡化处理后的当前帧图片
  cur_time = _cur_time; // 当前帧图片的时间戳
  TicToc t_r;  

  // (1) 如果EQUALIZE=1，表示输入图片太亮或太暗，则对其进行直方图均衡化处理后再赋给img，否则直接赋给img     
  if (EQUALIZE)
  {
    // 自适应直方图均衡
    // createCLAHE(double clipLimit, Size tileGridSize)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    TicToc t_c;
    clahe->apply(_img, img);
    ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
  }
  else
    img = _img;

  // (2) 更新数据域当前帧图片forw_img
    // 如果数据域的当前帧图像为空（forw_img.empty()），则说明当前是第一次读入图像数据，将读入的图像同时赋给当前帧forw_img、
    // 前一帧cur_img和前二帧prev_img；否则，只需要更新当前帧图像forw_img
  if (forw_img.empty())
  {
    prev_img = cur_img = forw_img = img;
  }
  else
  {
    forw_img = img;
  }

  // (3) 如果前一帧的特征点数目大于0，则从前一帧特征点（cur_pts）进行LK光流跟踪，剔除跟踪失败的点和位于当前图像边界外的点后，得到当前帧特征点fow_pts
  forw_pts.clear(); // 将当前帧特征点向量forw_pts清空
  if (cur_pts.size() > 0)
  {
    TicToc t_o;
    vector<uchar> status; // 特征点的跟踪状态，跟踪失败=0，跟踪成功=1
    vector<float> err;

    // 1> 调用cv::calcOpticalFlowPyrLK()方法，对前一帧的特征点cur_pts进行LK金字塔光流跟踪，跟踪结果存到forw_pts，跟踪状态存到到status
    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

    // 2> 将跟踪成功但位于图像边界外的点的状态标记为0（status=0）
    for (int i = 0; i < int(forw_pts.size()); i++)
      if (status[i] && !inBorder(forw_pts[i])) 
        status[i] = 0;  

    // 3> 调用函数reduceVector()。根据跟踪状态status，把当前帧跟踪失败的点和位于图像边界外的点从存储向量中剔除，得到更新后的forw_pts、ids和track_cnt。
      // 此外还要把prev_pts、cur_pts和cur_un_pts中对应位置剔除。
    reduceVector(forw_pts, status); // 当前帧特征点的位置、id和跟踪次数
    reduceVector(ids, status);      
    reduceVector(track_cnt, status);

    reduceVector(prev_pts, status); // ???
    reduceVector(cur_pts, status);
    reduceVector(cur_un_pts, status);
    
    ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
  }

  // (4) 光流追踪完成后，当前帧所有光流跟踪得到的特征点的成功跟踪次数加1
    // 此处应该放到前面的if中？
  for (auto &n : track_cnt)
    n++;

  // (5) 如果PUB_THIS_FRAME=1，则需要发布当前帧的特征点，进行如下操作: 
  if (PUB_THIS_FRAME)
  {
    // 1> 调用函数rejectWithF()。在其中调用cv::findFundamentalMat()函数，计算fundamental矩阵并利用RANSAC方法，来剔除当前帧跟踪特征点中的外点。
    rejectWithF();

    // 2> 调用setMask()函数。实现:
      // 1) 对当前帧跟踪特征点按照跟踪次数从大到小进行排序（forw_pts、ids、track_cnt）；
      // 2) 如果是鱼眼相机，则将鱼眼mask赋给限定当前帧特征点分布范围的mask，
      // 3) 按照跟踪次数从多到少依次选取特征点，并在mask中将选取特征点半径（MIN_DIST）内值设为0，
        // 最后得到鱼眼mask+已跟踪特征点双重限定后用于限定新特征点提取范围的mask。
    ROS_DEBUG("set mask begins");
    TicToc t_m;
    setMask(); 
    ROS_DEBUG("set mask costs %fms", t_m.toc());

    // 3> 如果跟踪特征点数目少于MAX_CNT=150，则调用方法cv::goodFeaturesToTrack()，从当前帧图片mask外区域提取新的shi-tomasi特征点补足数目，
      // 结果存到数据域（n_pts）
    ROS_DEBUG("detect new features begins");
    TicToc t_t;
    int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());  // 需要新提取的特征点数目
    if (n_max_cnt > 0)
    {
      // 判断特征跟踪点setMask()后的mask状态是否正确
      if (mask.empty())
        cout << "mask is empty " << endl;
      if (mask.type() != CV_8UC1)
        cout << "mask type wrong " << endl;
      if (mask.size() != forw_img.size())
        cout << "wrong size " << endl;
      
      // 在mask中值不为0的区域检测新的shi-tomasi角点，使得总角点数目达到MAX_CNT，存到变量n_pts
        /* void cv::goodFeaturesToTrack(    
         *   InputArray   image,              输入图像
         *   OutputArray  corners,            存放检测到的角点的vector
         *   int          maxCorners,         返回的角点的数量的最大值
         *   double       qualityLevel,       角点质量水平的最低阈值（范围为0到1，质量最高角点的水平为1），小于该阈值的角点被拒绝
         *   double       minDistance,        返回角点之间欧式距离的最小值
         *   InputArray   mask = noArray(),   和输入图像具有相同大小，类型必须为CV_8UC1，用来描述图像中感兴趣的区域，只在感兴趣区域中检测角点
         *   int          blockSize = 3,              计算协方差矩阵时的窗口大小
         *   bool         useHarrisDetector = false,  指示是否使用Harris角点检测，如不指定则使用shi-tomasi算法
         *   double       k = 0.04                    Harris角点检测需要的k值
         *)   
         */
      cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
    }
    else
      n_pts.clear();
    ROS_DEBUG("detect feature costs: %fms", t_t.toc());

    // 4> 调用类函数addPoints()，将新检测到的特征点n_pts追加到forw_pts中，ids初始化-1,track_cnt初始化为1。
    ROS_DEBUG("add feature begins");
    TicToc t_a;
    addPoints();

    ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
  }

  // (6) 更新prev帧的图片、像素特征点、归一化特征点为cur帧；更新cur帧的图片、像素特征点为当前fow帧。
    // 更新前二帧的图片、特征点、归一化特征点为前一帧
  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;

    // 更新前一帧的图片、特征点为当前帧
  cur_img = forw_img;
  cur_pts = forw_pts;

  // (7) 调用类函数undistortedPoints()，将当前帧像素特征点（cur_pts）投影到二维维归一化特征点（cur_un_pts和cur_un_pts_map）；
    // 计算归一化特征点速度（pts_velocity）；更新前一帧归一化特征点和时间戳（prev_un_pts_map、prev_time）；
  undistortedPoints();
  prev_time = cur_time; // 更新前一帧的时间戳为当前帧时间戳（可以放到函数内）
}

// Function: rejectWithF() ************************************************************
/**
 * 功能: 调用cv::findFundamentalMat()函数，计算fundamental矩阵并利用RANSAC方法，来剔除当前帧跟踪特征点中的外点。
 *      （即当前帧跟踪特征点中，相对于前一帧对应点不满足对极几何约束的点）
 * 注: reduceVector(cur_pts, status)函数，用于根据向量中对应元素的状态值，将该元素删去。
*/
void FeatureTracker::rejectWithF()
{
  // 如果当前跟踪的特征点数>=8个
  if (forw_pts.size() >= 8)
  {
    ROS_DEBUG("FM ransac begins");
    TicToc t_f;

    // 将当前帧和前一帧的对应跟踪特征点，从二维像素坐标转换到三维归一化平面坐标(un_forw_pts, un_cur_pts)
    vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
      Eigen::Vector3d tmp_p;  // 中间变量
      
      // 将前一帧的特征点从二维像素坐标转换到三维归一化平面坐标
      m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p); // 根据不同的相机模型将二维坐标转换到三维坐标
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0; // 转换为归一化平面坐标
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      // 将当前帧的对应特征点从二维像素坐标转换到三维归一化平面坐标
      m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
      un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    // 调用cv::findFundamentalMat()，计算F矩阵并利用RANSAC方法，来剔除当前帧跟踪特征点中的外点。
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
    int size_a = cur_pts.size();    // 当前帧跟踪点的数目
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);

    ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
    ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
  }
}

// Function: updateID() *************************************************************
// 检测当前帧某一个特征点是否是新提取的特征点，若是则更新其id为全局变量n_id，且n_id++（本次跟踪初始时为0）
bool FeatureTracker::updateID(unsigned int i)
{
  if (i < ids.size())
  {
    // 如果该特征点是新提取的特征点，则将其id设为n_id，且n_id++，n_id在本次跟踪初始时初始化为0
    if (ids[i] == -1)
      ids[i] = n_id++; // n_id为当前相机从本次跟踪开始以来，所有独立帧特征点的id
    return true;
  }
  else
    return false;
}

// Function: readIntrinsicParameter() ***********************************************
// 读取相机内参矩阵、畸变参数等参数
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
  ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
  m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

// Function: showUndistortion() *******************************************************
// 显示一张空图片去畸变矫正后的特征点，输入name为图像帧名称？？？
void FeatureTracker::showUndistortion(const string &name)
{
  cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
  vector<Eigen::Vector2d> distortedp, undistortedp; // 畸变点和去畸变点

  // 遍历图片中的所有像素，利用相机内参将2d像素点去畸变并投影到3d空间点，再转化到归一化平面二维点，存入undistortedp
  for (int i = 0; i < COL; i++)
    for (int j = 0; j < ROW; j++)
    {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      m_camera->liftProjective(a, b); // 利用相机内参，将2d像素点去畸变，并投影到3d空间点
      distortedp.push_back(a);        // 将该2d像素点加入未去畸变的点
      undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z())); // 将转化到归一化平面上的点加入去畸变点
      //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
    }
  
  // 遍历图片中所有去畸变后的归一化二维点
  for (int i = 0; i < int(undistortedp.size()); i++)
  {
    cv::Mat pp(3, 1, CV_32FC1); // 3行1列的矩阵
    pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
    pp.at<float>(2, 0) = 1.0;
    //cout << trackerData[0].K << endl;
    //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
    //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
    {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
    }
    else
    {
      //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
    }
  }
  cv::imshow(name, undistortedImg);
  cv::waitKey(0);
}

// Function: undistortedPoints() ************************************************************
/**
 * 功能: 
 *  1> 遍历当前帧所有像素特征点（cur_pts），将其投影为二维归一化特征点赋给cur_un_pts和cur_un_pts_map（带有特征点id的map键值对）;
 *  2> 计算当前帧每个归一化特征点的速度，存到pts_velocity；
 *  3> 更新前一帧归一化特征点，prev_un_pts_map = cur_un_pts_map；
 * 输入: 
 *  1> cur_pts        - 当前帧二维像素特征点（已从fowr_pts更新）
 * 输出: 
 *  1> cur_un_pts     - 当前帧二维归一化特征点（cv::Point2f格式）
 *  2> cur_un_pts_map - 带有id的当前帧二维归一化特征点
 *  3> pts_velocity   - 当前帧每个归一化特征点的速度，新提取点速度置为0
 * 注: 归一化平面特征点的速度公式: v=(cur_un_pts-prev_un_pts)/(cur_time-prev_time)
*/
void FeatureTracker::undistortedPoints()
{
  // 清空
  cur_un_pts.clear();
  cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());

  // (1) 遍历当前帧所有像素特征点（cur_pts），将其投影为二维归一化特征点，赋给cur_un_pts和cur_un_pts_map（带有特征点id，map键值对）
    // * 注意此时已经将fowr_pts赋给cur_pts，将原先的cur_pts赋给prev_pts，因此cur_pts就是当前帧！！！（MLB editted）
  for (unsigned int i = 0; i < cur_pts.size(); i++)
  {
    // a、b分别是当前特征点的二维像素坐标，和反投影后的三维空间坐标
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector3d b;

    // 根据不同的相机模型将二维像素坐标反投影到三维归一化坐标（z=1）
    m_camera->liftProjective(a, b);

    // 再将三维坐标b延伸到归一化平面上，即cur_un_pts
    cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));  // 对于针孔相机模型，已经有z=1，此处无需再除
    cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
      //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
  }

  // (2) 计算当前帧每个归一化特征点的速度，存到pts_velocity，仅用于显示？
    // 归一化平面特征点的速度公式: v=(cur_un_pts-prev_un_pts)/(cur_time-prev_time)
  if (!prev_un_pts_map.empty())
  {
    // cur帧与prev帧的时间戳差值
    double dt = cur_time - prev_time;
    pts_velocity.clear();

    // 遍历当前帧所有归一化特征点，如果不是新提取的特征点，则找出其在前一帧中的对应点坐标，计算归一化特征点的移动速度
    for (unsigned int i = 0; i < cur_un_pts.size(); i++)
    {
      // 如果不是新提取的特征点（ids[i] != -1），则找出其在前一帧中的对应点坐标，计算归一化特征点的移动速度
      if (ids[i] != -1)
      {
        std::map<int, cv::Point2f>::iterator it;
        it = prev_un_pts_map.find(ids[i]);  // 通过该特征点id在前一帧中查找对应特征点
        if (it != prev_un_pts_map.end())
        {
          double v_x = (cur_un_pts[i].x - it->second.x) / dt;
          double v_y = (cur_un_pts[i].y - it->second.y) / dt;
          pts_velocity.push_back(cv::Point2f(v_x, v_y));
        }
        else
          pts_velocity.push_back(cv::Point2f(0, 0));
      }
      // 否则，即该点是新提取的特征点，直接令归一化特征点速度为0
      else
      {
        pts_velocity.push_back(cv::Point2f(0, 0));
      }
    }
  }
  else
  {
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }

  // (3) 更新前一帧归一化特征点，prev_un_pts_map = cur_un_pts_map
  prev_un_pts_map = cur_un_pts_map;
}
