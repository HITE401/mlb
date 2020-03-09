#include "camodocal/camera_models/CameraFactory.h"

#include <boost/algorithm/string.hpp>

#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include "ceres/ceres.h"

namespace camodocal
{

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{
}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
  if (m_instance.get() == 0)
  {
    m_instance.reset(new CameraFactory);
  }

  return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string &cameraName,
                              cv::Size imageSize) const
{
  switch (modelType)
  {
  case Camera::KANNALA_BRANDT:
  {
    EquidistantCameraPtr camera(new EquidistantCamera);

    EquidistantCamera::Parameters params = camera->getParameters();
    params.cameraName() = cameraName;
    params.imageWidth() = imageSize.width;
    params.imageHeight() = imageSize.height;
    camera->setParameters(params);
    return camera;
  }
  case Camera::PINHOLE:
  {
    PinholeCameraPtr camera(new PinholeCamera);

    PinholeCamera::Parameters params = camera->getParameters();
    params.cameraName() = cameraName;
    params.imageWidth() = imageSize.width;
    params.imageHeight() = imageSize.height;
    camera->setParameters(params);
    return camera;
  }
  case Camera::SCARAMUZZA:
  {
    OCAMCameraPtr camera(new OCAMCamera);

    OCAMCamera::Parameters params = camera->getParameters();
    params.cameraName() = cameraName;
    params.imageWidth() = imageSize.width;
    params.imageHeight() = imageSize.height;
    camera->setParameters(params);
    return camera;
  }
  case Camera::MEI:
  default:
  {
    CataCameraPtr camera(new CataCamera);

    CataCamera::Parameters params = camera->getParameters();
    params.cameraName() = cameraName;
    params.imageWidth() = imageSize.width;
    params.imageHeight() = imageSize.height;
    camera->setParameters(params);
    return camera;
  }
  }
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(const std::string &filename)
{
  // 1. 打开相机配置yaml文件
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    return CameraPtr();
  }

  // 2. 从yaml文件中读取相机类型参数"model_type"，其值包括针孔相机模型"pinhole"等，由此确定相机模型对应为Camera::PINHOLE
  Camera::ModelType modelType = Camera::MEI;  // 初始化相机类型为Camera::MEI
  if (!fs["model_type"].isNone()) // 如果相机类型参数不为空
  {
    // 将yaml文件中的"model_type"赋给变量sModelType
    std::string sModelType;
    fs["model_type"] >> sModelType;

    // 比较判断
    if (boost::iequals(sModelType, "kannala_brandt"))   // "kannala_brandt"
    {
      modelType = Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(sModelType, "mei"))         // "mei"
    {
      modelType = Camera::MEI;
    }
    else if (boost::iequals(sModelType, "scaramuzza"))  // "scaramuzza"
    {
      modelType = Camera::SCARAMUZZA;
    }
    else if (boost::iequals(sModelType, "pinhole"))     // "pinhole"
    {
      modelType = Camera::PINHOLE;
    }
    else
    {
      std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
      return CameraPtr();
    }
  }

  // 
  switch (modelType)
  {
  case Camera::KANNALA_BRANDT:
  {
    EquidistantCameraPtr camera(new EquidistantCamera);

    EquidistantCamera::Parameters params = camera->getParameters();
    params.readFromYamlFile(filename);
    camera->setParameters(params);
    return camera;
  }

  // 针孔相机
  case Camera::PINHOLE:
  {
    PinholeCameraPtr camera(new PinholeCamera);

    PinholeCamera::Parameters params = camera->getParameters();
    params.readFromYamlFile(filename);
    camera->setParameters(params);
    
    return camera;
  }
  case Camera::SCARAMUZZA:
  {
    OCAMCameraPtr camera(new OCAMCamera);

    OCAMCamera::Parameters params = camera->getParameters();
    params.readFromYamlFile(filename);
    camera->setParameters(params);
    return camera;
  }
  case Camera::MEI:
  default:
  {
    CataCameraPtr camera(new CataCamera);

    CataCamera::Parameters params = camera->getParameters();
    params.readFromYamlFile(filename);
    camera->setParameters(params);
    return camera;
  }
  }

  return CameraPtr();
}

} // namespace camodocal
