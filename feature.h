#ifndef FEATURE_H_
#define FEATURE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/boost.h>
#include <vector>
class Feature
{
public:
  //超体素点云协方差矩阵的特征值，按非降序排列
  Eigen::Vector3f lamda_;
  //超体素点云中的点的法向量与地面（水平面）的夹角，第一个值是平均值，第二个值是方差
  Eigen::Vector2f angel_;
  //超体素点云的最高点，中间点和最低点的高度，按非降序排列
  Eigen::Vector3f height_;
  //超体素点云在CIELab颜色空间的颜色的平均值和方差，前三个是lab平均值，后三个是lab方差
  Eigen::Matrix<float, 6, 1> lab_;

  void getLamda(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

  void getAngel(const pcl::PointCloud<pcl::Normal> &normals);

  void getHeight(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);

  void getLab(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
  
  Eigen::Matrix<float, 14, 1> getFeature();

private:
  Eigen::Vector3f
  RGB2Lab(const Eigen::Vector3i &colorRGB);

  Eigen::Vector2f
  compute(const std::vector<float> &vec_in);
};

#endif
