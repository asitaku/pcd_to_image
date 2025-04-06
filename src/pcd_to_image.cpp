//
// Created by irving on 23-10-7.
//
#include "pcd_to_image/pcd_to_image.h"

namespace pcd_to_image {
pcl::PointCloud<pcl::PointXYZ>::Ptr
filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float x_min,
                 float x_max, float y_min, float y_max, float z_min,
                 float z_max) {
  // 创建滤波器对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");       // 设置滤波的字段为 X 坐标
  pass.setFilterLimits(x_min, x_max); // 设置 X 坐标范围
  pass.filter(*cloud);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");       // 设置滤波的字段为 Y 坐标
  pass.setFilterLimits(y_min, y_max); // 设置 Y 坐标范围

  pass.filter(*cloud);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");       // 设置滤波的字段为 Y 坐标
  pass.setFilterLimits(z_min, z_max); // 设置 Y 坐标范围

  pass.filter(*cloud);

  return cloud;
}

} // namespace pcd_to_image
int main() {
  using namespace pcd_to_image;
  // 从PCD文件中读取点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/media/ywj/Steins_Gate/scans_proc.pcd", *cloud) ==
      -1) {
    std::cout << "Failed to load PCD file" << std::endl;
    return -1;
  }

  double min_x = -5;
  double max_x = 15;
  double min_y = -7;
  double max_y = 7;
  double min_z = -0.5;
  double max_z = 2;

  cloud = filterPointCloud(cloud, min_x, max_x, min_y, max_y, min_z, max_z);

  // 估计法线
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//  ne.setInputCloud(cloud);
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//  ne.setSearchMethod(tree);
//  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//  ne.setRadiusSearch(0.1); // 可根据数据集调整
//  ne.compute(*normals);

  // 初始化SAC分割器，并设置模型类型和方法
//  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
//  seg.setOptimizeCoefficients(true);
//  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//  seg.setMethodType(pcl::SAC_RANSAC);
//  seg.setMaxIterations(100);
//  seg.setDistanceThreshold(0.3);
//  seg.setAxis(Eigen::Vector3f(0,0,1)); // Z轴方向
//  seg.setEpsAngle(20 * M_PI / 180.0); // 与Z轴的最大夹角20度，约束为水平面或缓坡
//
//  seg.setInputCloud(cloud);
//  seg.setInputNormals(normals);
//
//  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//  seg.segment(*inliers, *coefficients);
//
//  // 提取符合条件的点
//  pcl::ExtractIndices<pcl::PointXYZ> extract;
//  extract.setInputCloud(cloud);
//  extract.setIndices(inliers);
//  extract.setNegative(true); // 筛去地面和缓坡
//  extract.filter(*cloud);

  /// 计算图像的尺寸（翻转前）
  int image_width = (max_x - min_x) / 0.05;
  int image_height = (max_y - min_y) / 0.05;

  // 创建图像
  cv::Mat image(image_height, image_width, CV_16UC1, cv::Scalar(0));

  /// 点叠加
  // 将点投影到图像上
  for (const auto &point : cloud->points) {
    // 计算点在图像中的坐标
    int image_x =
        static_cast<int>((point.x - min_x) / (max_x - min_x) * image_width);
    int image_y =
        static_cast<int>((point.y - min_y) / (max_y - min_y) * image_height);
    int image_z = static_cast<int>((point.z - min_z) / (max_z - min_z) * 255);

    // 在图像上绘制点
    image.at<ushort>(image_y, image_x) += 3;
  }

  /// 标准化
  cv::normalize(image, image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  /// gamma
//  cv::Mat look_up_table_ = cv::Mat::ones(1, 256, CV_8U);
//  uchar *p = look_up_table_.ptr();
//  for (int i = 0; i < 256; ++i)
//    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 0.1) * 255.0);
  //    cv::LUT(image,look_up_table_,image);

  /// 滤波
  //    cv::threshold(image, image, 50, 255, cv::THRESH_BINARY);
  //    cv::Mat openedImage;
  //    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,
  //    3)); // 结构元素 cv::morphologyEx(image, image, cv::MORPH_CLOSE,
  //    kernel);

  /// 翻转
  //    cv::flip(image,image,0);
  //    cv::transpose(image,image);

  cv::imshow("Image", image);
  cv::imwrite("/media/ywj/Steins_Gate/scans_proc_img.png", image);
  cv::waitKey(0);

  return 0;
}