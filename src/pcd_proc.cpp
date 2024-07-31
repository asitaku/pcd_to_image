//
// Created by irving on 23-10-11.
//
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

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

int main()
{
    // 读取PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(
      "/home/xianghong/sentry_files/PCD/july_8.pcd", *cloud);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // 定义旋转角度（弧度制）
  float roll = 0.82;
  float pitch = 0.05;
  float yaw = -M_PI;

  // 创建旋转矩阵
  Eigen::Matrix3f rotation;
  rotation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

  // 将旋转矩阵复制到4x4变换矩阵中
  transform.block<3, 3>(0, 0) = rotation;

  // 应用旋转变换
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *transformedCloud, transform);

  //  /// 范围滤波
  //  double min_x = -8;
  //    double max_x = 25;
  //    double min_y = -10;
  //    double max_y = 10;
  //    double min_z = -0.1;
  //    double max_z = 1;
  //
  //    transformedCloud = filterPointCloud(cloud, min_x, max_x, min_y, max_y,
  //    min_z, max_z);
  //
  // 保存到新的PCD文件中
  pcl::io::savePCDFileASCII("/home/xianghong/sentry_files/PCD/july_8_trans.pcd",
                            *transformedCloud);

  std::cout << "Saved transformed point cloud to output.pcd" << std::endl;
  return 0;
}