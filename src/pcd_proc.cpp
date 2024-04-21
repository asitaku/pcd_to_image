//
// Created by irving on 23-10-11.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
int main()
{
    // 读取PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/media/irving/UBUNTU 20_0/robomaster/哨兵pcd/0.03降采样.pcd", *cloud);

    // 定义旋转变换矩阵
    Eigen::Matrix4f transform_roll = Eigen::Matrix4f::Identity();

    // 绕X轴向上抬45度
    float angle = -0.785; // 45度对应的弧度值
    transform_roll (1, 1) = cos(angle);
    transform_roll (1, 2) = -sin(angle);
    transform_roll (2, 1) = sin(angle);
    transform_roll (2, 2) = cos(angle);

    Eigen::Matrix4f transform_yaw = Eigen::Matrix4f::Identity();

    // 绕Z轴旋转5度
    angle = 0; // 5度对应的弧度值
    transform_yaw(0, 0) = cos(angle);
    transform_yaw(0, 1) = -sin(angle);
    transform_yaw(1, 0) = sin(angle);
    transform_yaw(1, 1) = cos(angle);


    Eigen::Matrix4f transform_pitch = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform = transform_yaw * transform_roll;

    // 应用旋转变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformedCloud, transform);

    // 保存到新的PCD文件中
    pcl::io::savePCDFileASCII("/media/irving/UBUNTU 20_0/robomaster/哨兵pcd/0.03降采样_roll45.pcd", *transformedCloud);

    std::cout << "Saved transformed point cloud to output.pcd" << std::endl;
    return 0;
}