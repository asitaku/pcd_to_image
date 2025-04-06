//
// Created by irving on 24-1-22.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

int main (int argc, char** argv)
{
    // 定义 Point Cloud 对象
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());


    // 读取点云文件
    pcl::PCDReader reader;
    reader.read ("/home/ywj/nation_origin.pcd", *cloud); // 替换为你的输入文件名

    std::cout<<0<<std::endl;

    // 转为pointcloud类作有类型处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(*cloud,*cloud_xyz);


/// 统计滤波器移除离群点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_xyz);
    sor.setMeanK(10); // 根据周围邻居数量
    sor.setStddevMulThresh(10.0); // 标准差阈值
//    sor.filter(*cloud_xyz);
    std::cout<<1<<std::endl;

/// 半径滤波器
//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    outrem.setInputCloud(cloud_xyz);
//    outrem.setRadiusSearch(0.05); // 设置搜索半径
//    outrem.setMinNeighborsInRadius(20); // 设置半径内最小邻居数目
//    // 应用滤波器
//    outrem.filter(*cloud_xyz);
//    std::cout<<2<<std::endl;

/// 体素网格过滤器对象
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_xyz);
    sor2.setLeafSize (0.0849f, 0.0849f, 0.0849f); // 设置体素网格的大小
    sor2.filter (*cloud_xyz);
    std::cout<<3<<std::endl;

    //转为pointcloud2类作无类型快速保存
    pcl::PCLPointCloud2::Ptr filtered_output (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(*cloud_xyz,*filtered_output);

    // 保存降采样后的点云
    pcl::PCDWriter writer;
    writer.write ("/home/ywj/nation_processed.pcd", *filtered_output); // 输出文件名
    std::cout<<4<<std::endl;

    return (0);
}
