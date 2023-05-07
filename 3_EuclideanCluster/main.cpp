#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <Windows.h>
#include <stdio.h>
#include <psapi.h>
#include <iostream>
#include <vector>

#include <QDir>


using namespace pcl::console;
using namespace std;


 


int main(int argc, char* argv[])
{
  //输出参考信息
  if (argc < 2){
    std::cerr << ".exe xx.pcd -d 0.02 -MinS 1 -MaxS 250000               "       << std::endl;			
    std::cerr << "            -d      Set the maximum distance between classes"    << std::endl;
    std::cerr << "            -MinS   0 unused passfilter; 1 means used"             << std::endl;
    std::cerr << "            -MaxS   Set the upper limit of pass through filtering" << std::endl;

    system("pause");
		return (0);
	}

  //参数赋值
	float Distance = 0.02;            // 类间最大距离，单位：m
  int   MinSize = 1;                // 聚类中包含的最少点云数目
  int   MaxSize = 250000;           // 聚类中包含的最多点云数目

	parse_argument(argc, argv, "-d" , Distance);	//设置类间最大距离，单位：m
	parse_argument(argc, argv, "-MinS", MinSize);	//设置聚类中包含的最少点云数目
	parse_argument(argc, argv, "-MaxS", MaxSize);	//设置聚类中包含的最多点云数目



  // 读取文件
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

  // 创建KD树对象
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdtree->setInputCloud (cloud);

  // 创建点云聚类索引向量
  std::vector<pcl::PointIndices> cluster_indices;

  // 实例化一个欧式聚类提取器
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  
  //欧式聚类提取器参数设定
  ec.setClusterTolerance (Distance); // 类间最大距离，单位：m
  ec.setMinClusterSize (MinSize);    // 聚类中包含的最少点云数目
  ec.setMaxClusterSize (MaxSize);    // 聚类中包含的最多点云数目
  ec.setSearchMethod (kdtree);       // 聚类方法：KD树
  ec.setInputCloud (cloud);          // 输入点云，执行欧式聚类
  ec.extract (cluster_indices);      // 存放位置


  // 判断文件夹是否存在，不存在则创建
  QDir dir("Cluster");
  if(!dir.exists()){
    bool ismkdir = dir.mkpath("Cluster");
    if(!ismkdir)
        cout << "Create path fail" << endl;
    else
        cout << "Create fullpath success" << endl;
  }

  // 存入点云文件
  pcl::PCDWriter writer;
  int j = 0;
  for (const auto & cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    } //*
    
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if(cloud_cluster->size () > 0){
      std::cout << "PointCloud representing the Cluster"<< std::to_string(j) <<": " << cloud_cluster->size () << " data points." << std::endl;
      writer.write<pcl::PointXYZRGB> ("./Cluster/cloud_cluster_" + std::to_string(j) + ".pcd", *cloud_cluster, false); //*
    }
    j++;
  }
  std::cout << "Program finished" << std::endl;


  system("pause");
  return (0);
}

