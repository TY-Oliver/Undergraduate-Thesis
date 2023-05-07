#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h> // 参数粘贴需要
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>

#include <psapi.h>
#include <iostream>
#include <thread>

using namespace pcl::console; // 参数粘贴需要，命名空间的概念最好学习一下
using namespace std::chrono_literals;

int main(int argc, char* argv[]){


  //输出参考信息
  if (argc < 2){
    std::cerr << "Fit_Circle.exe xx.pcd -dt 0.02                    "       << std::endl;			
    std::cerr << "            -dt      Set Distance Threshold"    << std::endl;


    system("pause");
		return (0);
	}

  //参数赋值
	float DistanceThreshold = 0.02;                         // 类间最大距离，单位：m
	parse_argument(argc, argv, "-dt" , DistanceThreshold);	//设置类间最大距离，单位：m


  // 变量初始化
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  std::vector<int> inliers; 
  Eigen::VectorXf modelParas;

  // 读取文件
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

  // 实例化模型
  pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGB>(cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_circle3D);
  ransac.setDistanceThreshold(DistanceThreshold);
  ransac.computeModel();
  ransac.getInliers(inliers);
  ransac.getModelCoefficients(modelParas);
  std::cout << modelParas<< std::endl;
  std::cout << "x y z rpc/2 nx ny nz"<< std::endl;

  pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *final);
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("Circle.pcd", *final, false);



	return 0;
}
