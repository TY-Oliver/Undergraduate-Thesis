#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>

#include <psapi.h>
#include <iostream>
#include <thread>

using namespace pcl::console;
using namespace std::chrono_literals;

int main(int argc, char* argv[]){


  //输出参考信息
  if (argc < 2){
    std::cerr << ".exe xx.pcd -dt 0.02 -MinS 1 -MaxS 250000               "       << std::endl;			
    std::cerr << "            -d      Set the maximum distance between classes"    << std::endl;
    std::cerr << "            -MinS   0 unused passfilter; 1 means used"             << std::endl;
    std::cerr << "            -MaxS   Set the upper limit of pass through filtering" << std::endl;

    system("pause");
		return (0);
	}

  //参数赋值
	float DistanceThreshold = 0.02;            // 类间最大距离，单位：m
  int   MinSize = 1;                // 聚类中包含的最少点云数目
  int   MaxSize = 250000;           // 聚类中包含的最多点云数目

	parse_argument(argc, argv, "-dt" , DistanceThreshold);	//设置类间最大距离，单位：m
	parse_argument(argc, argv, "-MinS", MinSize);	//设置聚类中包含的最少点云数目
	parse_argument(argc, argv, "-MaxS", MaxSize);	//设置聚类中包含的最多点云数目

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
