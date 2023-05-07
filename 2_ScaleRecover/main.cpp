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
#include<sstream>
#include <thread>

using namespace pcl::console;
using namespace std::chrono_literals;

int main(int argc, char* argv[]){


  //输出参考信息
  if (argc < 2){
    std::cerr << "Scale_Recovery.exe Cloud.pcd -OriginalScale 0.062 -CurrentScale 0.20937"       << std::endl;			

    system("pause");
		return (0);
	}

  //参数赋值
	double OriginalScale = 0.062;            // 原尺度，  或者说实际尺度，单位：m
  double CurrentScale  = 0.20937;          // 当前尺度，或者说点云尺度，单位：m


	parse_argument(argc, argv, "-OriginalScale" , OriginalScale);	//设置  原尺度，  或者说实际尺度，单位：m
	parse_argument(argc, argv, "-CurrentScale", CurrentScale);	  //设置当前尺度，或者说点云尺度，单位：m

  // 尺度恢复系数
  double lamda = OriginalScale/CurrentScale;
  std::cout << "The scale coefficient of restitution is "<< std::to_string(lamda) << std::endl; //*


  // 变量初始化
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  std::vector<int> inliers; 
  Eigen::VectorXf modelParas;

  // 读取文件
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
  std::cout << "PointCloud before recovering has: " << cloud->size () << " data points." << std::endl; //*

  // 尺度恢复
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		// 坐标范围在[0,1024),正方体内随机取点
		cloud->points[i].x = lamda * cloud->points[i].x;
		cloud->points[i].y = lamda * cloud->points[i].y;
		cloud->points[i].z = lamda * cloud->points[i].z;

  }

  // 写入文件
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("RecoveryCloud.pcd", *cloud, false);

	return 0;
}
