#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <Windows.h>
#include <stdio.h>
#include <psapi.h>
#include <pcl/filters/extract_indices.h>

#include <QDir>
 

using namespace pcl::console;
int main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cerr << "        1_cluster.pcd -kn 50 -st 30 -ct 0.05" << std::endl;			     //输出参考信息
    	std::cerr << "            -kn Set the number of KNN Points" << std::endl;    //
    	std::cerr << "            -st Set smoothing threshold" << std::endl;
    	std::cerr << "            -ct Set Curvature Threshold" << std::endl;
		return (0);
	}
	time_t start, end, diff[5];														         			//用于保存运行的相应时间等
	start = time(0);																					//记录当前的时间

	int KN_normal = 50;
	float SmoothnessThreshold = 30.0, CurvatureThreshold = 0.05;
	
	parse_argument(argc, argv, "-kn", KN_normal);														//设置k近邻点的个数
	parse_argument(argc, argv, "-st", SmoothnessThreshold);												//设置平滑阈值
	parse_argument(argc, argv, "-ct", CurvatureThreshold);												//设置曲率阈值
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);			    //设置输入点云的对象，用于加载点云数据

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	end = time(0);																						//记录加载完点云时的时间
	diff[0] = difftime(end, start);																		//记载加载所用的时间
	PCL_INFO("Loading pcd file takes(seconds):%d\n", diff[0]);
 
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = pcl::shared_ptr< pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);						//智能指针声明kd tree与法线
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;								//实例化一个法线估计对象
	normal_estimator.setSearchMethod(tree);																//设置搜索方式
	normal_estimator.setInputCloud(cloud);																//设置输入点云
	normal_estimator.setKSearch(KN_normal);																//设置k近邻点的个数
	normal_estimator.compute(*normals);																	//执行法线估计
	end = time(0);
	diff[1] = difftime(end, start) - diff[0];															//记录法线估计所用的时间
	PCL_INFO("Estimationg normal takes : %d seconds\n", diff[1]);
	
	pcl::IndicesPtr indices(new std::vector<int>);														//声明一个索引指针

 
	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal>reg;												//创建区域生长的分割对象
	reg.setMinClusterSize(50);																			//设置聚类所需要的最小点数
	reg.setMaxClusterSize(1000000);																		//设置一个聚类的最大点数
	reg.setSearchMethod(tree);																			//设置搜索方式
	reg.setNumberOfNeighbours(30);																		//设置近邻点的个数
	reg.setInputCloud(cloud);																			//设置输入点云
	reg.setInputNormals(normals);																		//输入点云的法向量
	reg.setSmoothnessThreshold(SmoothnessThreshold / 180.0 * M_PI);										//设置平滑阈值
	reg.setCurvatureThreshold(CurvatureThreshold);														//设置曲率阈值
 
	std::vector<pcl::PointIndices> clusters;															//用动态数组保存聚类的结果
	reg.extract(clusters);																				
	end = time(0);																						//计算聚类所需的时间
	diff[2] = difftime(end, start) - diff[0] - diff[1];

	PCL_INFO("Region growing takes %d seconds\n", diff[2]);
 
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
	std::cout << "These are the indices of the points of the initial" << std::endl << "cloud that belong to the first cluster:" << std::endl;
	

  //----------------------------Extract clusters---------------------------------
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::PointIndices::Ptr acluster (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PCDWriter writer;

	  // 判断文件夹是否存在，不存在则创建
    QDir dir("RegionGrowth");
    if(!dir.exists()){
    bool ismkdir = dir.mkpath("RegionGrowth");
    if(!ismkdir)
        cout << "Create path fail" << endl;
    else
        cout << "Create fullpath success" << endl;
    }

	for (size_t i = 0; i < clusters.size(); i++)
	{
		*acluster = clusters[i];
		extract.setInputCloud (cloud);
		extract.setIndices (acluster);
		extract.setNegative (false);
		extract.filter (*cloud_cluster);
		writer.write ("./RegionGrowth/cluster_"+ std::to_string(i) + ".pcd", *cloud_cluster, false);

	}

	return (0);
}
