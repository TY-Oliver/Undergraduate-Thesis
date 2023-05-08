# 植物叶片点云分割方法研究

Date：2023.05.07

Author：Oliver

Description：这是本人的本科毕业论文，创建一个仓库以共享源代码。

## 研究过程

![LiuCheng](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/Asset/LiuCheng.png)

## 三维重建
1. 拍摄图片：左图是半球形拍摄的角度，右图是拍摄所得到的图片示例

<img src="/Asset/Angle.png" width="336" height="315"><img src="/Asset/Picture.png" width="435" height="315">


2. 使用[COLMAP](https://colmap.github.io/)进行三维重建

3. 重建得到的六组数据

   ![OriData4](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/Asset/OriData4.png)


## 盆口圆拟合

[PCL编写的代码](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/1_Fit_Circle/main.cpp)

使用方法：

1. 将需要拟合的圆用CloudCompare软件裁剪出来

2. CMD输入

   ```
    Fit_Circle.exe 1.pcd -dt 0.02
   ```
   -dt：RANSAC三维圆拟合的内点范围，单位：米

3. 输出

   ```
   PointCloud before filtering has: 37033 data points.
   4.85608
   -4.89641
   -3.42948
   1.71573
   0.0372366
   -0.998126
   -0.0485669
   x y z rpc/2 nx ny nz
   
   ```

   第1-3个值是圆心坐标(x,y,z)，第4个值是圆的半径，第5-7个值是圆所在平面的法向量(nx,ny,nz)

六组实验数据的输出结果：

![Circle](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/Asset/Circle.png)

## 尺度恢复


[PCL编写的代码](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/2_ScaleRecover/main.cpp)

使用方法：

1. 将需要拟合的圆用CloudCompare软件裁剪出来

2. CMD输入

   ```
   Scale_Recovery.exe Cloud.pcd -OriginalScale 0.062 -CurrentScale 0.20937
   ```
   -OriginalScale：原始的尺度，即参照物的实际大小，单位：米
   
   -CurrentScale：原点云的尺度，即参照物的点云大小，单位：米
   
3. 输出：会在当前路径下生成一个文件“RecoveryCloud.pcd”



## 欧式聚类

[PCL编写的代码](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/3_EuclideanCluster/main.cpp)


使用方法：

1. CMD运行

   ```
   Euclidean_Cluster.exe 1_cluster.pcd -d 0.02 -MinS 1 -MaxS 250000
   ```

   -d：欧式聚类类间最小间距

   -MinS：最小类包含点数

   -MaxS：最大类包含点数

2. 输出：程序自动创建一个Cluster文件夹，里面存放各个类的点云

3. 手动挑选出与叶片相关的，合并成同一类。

<img src="/Asset/PrePro.png">


## 区域生长分割
[PCL编写的代码](https://github.com/TY-Oliver/Undergraduate-Thesis/blob/master/4_RegionGrowth/RegionGrowth.cpp)

使用方法：

1. CMD运行

   ```
   RegionGrowth.exe 1_cluster.pcd -kn 50 -st 30 -ct 0.05
   ```

   -kn：K近邻搜索范围的点的个数

   -st：平滑阈值——检验法向量的夹角，单位：弧度

   -ct：曲率阈值

2. 输出：程序自动创建一个RegionGrowth文件夹，里面存放各个类的点云

3. CloudCompare手动挑选出分割出来的叶片，上色

<img src="/Asset/Result.png">

## 结论

大学本科四年就此画上句号
