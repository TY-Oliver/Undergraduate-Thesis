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

3. 输出：会在当前路径下生成一个文件“RecoveryCloud.pcd”

