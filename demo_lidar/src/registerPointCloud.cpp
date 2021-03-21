#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "pointDefinition.h"

//订阅每帧深度点云及afterBA的odom，线性插值，得到每帧点云的运动估计值，从而将深度点云影射到世界坐标系camera_init下，并发布得到较为稠密的点云地图
const double PI = 3.1415926;

const int keepVoDataNum = 30;
double voDataTime[keepVoDataNum] = {0};
double voRx[keepVoDataNum] = {0};
double voRy[keepVoDataNum] = {0};
double voRz[keepVoDataNum] = {0};
double voTx[keepVoDataNum] = {0};
double voTy[keepVoDataNum] = {0};
double voTz[keepVoDataNum] = {0};
int voDataInd = -1;
int voRegInd = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr surroundCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr syncCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());

int startCount = -1;
const int startSkipNum = 5;

int showCount = -1;
const int showSkipNum = 15;

ros::Publisher *surroundCloudPubPointer = NULL;

//订阅BA后的odom估计值
//插值得到深度图对应的运动估计值，将点云影射到世界坐标系。
void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData)
{
  double time = voData->header.stamp.toSec();

//旋转
  double rx, ry, rz;
  geometry_msgs::Quaternion geoQuat = voData->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(rz, rx, ry);

//偏移
  double tx = voData->pose.pose.position.x;
  double ty = voData->pose.pose.position.y;
  double tz = voData->pose.pose.position.z;

//维护一个存放keepVoDataNum个odom值的odom队列
//用来插值得到每帧点云的对应时刻的相机运动估计
  voDataInd = (voDataInd + 1) % keepVoDataNum;
  voDataTime[voDataInd] = time;
  voRx[voDataInd] = rx;
  voRy[voDataInd] = ry;
  voRz[voDataInd] = rz;
  voTx[voDataInd] = tx;
  voTy[voDataInd] = ty;
  voTz[voDataInd] = tz;
}

//订阅雷达深度点云，将图像转化为点云
//根据BA后的odom估计值进行插值，将点云转化到世界坐标系下
void syncCloudHandler(const sensor_msgs::PointCloud2ConstPtr& syncCloud2)
{
//根据startSkipNum跳过一些点的生成
//舍去一定的点
  if (startCount < startSkipNum) {
    startCount++;
    return;
  }

  double time = syncCloud2->header.stamp.toSec();

//获取点云数据
  syncCloud->clear();
  pcl::fromROSMsg(*syncCloud2, *syncCloud);

  double scaleCur = 1;
  double scaleLast = 0;
//voPreInd为最新的odom估计值
  int voPreInd = keepVoDataNum - 1;
//如果vo队列有值
  if (voDataInd >= 0) {
//寻找比当前点云时间戳最接近的更新的vo数据
//当 当前帧点云时间戳>记录的vo时间戳，叠代更新vo数据
    while (voDataTime[voRegInd] <= time && voRegInd != voDataInd) {
//寻找并记录更新的vo值
      voRegInd = (voRegInd + 1) % keepVoDataNum;
    }

//找到当前帧点云对应的前后两帧vo数据，用来线性插值得到当前帧点云对应的相机运动
//相当于voPreInd = (voRegInd - 1)%keepVoDataNum
    voPreInd = (voRegInd + keepVoDataNum - 1) % keepVoDataNum;
    double voTimePre = voDataTime[voPreInd];
    double voTimeReg = voDataTime[voRegInd];

//如果这两帧vo数据时间间隔<0.5s, 则线性插值
    if (voTimeReg - voTimePre < 0.5) {
      double scaleLast =  (voTimeReg - time) / (voTimeReg - voTimePre);
      double scaleCur =  (time - voTimePre) / (voTimeReg - voTimePre);
      if (scaleLast > 1) {
        scaleLast = 1;
      } else if (scaleLast < 0) {
        scaleLast = 0;
      }
      if (scaleCur > 1) {
        scaleCur = 1;
      } else if (scaleCur < 0) {
        scaleCur = 0;
      }
    }
  }

// rx2,ry2,rz2为插值得到的运动旋转值；tx2,ty2,tz2为插值得到的运动偏移量
  double rx2 = voRx[voRegInd] * scaleCur + voRx[voPreInd] * scaleLast;
  double ry2;
  if (voRy[voRegInd] - voRy[voPreInd] > PI) {
    ry2 = voRy[voRegInd] * scaleCur + (voRy[voPreInd] + 2 * PI) * scaleLast;
  } else if (voRy[voRegInd] - voRy[voPreInd] < -PI) {
    ry2 = voRy[voRegInd] * scaleCur + (voRy[voPreInd] - 2 * PI) * scaleLast;
  } else {
    ry2 = voRy[voRegInd] * scaleCur + voRy[voPreInd] * scaleLast;
  }
  double rz2 = voRz[voRegInd] * scaleCur + voRz[voPreInd] * scaleLast;

  double tx2 = voTx[voRegInd] * scaleCur + voTx[voPreInd] * scaleLast;
  double ty2 = voTy[voRegInd] * scaleCur + voTy[voPreInd] * scaleLast;
  double tz2 = voTz[voRegInd] * scaleCur + voTz[voPreInd] * scaleLast;

  double cosrx2 = cos(rx2);
  double sinrx2 = sin(rx2);
  double cosry2 = cos(ry2);
  double sinry2 = sin(ry2);
  double cosrz2 = cos(rz2);
  double sinrz2 = sin(rz2);

  pcl::PointXYZ point;
  int syncCloudNum = syncCloud->points.size();
  for (int i = 0; i < syncCloudNum; i++) {
    point = syncCloud->points[i];
    double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
//当点距离相机的距离在0.3米到5米之间，则将该点转换到世界坐标系下，
//点云距离在此范围内的可信度较高
    if (pointDis > 0.3 && pointDis < 5) {
      double x1 = cosrz2 * point.x - sinrz2 * point.y;
      double y1 = sinrz2 * point.x + cosrz2 * point.y;
      double z1 = point.z;

      double x2 = x1;
      double y2 = cosrx2 * y1 + sinrx2 * z1;
      double z2 = -sinrx2 * y1 + cosrx2 * z1;

      point.x = cosry2 * x2 - sinry2 * z2 + tx2;
      point.y = y2 + ty2;
      point.z = sinry2 * x2 + cosry2 * z2 + tz2;

//将转换到世界坐标系下的点云存放在surroundCloud中
      surroundCloud->push_back(point);
    }
  }

//记录显示的点云帧数
  showCount = (showCount + 1) % (showSkipNum + 1);
  if (showCount != showSkipNum) {
    return;
  }

  tempCloud->clear();
  int surroundCloudNum = surroundCloud->points.size();
  for (int i = 0; i < surroundCloudNum; i++) {
    point = surroundCloud->points[i];

    double xDiff = point.x - voTx[voRegInd];
    double yDiff = point.y - voTy[voRegInd];
    double zDiff = point.z - voTz[voRegInd];

//判断点云是否跟与它的时间戳最接近且新的相机的位置差距
    double pointDis = sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);
//50米之内则显示该点云
//tempCloud存放显示的点云
    if (pointDis < 50) {
      tempCloud->push_back(point);
    }
  }

  surroundCloud->clear();
  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
  downSizeFilter.setInputCloud(tempCloud);
  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
//surroundCloud现在存放降采样后的点云
//而后也没有清除，因此，下一帧深度图来后，继续追加在surroundCloud后，通过tempCloud暂存，并将采样，再次放回surroundCloud中，因此surroundCloud存放整体的点云地图。

//但是即使这样，也有很大的点云重复，如大约静止，但相机运算估计有轻微漂移，我们会看到点云，随着漂移，出现多层的漂移重复
  downSizeFilter.filter(*surroundCloud);

  sensor_msgs::PointCloud2 surroundCloud2;
  pcl::toROSMsg(*surroundCloud, surroundCloud2);
  surroundCloud2.header.frame_id = "/camera_init";
  surroundCloud2.header.stamp = syncCloud2->header.stamp;
  surroundCloudPubPointer->publish(surroundCloud2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "registerPointCloud");
  ros::NodeHandle nh;

  ros::Subscriber voDataSub = nh.subscribe<nav_msgs::Odometry> ("/cam2_to_init", 5, voDataHandler);

  ros::Subscriber syncCloudSub = nh.subscribe<sensor_msgs::PointCloud2>
                                 ("/sync_scan_cloud_filtered", 5, syncCloudHandler);

  ros::Publisher surroundCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/surround_cloud", 1);
  surroundCloudPubPointer = &surroundCloudPub;

  ros::spin();

  return 0;
}
