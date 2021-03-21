#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "pointDefinition.h"

const double PI = 3.1415926;

//相机坐标系定义：x:左; y:上; z:前，图像像素: x:右为正; y下为正
const int keepVoDataNum = 30;
// voRx[]~voTz[]保存keepVoDataNum个连续的相机位姿
double voDataTime[keepVoDataNum] = {0};
double voRx[keepVoDataNum] = {0};
double voRy[keepVoDataNum] = {0};
double voRz[keepVoDataNum] = {0};
double voTx[keepVoDataNum] = {0};
double voTy[keepVoDataNum] = {0};
double voTz[keepVoDataNum] = {0};
int voDataInd = -1;
int voRegInd = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr syncCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZI>());

double timeRec = 0;
double rxRec = 0, ryRec = 0, rzRec = 0;
double txRec = 0, tyRec = 0, tzRec = 0;

bool systemInited = false;
double initTime;

int startCount = -1;
const int startSkipNum = 5;

ros::Publisher *depthCloudPubPointer = NULL;

//将depthCloud转换到当前相机坐标系下，降采样发布
void voDataHandler(const nav_msgs::Odometry::ConstPtr& voData)
{
  double time = voData->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = voData->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

//与Rec时刻的旋转差值
  double rx = voData->twist.twist.angular.x - rxRec;
  double ry = voData->twist.twist.angular.y - ryRec;
  double rz = voData->twist.twist.angular.z - rzRec;

//同样偏航防止[-PI,PI]之间的跳变
  if (ry < -PI) {
    ry += 2 * PI;
  } else if (ry > PI) {
    ry -= 2 * PI;
  }

  //与Rec的偏移差值
  double tx = voData->pose.pose.position.x - txRec;
  double ty = voData->pose.pose.position.y - tyRec;
  double tz = voData->pose.pose.position.z - tzRec;

  rxRec = voData->twist.twist.angular.x;
  ryRec = voData->twist.twist.angular.y;
  rzRec = voData->twist.twist.angular.z;

  txRec = voData->pose.pose.position.x;
  tyRec = voData->pose.pose.position.y;
  tzRec = voData->pose.pose.position.z;

  double x1 = cos(yaw) * tx + sin(yaw) * tz;
  double y1 = ty;
  double z1 = -sin(yaw) * tx + cos(yaw) * tz;

  double x2 = x1;
  double y2 = cos(pitch) * y1 - sin(pitch) * z1;
  double z2 = sin(pitch) * y1 + cos(pitch) * z1;

  tx = cos(roll) * x2 + sin(roll) * y2;
  ty = -sin(roll) * x2 + cos(roll) * y2;
  tz = z2;

  voDataInd = (voDataInd + 1) % keepVoDataNum;
  voDataTime[voDataInd] = time;
  voRx[voDataInd] = rx;
  voRy[voDataInd] = ry;
  voRz[voDataInd] = rz;
  voTx[voDataInd] = tx;
  voTy[voDataInd] = ty;
  voTz[voDataInd] = tz;

  double cosrx = cos(rx);
  double sinrx = sin(rx);
  double cosry = cos(ry);
  double sinry = sin(ry);
  double cosrz = cos(rz);
  double sinrz = sin(rz);

  if (time - timeRec < 0.5) {
    pcl::PointXYZI point;
    tempCloud->clear();
    double x1, y1, z1, x2, y2, z2;
    int depthCloudNum = depthCloud->points.size();
    for (int i = 0; i < depthCloudNum; i++) {
      point = depthCloud->points[i];

      x1 = cosry * point.x - sinry * point.z;
      y1 = point.y;
      z1 = sinry * point.x + cosry * point.z;

      x2 = x1;
      y2 = cosrx * y1 + sinrx * z1;
      z2 = -sinrx * y1 + cosrx * z1;

      point.x = cosrz * x2 + sinrz * y2 - tx;
      point.y = -sinrz * x2 + cosrz * y2 - ty;
      point.z = z2 - tz;

      double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      double timeDis = time - initTime - point.intensity;
      if (fabs(point.x / point.z) < 2 && fabs(point.y / point.z) < 1 && point.z > 0.5 && pointDis < 15 &&
          timeDis < 5.0) {
        tempCloud->push_back(point);
      }
    }

	//体素过滤点云
    depthCloud->clear();
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(tempCloud);
	//点云占据体素最小尺寸为边长0.05m的立方体，1个边长0.05m的立方体只保存一个点
    downSizeFilter.setLeafSize(0.05, 0.05, 0.05);
    downSizeFilter.filter(*depthCloud);
    depthCloudNum = depthCloud->points.size();

    tempCloud->clear();
    for (int i = 0; i < depthCloudNum; i++) {
      point = depthCloud->points[i];

	  //只保留相机前面的
      if (fabs(point.x / point.z) < 1 && fabs(point.y / point.z) < 0.6) {
        point.intensity = depthCloud->points[i].z;
        point.x *= 10 / depthCloud->points[i].z;
        point.y *= 10 / depthCloud->points[i].z;
        point.z = 10;

        tempCloud->push_back(point);
      }
    }

    tempCloud2->clear();
    downSizeFilter.setInputCloud(tempCloud);
    downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilter.filter(*tempCloud2);
    int tempCloud2Num = tempCloud2->points.size();

    for (int i = 0; i < tempCloud2Num; i++) {
      tempCloud2->points[i].z = tempCloud2->points[i].intensity;
      tempCloud2->points[i].x *= tempCloud2->points[i].z / 10;
      tempCloud2->points[i].y *= tempCloud2->points[i].z / 10;
      tempCloud2->points[i].intensity = 10;
    }

    sensor_msgs::PointCloud2 depthCloud2;
    pcl::toROSMsg(*tempCloud2, depthCloud2);
    depthCloud2.header.frame_id = "camera2";
    depthCloud2.header.stamp = voData->header.stamp;
    depthCloudPubPointer->publish(depthCloud2);
  }

  timeRec = time;
}

//当得到的点云坐标转换到最近时刻的相机坐标系下，再添加到depthCloud中
void syncCloudHandler(const sensor_msgs::PointCloud2ConstPtr& syncCloud2)
{
  //跳过一些点云帧
  if (startCount < startSkipNum) {
    startCount++;
    return;
  }

  //设置初始时间
  if (!systemInited) {
    initTime = syncCloud2->header.stamp.toSec();
    systemInited = true;
  }

  double time = syncCloud2->header.stamp.toSec();
  double timeLasted = time - initTime;

  syncCloud->clear();
  pcl::fromROSMsg(*syncCloud2, *syncCloud);

  double scale = 0;
  int voPreInd = keepVoDataNum - 1;
  if (voDataInd >= 0) {
    while (voDataTime[voRegInd] <= time && voRegInd != voDataInd) {
      voRegInd = (voRegInd + 1) % keepVoDataNum;
    }

    voPreInd = (voRegInd + keepVoDataNum - 1) % keepVoDataNum;
    double voTimePre = voDataTime[voPreInd];
    double voTimeReg = voDataTime[voRegInd];

    if (voTimeReg - voTimePre < 0.5) {
      double scale =  (voTimeReg - time) / (voTimeReg - voTimePre);
      if (scale > 1) {
        scale = 1;
      } else if (scale < 0) {
        scale = 0;
      }
    }
  }
  ////通过插值得到与点云对应的坐标系,这个坐标系下保存的rx2~rz2,tx2~tz2指的是点云对应的帧与voRegInd指向的帧之间的R,T关系
  double rx2 = voRx[voRegInd] * scale;
  double ry2 = voRy[voRegInd] * scale;
  double rz2 = voRz[voRegInd] * scale;

  double tx2 = voTx[voRegInd] * scale;
  double ty2 = voTy[voRegInd] * scale;
  double tz2 = voTz[voRegInd] * scale;

  double cosrx2 = cos(rx2);
  double sinrx2 = sin(rx2);
  double cosry2 = cos(ry2);
  double sinry2 = sin(ry2);
  double cosrz2 = cos(rz2);
  double sinrz2 = sin(rz2);

  pcl::PointXYZI point;
  double x1, y1, z1, x2, y2, z2;
  int syncCloudNum = syncCloud->points.size();
  for (int i = 0; i < syncCloudNum; i++) {
    point.x = syncCloud->points[i].x;
    point.y = syncCloud->points[i].y;
    point.z = syncCloud->points[i].z;
    point.intensity = timeLasted;

    x1 = cosry2 * point.x - sinry2 * point.z;
    y1 = point.y;
    z1 = sinry2 * point.x + cosry2 * point.z;

    x2 = x1;
    y2 = cosrx2 * y1 + sinrx2 * z1;
    z2 = -sinrx2 * y1 + cosrx2 * z1;

    point.x = cosrz2 * x2 + sinrz2 * y2 - tx2;
    point.y = -sinrz2 * x2 + cosrz2 * y2 - ty2;
    point.z = z2 - tz2;

    if (voDataInd >= 0) {
      int voAftInd = (voRegInd + 1) % keepVoDataNum;
      while (voAftInd != (voDataInd + 1) % keepVoDataNum) {
        double rx = voRx[voAftInd];
        double ry = voRy[voAftInd];
        double rz = voRz[voAftInd];

        double tx = voTx[voAftInd];
        double ty = voTy[voAftInd];
        double tz = voTz[voAftInd];

        double cosrx = cos(rx);
        double sinrx = sin(rx);
        double cosry = cos(ry);
        double sinry = sin(ry);
        double cosrz = cos(rz);
        double sinrz = sin(rz);

        x1 = cosry * point.x - sinry * point.z;
        y1 = point.y;
        z1 = sinry * point.x + cosry * point.z;

        x2 = x1;
        y2 = cosrx * y1 + sinrx * z1;
        z2 = -sinrx * y1 + cosrx * z1;

        point.x = cosrz * x2 + sinrz * y2 - tx;
        point.y = -sinrz * x2 + cosrz * y2 - ty;
        point.z = z2 - tz;

        voAftInd = (voAftInd + 1) % keepVoDataNum;
      }
    }

    double pointDis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (fabs(point.x / point.z) < 2 && fabs(point.y / point.z) < 1.5 && point.z > 0.5 && pointDis < 15) {
      depthCloud->push_back(point);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "processDepthmap");
  ros::NodeHandle nh;

  ros::Subscriber voDataSub = nh.subscribe<nav_msgs::Odometry> ("/cam_to_init", 5, voDataHandler);

  ros::Subscriber syncCloudSub = nh.subscribe<sensor_msgs::PointCloud2>
                                 ("/sync_scan_cloud_filtered", 5, syncCloudHandler);

  ros::Publisher depthCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/depth_cloud", 5);
  depthCloudPubPointer = &depthCloudPub;

  ros::spin();

  return 0;
}
