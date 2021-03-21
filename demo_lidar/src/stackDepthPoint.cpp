#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "pointDefinition.h"

const double PI = 3.1415926;

const int keyframeNum = 5;
pcl::PointCloud<DepthPoint>::Ptr depthPoints[keyframeNum];
double depthPointsTime[keyframeNum];
int keyframeCount = 0;
int frameCount = 0;

pcl::PointCloud<DepthPoint>::Ptr depthPointsStacked(new pcl::PointCloud<DepthPoint>());
ros::Publisher *depthPointsPubPointer = NULL;

double lastPubTime = 0;

void depthPointsHandler(const sensor_msgs::PointCloud2ConstPtr& depthPoints2)
{
  frameCount = (frameCount + 1) % 5;
  if (frameCount != 0) {
    return;
  }
//depthPoints由上到下，点云越来越新，depthPoints[0]为最老帧点云
  pcl::PointCloud<DepthPoint>::Ptr depthPointsCur = depthPoints[0];
//清除最老帧点云
  depthPointsCur->clear();
  pcl::fromROSMsg(*depthPoints2, *depthPointsCur);

//将depthPoints循序网上挪，腾出最下位置给最新的点云
  for (int i = 0; i < keyframeNum - 1; i++) {
    depthPoints[i] = depthPoints[i + 1];
    depthPointsTime[i] = depthPointsTime[i + 1];
  }
//depthPoints最后一个位置存放depthPointsCur
  depthPoints[keyframeNum - 1] = depthPointsCur;
  depthPointsTime[keyframeNum - 1] = depthPoints2->header.stamp.toSec();

  keyframeCount++;
//最老的点云的时间戳大于上一次发布stack点云的时间时，即使点云每keyframeNum触发一次发布stack点云
  if (keyframeCount >= keyframeNum && depthPointsTime[0] >= lastPubTime) {
    depthPointsStacked->clear();
	//将keyframeNum帧点云打包到depthPointsStacked中
    for (int i = 0; i < keyframeNum; i++) {
      *depthPointsStacked += *depthPoints[i];
    }

    sensor_msgs::PointCloud2 depthPoints3;
    pcl::toROSMsg(*depthPointsStacked, depthPoints3);
    depthPoints3.header.frame_id = "camera";
    depthPoints3.header.stamp = depthPoints2->header.stamp;
    depthPointsPubPointer->publish(depthPoints3);

    lastPubTime = depthPointsTime[keyframeNum - 1];
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stackDepthPoint");
  ros::NodeHandle nh;

  for (int i = 0; i < keyframeNum; i++) {
    pcl::PointCloud<DepthPoint>::Ptr depthPointsTemp(new pcl::PointCloud<DepthPoint>());
    depthPoints[i] = depthPointsTemp;
  }

  ros::Subscriber depthPointsSub = nh.subscribe<sensor_msgs::PointCloud2>
                                   ("/depth_points_last", 5, depthPointsHandler);

  ros::Publisher depthPointsPub = nh.advertise<sensor_msgs::PointCloud2> ("/depth_points_stacked", 1);
  depthPointsPubPointer = &depthPointsPub;

  ros::spin();

  return 0;
}
