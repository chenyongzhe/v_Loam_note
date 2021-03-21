#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "isam/isam.h"
#include "isam/slam_depthmono.h"
#include "isam/robust.h"

#include "cameraParameters.h"
#include "pointDefinition.h"

//5个关键帧进行一次BA优化
using namespace std;
using namespace isam;
using namespace Eigen;
//关于isam的使用，可以见https://heyijia.blog.csdn.net/article/details/53367268
//不过需：111行再加上graph.push_back(gra[3])
//isam 基于factor graph
//步骤：
//1.创建 factor_graph
//2.添加变量(相机的pose,点云观测值)及初始值
//3.添加factor，factor由测量值构成，如起始点估计、里程计测量(两帧运动之间的运动测量值)、landmark测量值
//4.优化器参数
//5.优化
const double PI = 3.1415926;

const int keyframeNum = 5;
pcl::PointCloud<DepthPoint>::Ptr depthPoints[keyframeNum];
pcl::PointCloud<DepthPoint>::Ptr depthPointsStacked(new pcl::PointCloud<DepthPoint>());

double depthPointsTime;
bool newKeyframe = false;

double rollRec, pitchRec, yawRec;
double txRec, tyRec, tzRec;

double transformBefBA[6] = {0};
double transformAftBA[6] = {0};

void diffRotation(double cx, double cy, double cz, double lx, double ly, double lz, 
                  double &ox, double &oy, double &oz)
{
  double srx = cos(cx)*cos(cy)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx)) 
             - cos(cx)*sin(cy)*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly)) - cos(lx)*cos(lz)*sin(cx);
  ox = -asin(srx);

  double srycrx = cos(cx)*sin(cy)*(cos(ly)*cos(lz) + sin(lx)*sin(ly)*sin(lz)) 
                - cos(cx)*cos(cy)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) - cos(lx)*sin(cx)*sin(lz);
  double crycrx = sin(cx)*sin(lx) + cos(cx)*cos(cy)*cos(lx)*cos(ly) + cos(cx)*cos(lx)*sin(cy)*sin(ly);
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  double srzcrx = cos(cx)*cos(lx)*cos(lz)*sin(cz) - (cos(cz)*sin(cy) 
                - cos(cy)*sin(cx)*sin(cz))*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx)) 
                - (cos(cy)*cos(cz) + sin(cx)*sin(cy)*sin(cz))*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly));
  double crzcrx = (sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx))*(sin(ly)*sin(lz) 
                + cos(ly)*cos(lz)*sin(lx)) + (cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy))*(cos(ly)*sin(lz) 
                - cos(lz)*sin(lx)*sin(ly)) + cos(cx)*cos(cz)*cos(lx)*cos(lz);
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

//得到BA后的transform
//BA是5个关键帧才优化一次，频率比VO输出的里程计慢很多，因此VO的发布时间戳和AftBA是不对应的
//Rec拿到的是VO的输出，而我们知道BefBA和AftBA之间的估计,BefBA是我们筛选后的关键帧的相机位姿输出,对应BA优化后的输出，因为BA优化的变量就是关键帧时间戳
//我们首先可以计算VO Rec与BefBA的运动差值
//再计算Bef与Aft之间的运动差值
//最后为Rec累加上Bef与Aft之间的运动差值，得到优化后的运动估计


//运动偏移
//计算世界坐标系下VO和Bef的运动偏移差值
//运动差值转换到相机坐标系下
//根据通过R_ba补偿得到的R_wc'，将运动偏移差值再转回世界坐标系，直接使用BA后的运动偏移计算，累计上偏移差值，从而得到BA后的运动偏移量估计

//运动旋转
//R_wc:相机Rec坐标系到世界坐标系旋转矩阵
//R_wb:Bef坐标系到世界坐标系旋转矩阵
//R_wa:Aft坐标系到世界坐标系旋转矩阵
//R_ab:Bef坐标系到Aft坐标系旋转矩阵,也即bef相对Aft的误差旋转
//R_wa=R_wb*R_ba -> R_ba=R_wb'*R_wa
//R_wc'=R_wc*R_ba为Rec补偿运动旋转差值
void transformAssociateToBA()
{
  double txDiff = txRec - transformBefBA[3];
  double tyDiff = tyRec - transformBefBA[4];
  double tzDiff = tzRec - transformBefBA[5];

  double x1 = cos(yawRec) * txDiff - sin(yawRec) * tzDiff;
  double y1 = tyDiff;
  double z1 = sin(yawRec) * txDiff + cos(yawRec) * tzDiff;

  double x2 = x1;
  double y2 = cos(pitchRec) * y1 + sin(pitchRec) * z1;
  double z2 = -sin(pitchRec) * y1 + cos(pitchRec) * z1;

  txDiff = cos(rollRec) * x2 + sin(rollRec) * y2;
  tyDiff = -sin(rollRec) * x2 + cos(rollRec) * y2;
  tzDiff = z2;

  double sbcx = sin(pitchRec);
  double cbcx = cos(pitchRec);
  double sbcy = sin(yawRec);
  double cbcy = cos(yawRec);
  double sbcz = sin(rollRec);
  double cbcz = cos(rollRec);

  double sblx = sin(transformBefBA[0]);
  double cblx = cos(transformBefBA[0]);
  double sbly = sin(transformBefBA[1]);
  double cbly = cos(transformBefBA[1]);
  double sblz = sin(transformBefBA[2]);
  double cblz = cos(transformBefBA[2]);

  double salx = sin(transformAftBA[0]);
  double calx = cos(transformAftBA[0]);
  double saly = sin(transformAftBA[1]);
  double caly = cos(transformAftBA[1]);
  double salz = sin(transformAftBA[2]);
  double calz = cos(transformAftBA[2]);

  double srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
             - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
             - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
             - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
             - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  pitchRec = -asin(srx);

  double srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  double crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
                + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  yawRec = atan2(srycrx / cos(pitchRec), crycrx / cos(pitchRec));
  
  double srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
                - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
                - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
                + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
                - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
                + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
                + calx*cblx*salz*sblz);
  double crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
                - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
                + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
                + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
                - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
                - calx*calz*cblx*sblz);
  rollRec = atan2(srzcrx / cos(pitchRec), crzcrx / cos(pitchRec));

	//根据更新后的旋转角度，将差值转回世界坐标系下
  x1 = cos(rollRec) * txDiff - sin(rollRec) * tyDiff;
  y1 = sin(rollRec) * txDiff + cos(rollRec) * tyDiff;
  z1 = tzDiff;

  x2 = x1;
  y2 = cos(pitchRec) * y1 - sin(pitchRec) * z1;
  z2 = sin(pitchRec) * y1 + cos(pitchRec) * z1;

  txDiff = cos(yawRec) * x2 + sin(yawRec) * z2;
  tyDiff = y2;
  tzDiff = -sin(yawRec) * x2 + cos(yawRec) * z2;

	//使用transformAftBA得到的运动偏移，累计上算出的差值，得到Rec优化后的运动偏移
  txRec = transformAftBA[3] + txDiff;
  tyRec = transformAftBA[4] + tyDiff;
  tzRec = transformAftBA[5] + tzDiff;
}

//将点云存放在depthPoints中,depthPoints存放多个关键帧的点云
void depthPointsHandler(const sensor_msgs::PointCloud2ConstPtr& depthPoints2)
{
  depthPointsTime = depthPoints2->header.stamp.toSec();

  depthPointsStacked->clear();
  pcl::fromROSMsg(*depthPoints2, *depthPointsStacked);
  int depthPointsStackedNum = depthPointsStacked->points.size();

//清除depthPoints缓存
  for (int i = 0; i < keyframeNum; i++) {
    depthPoints[i]->clear();
  }

//ind=-2存放相机的运动旋转
//1个depthPointsStackedNum存放keyframeCount关键帧的点云集合
  int keyframeCount = -1;
  for (int i = 0; i < depthPointsStackedNum; i++) {
    if (depthPointsStacked->points[i].ind == -2) {
      keyframeCount++;
    }

    if (keyframeCount >= 0 && keyframeCount < keyframeNum) {
      depthPoints[keyframeCount]->push_back(depthPointsStacked->points[i]);
    }
  }

//每个关键帧的点云数量都要保证超过10，否则enoughPoints设为false
  bool enoughPoints = true;
  for (int i = 0; i < keyframeNum; i++) {
    if (depthPoints[i]->points.size() < 10) {
      enoughPoints = false;
    }
  }

  if (enoughPoints) {
    newKeyframe = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bundleAdjust");
  ros::NodeHandle nh;

  for (int i = 0; i < keyframeNum; i++) {
    pcl::PointCloud<DepthPoint>::Ptr depthPointsTemp(new pcl::PointCloud<DepthPoint>());
    depthPoints[i] = depthPointsTemp;
  }

  ros::Subscriber depthPointsSub = nh.subscribe<sensor_msgs::PointCloud2>
                                   ("/depth_points_stacked", 1, depthPointsHandler);

  ros::Publisher odomBefBAPub = nh.advertise<nav_msgs::Odometry> ("/bef_ba_to_init", 1);

  ros::Publisher odomAftBAPub = nh.advertise<nav_msgs::Odometry> ("/aft_ba_to_init", 1);

  tf::TransformBroadcaster tfBroadcaster;

  Vector2d pp(0, 0);
  DepthmonoCamera camera(1, pp);

  MatrixXd mpNoise = eye(3);
  mpNoise(2, 2) = 0.01;
  Noise noise0 = Information(mpNoise);

  MatrixXd dpNoise = eye(3);
  //dpNoise(2, 2) = 1.;
  Noise noise1 = Information(dpNoise);

  MatrixXd ssNoise = 10000. * eye(6);
  //ssNoise(3, 3) = 100;
  //ssNoise(4, 4) = 100;
  //ssNoise(5, 5) = 100;
  Noise noise2 = Information(ssNoise);

  MatrixXd psNoise = 10000. * eye(6);
  psNoise(3, 3) = 100;
  psNoise(4, 4) = 100;
  psNoise(5, 5) = 100;
  Noise noise3 = Information(psNoise);

  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newKeyframe) {
      newKeyframe = false;
//* 1. ba 为整体的factor graph
      Slam ba;

      vector<Pose3d_Node*> poses;
	//pose为要优化得到的变量，pose0为0点坐标,不参与优化。
      Pose3d_Node* pose0 = new Pose3d_Node();
      poses.push_back(pose0);
      ba.add_node(pose0);

	//每个depthPoints的前两个存放相机在世界坐标系下的旋转和偏移
      rollRec = depthPoints[0]->points[0].depth;
      pitchRec = depthPoints[0]->points[0].u;
      yawRec = depthPoints[0]->points[0].v;
      txRec = depthPoints[0]->points[1].u;
      tyRec = depthPoints[0]->points[1].v;
      tzRec = depthPoints[0]->points[1].depth;

      transformAssociateToBA();

//* 2. 添加起始点factor，即pose0
	//factor为测量值，包括：里程计的输出(两帧之间的运动)，以及起始点，即pose0，pose0不作为变量优化
      Pose3d_Factor* poseFactors0 = new Pose3d_Factor(pose0, 
                                    Pose3d(tzRec, txRec, tyRec, yawRec, pitchRec, rollRec), noise2);
      ba.add_factor(poseFactors0);

      rollRec = depthPoints[0]->points[0].depth;
      pitchRec = depthPoints[0]->points[0].u;
      yawRec = depthPoints[0]->points[0].v;
      txRec = depthPoints[0]->points[1].u;
      tyRec = depthPoints[0]->points[1].v;
      tzRec = depthPoints[0]->points[1].depth;
	//posePoseFactors用来存放里程计factor(相机两帧之间的运动计算)
      vector<Pose3d_Pose3d_Factor*> posePoseFactors;
      for (int i = 1; i < keyframeNum; i++) {
		//node为要优化的变量
        Pose3d_Node* posen = new Pose3d_Node();
//* 3. 添加待优化解算变量，即poses, keyframeNum个pose
        poses.push_back(posen);
        ba.add_node(posen);
		//根据记录的运动，得到两两关键帧之间的运动测量值between_factor
        double roll = depthPoints[i]->points[0].depth;
        double pitch = depthPoints[i]->points[0].u;
        double yaw = depthPoints[i]->points[0].v;
        double tx = depthPoints[i]->points[1].u;
        double ty = depthPoints[i]->points[1].v;
        double tz = depthPoints[i]->points[1].depth;
		//得到Rec对应的相机坐标系下的运动偏移(当前计算的关键帧与Rec关键帧之间的运动偏移)
        double txDiff = tx - txRec;
        double tyDiff = ty - tyRec;
        double tzDiff = tz - tzRec;

		//将两帧之间的运动偏移差转换到世界坐标系下
        double x1 = cos(yawRec) * txDiff - sin(yawRec) * tzDiff;
        double y1 = tyDiff;
        double z1 = sin(yawRec) * txDiff + cos(yawRec) * tzDiff;

        double x2 = x1;
        double y2 = cos(pitchRec) * y1 + sin(pitchRec) * z1;
        double z2 = -sin(pitchRec) * y1 + cos(pitchRec) * z1;

        txDiff = cos(rollRec) * x2 + sin(rollRec) * y2;
        tyDiff = -sin(rollRec) * x2 + cos(rollRec) * y2;
        tzDiff = z2;

		//计算旋转偏差
        double rollDiff, pitchDiff, yawDiff;
        diffRotation(pitch, yaw, roll, pitchRec, yawRec, rollRec, pitchDiff, yawDiff, rollDiff);

        Pose3d_Pose3d_Factor* poseposeFactorn = new Pose3d_Pose3d_Factor
                                                (poses[i - 1], posen, Pose3d(tzDiff, txDiff, tyDiff, 
                                                yawDiff, pitchDiff, rollDiff), noise3);
//* 4. 添加里程计factor，也是keyframeNum个
        posePoseFactors.push_back(poseposeFactorn);
        ba.add_factor(poseposeFactorn);

        rollRec = roll; pitchRec = pitch; yawRec = yaw;
        txRec = tx; tyRec = ty; tzRec = tz;
      }

	//points存放匹配的且有深度的点云变量
      vector<Point3d_Node*> points;
      std::vector<int> pointsInd;
	//点云测量值factor，添加keyframeNum帧的点云数据，每个关键帧添加ptRecNum个点云测量值
      vector<Depthmono_Factor*> depthmonoFactors;
      for (int i = 0; i < keyframeNum; i++) {
        pcl::PointCloud<DepthPoint>::Ptr dpPointer = depthPoints[i];
		//kfptNum 1个关键帧中的点云数量
        int kfptNum = dpPointer->points.size();
		//points初始化时不存放点云，后续将找到的匹配点且有深度的点云存放points
        int ptRecNum = points.size();

		//第一个关键帧，向下对比
        if (i == 0) {
			//dpPointerNext存放下一关键帧点云，用来查找匹配点,头一帧存放的点云是最老的，因此向下关键帧匹配关键点
          pcl::PointCloud<DepthPoint>::Ptr dpPointerNext = depthPoints[i + 1];
          int kfptNumNext = dpPointerNext->points.size();

		//去掉前两个用于存放相机旋转和偏移的points
          int ptCountNext = 2;
          for (int j = 2; j < kfptNum; j++) {
            bool ptFound = false;
            for (; ptCountNext < kfptNumNext; ptCountNext++) {
				//如果找到匹配点
              if (dpPointerNext->points[ptCountNext].ind == dpPointer->points[j].ind) {
                ptFound = true;
              }
              if (dpPointerNext->points[ptCountNext].ind >= dpPointer->points[j].ind) {
                break;
              }
            }

			//如果找到匹配点，且通过深度匹配到深度,找到一个深度匹配点，则在factor中新增一个pointn
            if (ptFound && dpPointer->points[j].label == 1) {
              Point3d_Node* pointn = new Point3d_Node();
			//为深度匹配的点云分配一个变量空间
              points.push_back(pointn);
			//pointsInd存放匹配的点云id
              pointsInd.push_back(dpPointer->points[j].ind);
//* 5. 添加点云坐标变量,添加keyframeNum帧的点云数据，每个关键帧添加ptRecNum个点云测量值
				//pointn为待优化变量，未赋值
              ba.add_node(pointn);

				//关键帧相机坐标下的点云观测值
              Depthmono_Factor* depthmonoFactorn = new Depthmono_Factor(poses[i], pointn, &camera,
                                                   DepthmonoMeasurement(dpPointer->points[j].u,
                                                   dpPointer->points[j].v, dpPointer->points[j].depth),
                                                   noise1);
              depthmonoFactors.push_back(depthmonoFactorn);
//* 6. 添加点云观测值factor,添加keyframeNum帧的点云数据，每个关键帧添加ptRecNum个点云测量值
              ba.add_factor(depthmonoFactorn);
            }
          }
		//最后一个关键帧，向上对比
        } else if (i == keyframeNum - 1) {
			//向上(old)关键帧查找匹配点
          pcl::PointCloud<DepthPoint>::Ptr dpPointerLast = depthPoints[i - 1];
          int kfptNumLast = dpPointerLast->points.size();

          int ptCountLast = 2;
          int ptRecCount = 0;
          for (int j = 2; j < kfptNum; j++) {
            bool ptFound = false;
            for (; ptCountLast < kfptNumLast; ptCountLast++) {
              if (dpPointerLast->points[ptCountLast].ind == dpPointer->points[j].ind) {
                ptFound = true;
              }
              if (dpPointerLast->points[ptCountLast].ind >= dpPointer->points[j].ind) {
                break;
              }
            }

			//dpPointer->points中匹配last帧的点云中再寻找与pointsInd中存放的匹配点云id相匹配的点云
			//即寻找该关键帧中共同的点云，然后prFound置为true，
            if (ptFound /*&& dpPointer->points[j].label == 1*/) {
              bool prFound = false;
              for (; ptRecCount < ptRecNum; ptRecCount++) {
                if (pointsInd[ptRecCount] == dpPointer->points[j].ind) {
                  prFound = true;
                }
                if (pointsInd[ptRecCount] >= dpPointer->points[j].ind) {
                  break;
                }
              }

              Point3d_Node* pointn;
              Depthmono_Factor* depthmonoFactorn;
              if (prFound) {
				//重复的点云，只添加其观测值factor，而不另外开辟变量空间
                pointn = points[ptRecCount];
                depthmonoFactorn = new Depthmono_Factor(poses[i], pointn, &camera,
                                   DepthmonoMeasurement(dpPointer->points[j].u,
                                   dpPointer->points[j].v, dpPointer->points[j].depth), noise0);
                //continue;
              } else {
                pointn = new Point3d_Node();
				//防止重复为相同点云开辟变量空间
                points.push_back(pointn);
                pointsInd.push_back(dpPointer->points[j].ind);
                ba.add_node(pointn);
                depthmonoFactorn = new Depthmono_Factor(poses[i], pointn, &camera,
                                   DepthmonoMeasurement(dpPointer->points[j].u,
                                   dpPointer->points[j].v, dpPointer->points[j].depth), noise1);
              }

              depthmonoFactors.push_back(depthmonoFactorn);
              ba.add_factor(depthmonoFactorn);
            }
          }
		//其他关键帧，上下对比
        } else {
          pcl::PointCloud<DepthPoint>::Ptr dpPointerNext = depthPoints[i + 1];
          pcl::PointCloud<DepthPoint>::Ptr dpPointerLast = depthPoints[i - 1];
          int kfptNumNext = dpPointerNext->points.size();
          int kfptNumLast = dpPointerLast->points.size();

          int ptCountNext = 2;
          int ptCountLast = 2;
          int ptRecCount = 0;
          for (int j = 2; j < kfptNum; j++) {
            bool ptFound = false;
			//先向下关键帧找匹配点
            for (; ptCountNext < kfptNumNext; ptCountNext++) {
              if (dpPointerNext->points[ptCountNext].ind == dpPointer->points[j].ind) {
                ptFound = true;
              }
              if (dpPointerNext->points[ptCountNext].ind >= dpPointer->points[j].ind) {
                break;
              }
            }

			//如果向下没有找到匹配，则再向上找
            if (!ptFound) {
              for (; ptCountLast < kfptNumLast; ptCountLast++) {
                if (dpPointerLast->points[ptCountLast].ind == dpPointer->points[j].ind) {
                  ptFound = true;
                }
                if (dpPointerLast->points[ptCountLast].ind >= dpPointer->points[j].ind) {
                  break;
                }
              }
            }

			//同样如果找到了，查看该点是否已经被登记(Rec)过了(prFound)
            if (ptFound /*&& dpPointer->points[j].label == 1*/) {
              bool prFound = false;
              for (; ptRecCount < ptRecNum; ptRecCount++) {
                if (pointsInd[ptRecCount] == dpPointer->points[j].ind) {
                  prFound = true;
                }
                if (pointsInd[ptRecCount] >= dpPointer->points[j].ind) {
                    break;
                }
              }

              Point3d_Node* pointn;
              Depthmono_Factor* depthmonoFactorn;
			//同样，被登记过的点值只添加观测factor，而不增加变量
              if (prFound) {
                pointn = points[ptRecCount];

                depthmonoFactorn = new Depthmono_Factor(poses[i], pointn, &camera,
                                   DepthmonoMeasurement(dpPointer->points[j].u,
                                   dpPointer->points[j].v, dpPointer->points[j].depth), noise0);
                //continue;
              } else {
                pointn = new Point3d_Node();
                points.push_back(pointn);
                pointsInd.push_back(dpPointer->points[j].ind);
                ba.add_node(pointn);

                depthmonoFactorn = new Depthmono_Factor(poses[i], pointn, &camera,
                                   DepthmonoMeasurement(dpPointer->points[j].u,
                                   dpPointer->points[j].v, dpPointer->points[j].depth), noise1);
              }

              depthmonoFactors.push_back(depthmonoFactorn);
              ba.add_factor(depthmonoFactorn);
            }
          }
        }
      }

//* 6. 配置优化器参数
      Properties prop = ba.properties();
      prop.method = DOG_LEG;
      ba.set_properties(prop);
//* 7. 优化
      ba.batch_optimization();

	//BA前的运动估计，最后一个关键帧点云中头两个存放了该时刻相机在世界坐标系下的旋转和偏移
      transformBefBA[0] = depthPoints[keyframeNum - 1]->points[0].u;
      transformBefBA[1] = depthPoints[keyframeNum - 1]->points[0].v;
      transformBefBA[2] = depthPoints[keyframeNum - 1]->points[0].depth;
      transformBefBA[3] = depthPoints[keyframeNum - 1]->points[1].u;
      transformBefBA[4] = depthPoints[keyframeNum - 1]->points[1].v;
      transformBefBA[5] = depthPoints[keyframeNum - 1]->points[1].depth;

	//BA优化后的运动估计，poses存放变量
	//BA优化后的最后一帧的运动估计
      transformAftBA[0] = poses[keyframeNum - 1]->value().pitch();
      transformAftBA[1] = poses[keyframeNum - 1]->value().yaw();
      transformAftBA[2] = poses[keyframeNum - 1]->value().roll();
      transformAftBA[3] = poses[keyframeNum - 1]->value().y();
      transformAftBA[4] = poses[keyframeNum - 1]->value().z();
      transformAftBA[5] = poses[keyframeNum - 1]->value().x();

	//作者设了一个置信权重，BAbelW为BA的置信权重
	  float BAbelW = 0.5;
      transformAftBA[0] = (1 - BAbelW) * transformAftBA[0] + BAbelW * transformBefBA[0];
      //transformAftBA[1] = (1 - 0.1) * transformAftBA[1] + 0.1 * transformBefBA[1];
      transformAftBA[2] = (1 - BAbelW) * transformAftBA[2] + BAbelW * transformBefBA[2];

	//清除factor及变量等
      int posesNum = poses.size();
      for (int i = 1; i < posesNum; i++) {
        delete poses[i];
      }
      poses.clear();

      delete poseFactors0;

      int posePoseFactorsNum = posePoseFactors.size();
      for (int i = 1; i < posePoseFactorsNum; i++) {
        delete posePoseFactors[i];
      }
      posePoseFactors.clear();

      int pointsNum = points.size();
      for (int i = 1; i < pointsNum; i++) {
        delete points[i];
      }
      points.clear();

      int depthmonoFactorsNum = depthmonoFactors.size();
      for (int i = 1; i < depthmonoFactorsNum; i++) {
        delete depthmonoFactors[i];
      }
      depthmonoFactors.clear();

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                (transformBefBA[2], -transformBefBA[0], -transformBefBA[1]);

	//发布BA前运动估计
      nav_msgs::Odometry odomBefBA;
      odomBefBA.header.frame_id = "/camera_init";
      odomBefBA.child_frame_id = "/bef_ba";
      odomBefBA.header.stamp = ros::Time().fromSec(depthPointsTime);
      odomBefBA.pose.pose.orientation.x = -geoQuat.y;
      odomBefBA.pose.pose.orientation.y = -geoQuat.z;
      odomBefBA.pose.pose.orientation.z = geoQuat.x;
      odomBefBA.pose.pose.orientation.w = geoQuat.w;
      odomBefBA.pose.pose.position.x = transformBefBA[3];
      odomBefBA.pose.pose.position.y = transformBefBA[4];
      odomBefBA.pose.pose.position.z = transformBefBA[5];
      odomBefBAPub.publish(odomBefBA);

      geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                (transformAftBA[2], -transformAftBA[0], -transformAftBA[1]);
	//发布BA后运动估计
      nav_msgs::Odometry odomAftBA;
      odomAftBA.header.frame_id = "/camera_init";
      odomAftBA.child_frame_id = "/aft_ba";
      odomAftBA.header.stamp = ros::Time().fromSec(depthPointsTime);
      odomAftBA.pose.pose.orientation.x = -geoQuat.y;
      odomAftBA.pose.pose.orientation.y = -geoQuat.z;
      odomAftBA.pose.pose.orientation.z = geoQuat.x;
      odomAftBA.pose.pose.orientation.w = geoQuat.w;
      odomAftBA.pose.pose.position.x = transformAftBA[3];
      odomAftBA.pose.pose.position.y = transformAftBA[4];
      odomAftBA.pose.pose.position.z = transformAftBA[5];
      odomAftBAPub.publish(odomAftBA);

      tf::StampedTransform tfAftBA;
      tfAftBA.frame_id_ = "/camera_init";
      tfAftBA.child_frame_id_ = "/aft_ba";
      tfAftBA.stamp_ = ros::Time().fromSec(depthPointsTime);
      tfAftBA.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      tfAftBA.setOrigin(tf::Vector3(transformAftBA[3], 
                            transformAftBA[4], transformAftBA[5]));
		//广播相机aft_ba tf
      tfBroadcaster.sendTransform(tfAftBA);
    }

    status = ros::ok();
    cv::waitKey(10);
  }

  return 0;
}
