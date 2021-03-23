#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "cameraParameters.h"
#include "pointDefinition.h"

using namespace std;
const double PI = 3.1415926;

//相机坐标系定义：x:左; y:上; z:前，图像像素: x:右为正; y下为正
pcl::PointCloud<ImagePoint>::Ptr imagePointsCur(new pcl::PointCloud<ImagePoint>());
pcl::PointCloud<ImagePoint>::Ptr imagePointsLast(new pcl::PointCloud<ImagePoint>());
pcl::PointCloud<ImagePoint>::Ptr startPointsCur(new pcl::PointCloud<ImagePoint>());
pcl::PointCloud<ImagePoint>::Ptr startPointsLast(new pcl::PointCloud<ImagePoint>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr startTransCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr startTransLast(new pcl::PointCloud<pcl::PointXYZHSV>());
//ipRelations存放两帧匹配点的归一化坐标
pcl::PointCloud<pcl::PointXYZHSV>::Ptr ipRelations(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr ipRelations2(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZ>::Ptr imagePointsProj(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<DepthPoint>::Ptr depthPointsCur(new pcl::PointCloud<DepthPoint>());
pcl::PointCloud<DepthPoint>::Ptr depthPointsLast(new pcl::PointCloud<DepthPoint>());
pcl::PointCloud<DepthPoint>::Ptr depthPointsSend(new pcl::PointCloud<DepthPoint>());

std::vector<int> ipInd;
std::vector<float> ipy2;

//只存放三角化测量获得的图像深度值
std::vector<float>* ipDepthCur = new std::vector<float>();
std::vector<float>* ipDepthLast = new std::vector<float>();

double imagePointsCurTime;
double imagePointsLastTime;

int imagePointsCurNum = 0;
int imagePointsLastNum = 0;

int depthPointsCurNum = 0;
int depthPointsLastNum = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr depthCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdTree(new pcl::KdTreeFLANN<pcl::PointXYZI>());

double depthCloudTime;
int depthCloudNum = 0;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqrDis;

//世界坐标系下偏移和旋转累积值
//论文求解的RT为将点云从第k-1帧映射到第k帧的变换
//因此从第k-1帧映射到第k帧相机的位姿变换与RT反向
//transformSum,angleSum存放当前帧与初始帧之间的转移矩阵
double transformSum[6] = {0};
double angleSum[3] = {0};

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;
bool imuInited = false;

double imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
double imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;

double imuYawInit = 0;
double imuTime[imuQueLength] = {0};
double imuRoll[imuQueLength] = {0};
double imuPitch[imuQueLength] = {0};
double imuYaw[imuQueLength] = {0};

ros::Publisher *voDataPubPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *depthPointsPubPointer = NULL;
ros::Publisher *imagePointsProjPubPointer = NULL;
ros::Publisher *imageShowPubPointer;

const int showDSRate = 2;

//计算相机在世界坐标下的累积旋转ox，oy，oz
//lx,ly,lz是帧与帧之间相机的欧拉角旋转角度
//cx,cy,cz是上一帧计算的相机在世界坐标系下的欧拉旋转角
//ox,oy,oz是当前计算的相机在世界坐标系下的欧拉旋转角输出
//由于在解算时，并非求的上一帧到下帧之间的旋转矩阵R{(i-1)to i}，我们由论文公式(3-6)求出的transform[0-5]是将上一帧坐标系下的点映射到下一帧坐标系下的变换矩阵对应的偏移和旋转欧拉角，
//设transform对应的旋转阵为R:{rpy}，因此，下一帧坐标系基于上一帧坐标系旋转矩阵R{(i-1)to i}=R^{-1}={-y,-p,-r}。同理变换矩阵T也是
//因此相机的累计旋转矩阵为Ro=R^{-1}*Rc
//其中：Rc=[ccy 0 scy;0 1 0;-scy 0 ccy]*[1 0 0;0 ccx -scx;0 scx ccx]*[ccz -scz 0;scz ccz 0;0 0 1]

//R{(i-1)to i}=R^{-1}=[clz -slz 0;slz clz 0;0 0 1]*[1 0 0;0 clx -slx;0 slx clx]*[cly 0 sly;0 1 0;-sly 0 cly]
void accumulateRotation(double cx, double cy, double cz, double lx, double ly, double lz, 
                        double &ox, double &oy, double &oz)
{
  double srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  double srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
                + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
  double crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
                - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  double srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
                + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
  double crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
                - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

//比较imu和解算得出的相机旋转累积的差值
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
//接收feature tracking 提取的last的features points上一帧特征点归一化图像坐标
//根据论文公式(3,4)(6)来解算d位置估计
// transform 存储两帧之间的运动估计矩阵
//先给点云匹配深度，分别通过激光点云深度匹配(v=1)及三角测距方法(v=2)
//再通过上帧图像点及匹配深度、本帧点云通过论文公式(3,4,6)及高斯牛顿法计算得transform
void imagePointsHandler(const sensor_msgs::PointCloud2ConstPtr& imagePoints2)
{
  imagePointsLastTime = imagePointsCurTime;
  imagePointsCurTime = imagePoints2->header.stamp.toSec();

  imuRollLast = imuRollCur;
  imuPitchLast = imuPitchCur;
  imuYawLast = imuYawCur;

  double transform[6] = {0.0};
	//提取imu数据,imuPointerFront与点云对应的imu数据
  if (imuPointerLast >= 0) {//如果imu有数据
	//遍历imuTime队列，找到比当前image发布时间快的imu id后退出，或者遍历完
    while (imuPointerFront != imuPointerLast) {
	  //当前点云时间<imu数据对应的时间，则不更新imuPointerFront
      if (imagePointsCurTime < imuTime[imuPointerFront]) {
        break;
      }
	  //更新imuPointerFront的值为队列中的下一个值
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

	//imuxxxCur存放的imu在世界坐标系下的旋转，由于yaw的变化范围会超过(-pi,pi)，可能出现-pi~pi之间的跳变，因此yaw的初始值需单独存放
	//如果imu的最新消息比image的还要慢，一般imu的发布频率远高于图像，先将最新的消息赋值给imuxxxCur
    if (imagePointsCurTime > imuTime[imuPointerFront]) {
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];
    } else {
		//找到比image新的imu消息，则线性插值得到发布的image时的imu估计值
		//线性插值，得到对应时间的imu数据
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      double ratioFront = (imagePointsCurTime - imuTime[imuPointerBack]) 
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      double ratioBack = (imuTime[imuPointerFront] - imagePointsCurTime) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
	  //从四元数解析到欧拉角的范围为[-PI, PI],因此会出现-PI与PI之间的跳变.roll和pitch幅度较小，不会出现跳变，但是yaw由于累积会超越PI，-PI边界值。
      //以下代码是解决yaw的跳变问题。
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
      } else {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }
    }

    if (imuInited) {
	  //imuPitchCur - imuPitchLast得到两帧图像之间imu的旋转参数
	  //transform[0]~transform[2]存放两帧旋转累积
      transform[0] = imuPitchCur - imuPitchLast;
      transform[1] = imuYawCur - imuYawLast;
      transform[2] = imuRollCur - imuRollLast;
    }
  }

	//将上一帧留存的xxCur赋值给xxLast，并将新得的值付给xxCur
  pcl::PointCloud<ImagePoint>::Ptr imagePointsTemp = imagePointsLast;
  imagePointsLast = imagePointsCur;
  imagePointsCur = imagePointsTemp;

  imagePointsCur->clear();
  pcl::fromROSMsg(*imagePoints2, *imagePointsCur);

  imagePointsLastNum = imagePointsCurNum;
  cout << "imagePointsLastNum:" << imagePointsCurNum << endl;
  imagePointsCurNum = imagePointsCur->points.size();
  cout << "imagePointsCurNum:" << imagePointsCurNum << endl;

  pcl::PointCloud<ImagePoint>::Ptr startPointsTemp = startPointsLast;
  startPointsLast = startPointsCur;
  startPointsCur = startPointsTemp;

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr startTransTemp = startTransLast;
  startTransLast = startTransCur;
  startTransCur = startTransTemp;
  //ipDepthLast存放三角测量的深度值
  std::vector<float>* ipDepthTemp = ipDepthLast;
  ipDepthLast = ipDepthCur;
  ipDepthCur = ipDepthTemp;

  int j = 0;
  pcl::PointXYZI ips;
  pcl::PointXYZHSV ipr;
  ipRelations->clear();
  ipInd.clear();
  //循环的数量是上一帧的点云数量，也就是说它是基于上一帧的特征点对本帧的搜索
  for (int i = 0; i < imagePointsLastNum; i++) {
    bool ipFound = false;//如果本帧中的共视关系不存在，则忽略不计算在内
    for (; j < imagePointsCurNum; j++) {
	  //查找是否有匹配的特征点
      if (imagePointsCur->points[j].ind == imagePointsLast->points[i].ind) {
		cout << "find a match point:imagePointsCur->points:" << imagePointsCur->points[j].ind <<endl;
        ipFound = true;
      }
      if (imagePointsCur->points[j].ind >= imagePointsLast->points[i].ind) {
        break;
      }
    }

    if (ipFound) {
		//找到匹配点
      ROS_INFO ("ip found! ******************");
      ROS_INFO ("last_point: u:%f v:%f\tcurr_point: u:%f v:%f",
		      imagePointsLast->points[i].u,
		      imagePointsLast->points[i].v,
		      imagePointsCur->points[j].u,
		      imagePointsCur->points[j].v);

		//ipr: image point relative
		//ipr: x,y对应上一帧图像点归一化坐标；z,h对应匹配上一帧xy的当前帧的图像点归一化坐标
		//ipr: s存放计算出的图像点的深度，v为通过哪种方法获得的图像点的深度:
		//ipr.v=1:通过激光点云计算获得图像点深度
      ipr.x = imagePointsLast->points[i].u;
      ipr.y = imagePointsLast->points[i].v;
      ipr.z = imagePointsCur->points[j].u;
      ipr.h = imagePointsCur->points[j].v;

		//ips存放last特征点坐标，将点归一化到深度为10的平面上，与激光激光采集到的深度点云对应
		//imagePointsLast->points是归一化后的图像点，即深度为1的平面上
		// ips: image point source
      ips.x = 10 * ipr.x;
      ips.y = 10 * ipr.y;
      ips.z = 10;

		//先为特征点查找深度，第一先查找深度图能否匹配特征点，第二再根据得到的transform来估计深度。
		//有深度图进行处理，为特征点匹配深度信息
		//只有last特征点才有深度
      if (depthCloudNum > 10) {
          ROS_INFO ("depthCloudNum over 10 *******");
		//深度特征匹配，查找ips的最近邻的3个值
		//kdTree 中存放深度点云
		//将图像点云及激光点云都放在深度为10的平面上，找到图像点最临近的激光点云
        kdTree->nearestKSearch(ips, 3, pointSearchInd, pointSearchSqrDis);

        double minDepth, maxDepth;
		//ips与匹配点的距离s小于阈值，且匹配了3个深度特征点
        if (pointSearchSqrDis[0] < 0.5 && pointSearchInd.size() == 3) {
			//恢复第1个特征点的原始坐标
			//x1y1z1,x2y1z2,x3y3z3存放3个激光邻近点的原始坐标
			//恢复原始点坐标
          pcl::PointXYZI depthPoint = depthCloud->points[pointSearchInd[0]];
          double x1 = depthPoint.x * depthPoint.intensity / 10;
          double y1 = depthPoint.y * depthPoint.intensity / 10;
          double z1 = depthPoint.intensity;
          minDepth = z1;
          maxDepth = z1;

			//恢复第2个特征点的原始坐标
          depthPoint = depthCloud->points[pointSearchInd[1]];
          double x2 = depthPoint.x * depthPoint.intensity / 10;
          double y2 = depthPoint.y * depthPoint.intensity / 10;
          double z2 = depthPoint.intensity;
          minDepth = (z2 < minDepth)? z2 : minDepth;
          maxDepth = (z2 > maxDepth)? z2 : maxDepth;

			//恢复第3个特征点的原始坐标
          depthPoint = depthCloud->points[pointSearchInd[2]];
          double x3 = depthPoint.x * depthPoint.intensity / 10;
          double y3 = depthPoint.y * depthPoint.intensity / 10;
          double z3 = depthPoint.intensity;
          minDepth = (z3 < minDepth)? z3 : minDepth;
          maxDepth = (z3 > maxDepth)? z3 : maxDepth;
          //imagePointsLast->points[i]的u v
          double u = ipr.x;
          double v = ipr.y;
			//由论文公式(10)求解获得ipr.s 利用(Xs-X1)((X1-X2)x(X1-X3))=0求解z,Xs=z[u,v,1]T
			//ipr.s为求解的图像深度值
          ipr.s = (x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1) 
                / (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2 + u*y1*z2 - u*y2*z1
                - v*x1*z2 + v*x2*z1 - u*y1*z3 + u*y3*z1 + v*x1*z3 - v*x3*z1 + u*y2*z3 
                - u*y3*z2 - v*x2*z3 + v*x3*z2);
			//使用激光点云找到图像对应深度值，设置ipr.v=1
          ipr.v = 1;

			//如果最临近三个点的的深度差值超过2m，说明这3个邻近点找的点不可靠,ipr.v恢复为0
      ////当三点深度差距过大则说明投影附近存在遮挡关系或物体边缘，放弃
          if (maxDepth - minDepth > 2) {
            ipr.s = 0;
            ipr.v = 0;
			//计算特征点的深度大于3个点的最大深度，则将最大深度赋值给点
          } else if (ipr.s - maxDepth > 0.2) {
            ipr.s = maxDepth;
			//计算特征点的深度小于3个点的最小深度，则将最小深度赋值给点
          } else if (ipr.s - minDepth < -0.2) {
            ipr.s = minDepth;
          }
		//其他没有找到匹配的激光深度点都将ipr.s ipr.v设为0
        } else {
          ipr.s = 0;
          ipr.v = 0;
        }
      } else {
        ipr.s = 0;
        ipr.v = 0;
      }

	//如果没有获取到深度,且当特征点在相机运动一定距离后，仍被匹配到，则利用三角化计算出该点深度
	//利用第一次发现该特征点时刻相机的运动位姿及归一化图像坐标，和当前时刻该点的运动位姿及归一化图像坐标计算该点深度
	//首先利用第一次发现该特征点时刻相机的运动位姿及归一化图像坐标将点云转化到世界坐标系下，
  //再利用当前时刻该点的运动位姿及归一化图像坐标将该点映射在当前相机坐标系下
	//这里坐标系的切换只进行了旋转变换，到最后才进行偏移量的计算
      if (fabs(ipr.v) < 0.5) {
		//transformSum 是当前帧相机的累积运动估计
		//startTransLast存放点云第一次匹配到时的相机运动估计
        double disX = transformSum[3] - startTransLast->points[i].h;
        double disY = transformSum[4] - startTransLast->points[i].s;
        double disZ = transformSum[5] - startTransLast->points[i].v;

	cout << "dis:" << disX << disY << disZ << endl;
	cout << "startTransLast->points:" << startTransLast->points[i].h << startTransLast->points[i].s << startTransLast->points[i].v << endl;

		//判断该点被匹配的距离是否已经超过阈值
        if (sqrt(disX * disX + disY * disY + disZ * disZ) > 1) {
		//TODO:这里需加上时间阈值，从而可以使无深度点云输入时，单目利用三角测量也能估计运动
        //if (sqrt(disX * disX + disY * disY + disZ * disZ) > 1) {

          double u0 = startPointsLast->points[i].u;
          double v0 = startPointsLast->points[i].v;
          double u1 = ipr.x;
          double v1 = ipr.y;
		//加-号是将相机运动估计转成变换矩阵
		//如，相机的位姿表示为T，将点云从世界坐标系下转换到当前相机坐标系下的变换矩阵则为T^{-1}
          //本帧相对原点的旋转
          double srx0 = sin(-startTransLast->points[i].x);
          double crx0 = cos(-startTransLast->points[i].x);
          double sry0 = sin(-startTransLast->points[i].y);
          double cry0 = cos(-startTransLast->points[i].y);
          double srz0 = sin(-startTransLast->points[i].z);
          double crz0 = cos(-startTransLast->points[i].z);
          //上一帧相对原点的旋转
          double srx1 = sin(-transformSum[0]);
          double crx1 = cos(-transformSum[0]);
          double sry1 = sin(-transformSum[1]);
          double cry1 = cos(-transformSum[1]);
          double srz1 = sin(-transformSum[2]);
          double crz1 = cos(-transformSum[2]);
          //本帧相对原点的位移
          double tx0 = -startTransLast->points[i].h;
          double ty0 = -startTransLast->points[i].s;
          double tz0 = -startTransLast->points[i].v;
           //上一帧相对原点的位移
          double tx1 = -transformSum[3];
          double ty1 = -transformSum[4];
          double tz1 = -transformSum[5];

		//先旋转至与世界坐标系三轴方向一致的坐标系下，但是原点仍旧为第一次发现该点的相机位置
		//原点位置不变，则保证原点到该点的射线方向不变
		//根据RPY的反向顺序YPR将点反转回到与世界坐标系三轴方向一致
		//想象将该点的第一次被发现对应的坐标系，按照计算出的该时刻的相机位姿(transformSum)的反向顺序和反向(-号原因)角度转到与世界坐标下三轴方向一致
          double x1 = crz0 * u0 + srz0 * v0;
          double y1 = -srz0 * u0 + crz0 * v0;
          double z1 = 1;

          double x2 = x1;
          double y2 = crx0 * y1 + srx0 * z1;
          double z2 = -srx0 * y1 + crx0 * z1;

          double x3 = cry0 * x2 - sry0 * z2;
          double y3 = y2;
          double z3 = sry0 * x2 + cry0 * z2;
		//旋转至与当前相机坐标下三轴方向一致的坐标系下，但是原点仍旧为第一次发现该点的相机位置
		//原点位置不变，则保证原点到该点的射线方向不变
		//根据RPY的顺序将点转到与当前相机坐标系三轴方向一致
		//将已旋转与世界坐标系三轴方向一致，原点为第一次该点被发现时的相机位置的坐标系，再旋转到与当前相机的坐标系三轴方向一致，但是原点仍旧不动
		//由于上边srx1、crx1、sry1、cry1、srz1、crz1也取了负值，因此以下的旋转变换也不同上述的旋转，变换也加了负号。看sin的符号。
          double x4 = cry1 * x3 + sry1 * z3;
          double y4 = y3;
          double z4 = -sry1 * x3 + cry1 * z3;

          double x5 = x4;
          double y5 = crx1 * y4 - srx1 * z4;
          double z5 = srx1 * y4 + crx1 * z4;

          double x6 = crz1 * x5 - srz1 * y5;
          double y6 = srz1 * x5 + crz1 * y5;
          double z6 = z5;

		//再次归一化，(u0 v0)与(u1 v1)分别所在的两个坐标系的特点：只有平移关系，没有旋转关系
          u0 = x6 / z6;
          v0 = y6 / z6;

		//tx1 - tx0，ty1 - ty0，tz1 - tz0为从第一次发现该点到当前相机的平移量，这个平移量是基于世界坐标系的，可以看成世界坐标下的一个矢量，从该点第一次被发现时的相机位置指向当前相机位置。
		//因此，只用将该矢量从世界坐标系下转换到当前坐标系下
          x1 = cry1 * (tx1 - tx0) + sry1 * (tz1 - tz0);
          y1 = ty1 - ty0;
          z1 = -sry1 * (tx1 - tx0) + cry1 * (tz1 - tz0);

          x2 = x1;
          y2 = crx1 * y1 - srx1 * z1;
          z2 = srx1 * y1 + crx1 * z1;

		//三角测量三由平移得到的，有平移才会有对极几何中的三角形，因此旋转会给三角测距带来难度，单纯的旋转不能使用三角测量。
		//作者将两帧点云对应的坐标系三轴方向相对
		//tx ty tz为在当前相机坐标系下，该点第一次被发现时的相机位置指向当前相机位置矢量表示
		  double tx = crz1 * x2 - srz1 * y2;
          double ty = srz1 * x2 + crz1 * y2;
          double tz = z2;

		//利用三角测量计算出特征点深度，详细计算参见博客
          double delta = sqrt((v0 - v1) * (v0 - v1) + (u0 - u1) * (u0 - u1))
                       * cos(atan2(tz * v1 - ty, tz * u1 - tx) - atan2(v0 - v1, u0 - u1));
          double depth = sqrt((tz * u0 - tx) * (tz * u0 - tx) + (tz * v0 - ty) * (tz * v0 - ty)) / delta;

          if (depth > 0.5 && depth < 100) {
            ipr.s = depth;
            ipr.v = 2;
          }
        }

//ipDepthLast是从ipDepthCur得来的，ipDepthCur是通过上一帧计算得到的运动及匹配点对得到的。就是上上一帧的点有深度，
        //通过得到的上上帧与上帧之间的运动，从而得知与上上帧对应的上帧的点云深度。
        if (ipr.v == 2) {
          if ((*ipDepthLast)[i] > 0) {
            ipr.s = 3 * ipr.s * (*ipDepthLast)[i] / (ipr.s + 2 * (*ipDepthLast)[i]);
          }
          (*ipDepthLast)[i] = ipr.s;
        } else if ((*ipDepthLast)[i] > 0) {
          ipr.s = (*ipDepthLast)[i];
          ipr.v = 2;
        }
      }

	//ipRelations存放两帧匹配点的归一化坐标
      ipRelations->push_back(ipr);
      ipInd.push_back(imagePointsLast->points[i].ind);
    }
  }

  int iterNum = 100;
  pcl::PointXYZHSV ipr2, ipr3, ipr4;
  int ipRelationsNum = ipRelations->points.size();
  int ptNumNoDepthRec = 0;
  int ptNumWithDepthRec = 0;
  double meanValueWithDepthRec = 100000;
  for (int iterCount = 0; iterCount < iterNum; iterCount++) {
    ipRelations2->clear();
    ipy2.clear();
    int ptNumNoDepth = 0;
    int ptNumWithDepth = 0;
    double meanValueNoDepth = 0;
    double meanValueWithDepth = 0;
    for (int i = 0; i < ipRelationsNum; i++) {
      ipr = ipRelations->points[i];
      cout << "INFO: ipr.v: " << ipr.v << endl;

      double u0 = ipr.x;
      double v0 = ipr.y;
      double u1 = ipr.z;
      double v1 = ipr.h;

      cout << "INFO: last u:" << u0 << "\tv:" << v0 << endl;
      cout << "INFO: curr u:" << u1 << "\tv:" << v1 << endl;

      double srx = sin(transform[0]);
      double crx = cos(transform[0]);
      double sry = sin(transform[1]);
      double cry = cos(transform[1]);
      double srz = sin(transform[2]);
      double crz = cos(transform[2]);
      double tx = transform[3];
      double ty = transform[4];
      double tz = transform[5];
	  //深度信息未匹配
      if (fabs(ipr.v) < 0.5) {

		//ipr2中的各个分量是公式(6)相对于各个分量[r1,r2,r3,t1,t2,t3]的偏导数
        //ipr2的6个参数分别是误差函数对xyzrpy的偏导数，请务必看准顺序。。。

        //对roll的偏导数
        ipr2.x = v0*(crz*srx*(tx - tz*u1) - crx*(ty*u1 - tx*v1) + srz*srx*(ty - tz*v1)) 
               - u0*(sry*srx*(ty*u1 - tx*v1) + crz*sry*crx*(tx - tz*u1) + sry*srz*crx*(ty - tz*v1)) 
               + cry*srx*(ty*u1 - tx*v1) + cry*crz*crx*(tx - tz*u1) + cry*srz*crx*(ty - tz*v1);
        //对pitch的偏导数
        ipr2.y = u0*((tx - tz*u1)*(srz*sry - crz*srx*cry) - (ty - tz*v1)*(crz*sry + srx*srz*cry) 
               + crx*cry*(ty*u1 - tx*v1)) - (tx - tz*u1)*(srz*cry + crz*srx*sry) 
               + (ty - tz*v1)*(crz*cry - srx*srz*sry) + crx*sry*(ty*u1 - tx*v1);
        //对yaw的偏导数
        ipr2.z = -u0*((tx - tz*u1)*(cry*crz - srx*sry*srz) + (ty - tz*v1)*(cry*srz + srx*sry*crz)) 
               - (tx - tz*u1)*(sry*crz + cry*srx*srz) - (ty - tz*v1)*(sry*srz - cry*srx*crz) 
               - v0*(crx*crz*(ty - tz*v1) - crx*srz*(tx - tz*u1));
        //对tx的偏导数
        ipr2.h = cry*crz*srx - v0*(crx*crz - srx*v1) - u0*(cry*srz + crz*srx*sry + crx*sry*v1) 
               - sry*srz + crx*cry*v1;
        //对ty的偏导数
        ipr2.s = crz*sry - v0*(crx*srz + srx*u1) + u0*(cry*crz + crx*sry*u1 - srx*sry*srz) 
               - crx*cry*u1 + cry*srx*srz;
        //对tz的偏导数
        ipr2.v = u1*(sry*srz - cry*crz*srx) - v1*(crz*sry + cry*srx*srz) + u0*(u1*(cry*srz + crz*srx*sry) 
               - v1*(cry*crz - srx*sry*srz)) + v0*(crx*crz*u1 + crx*srz*v1);

	cout << "INFO: ipr2.x: " << ipr2.x << endl;
	cout << "INFO: ipr2.y: " << ipr2.y << endl;
	cout << "INFO: ipr2.z: " << ipr2.z << endl;
	cout << "INFO: ipr2.h: " << ipr2.h << endl;
	cout << "INFO: ipr2.s: " << ipr2.s << endl;
	cout << "INFO: ipr2.v: " << ipr2.v << endl;

		//y2对应论文中的公式(6),其中R阵采用欧拉角矩阵(RPY z轴->x轴->y轴)相乘, 西天北的坐标系
		//y2 = [-v1*tz+ty, u1*tz-tx, -u1*ty+v1*tx]R[u0,v0,1]^T
		//demo_lidar采用的坐标系为(左上前)，因此对应的RPY( z轴->x轴->y轴)
		//R=[crz -srz 0;srz crz 0;0 0 1]*[1 0 0;0 crx -srx;0 srx crx]*[cry 0 sry;0 1 0;-sry 0 cry];
		//R_{(i-1)toi}=R^{-1}=[cry 0 -sry;0 1 0;sry 0 cry]*[1 0 0;0 crx srx;0 -srx crx]*[crz srz 0;-srz crz 0;0 0 1]
		//R=R_{(i-1)toi}^{-1}
		//也就是说这里求出来的是R_{(i-1)toi}的逆，后边使用accumulateRotation计算要求即R_{(i-1)toi}
     //y2是误差函数，大概是两组点在转换到同一坐标系下后，在z=10平面投影的点云的x+y距离f
        double y2 = (ty - tz*v1)*(crz*sry + cry*srx*srz) - (tx - tz*u1)*(sry*srz - cry*crz*srx) 
                  - v0*(srx*(ty*u1 - tx*v1) + crx*crz*(tx - tz*u1) + crx*srz*(ty - tz*v1)) 
                  + u0*((ty - tz*v1)*(cry*crz - srx*sry*srz) - (tx - tz*u1)*(cry*srz + crz*srx*sry) 
                  + crx*sry*(ty*u1 - tx*v1)) - crx*cry*(ty*u1 - tx*v1);

	cout << "INFO: y2: " << y2 << endl;

        if (ptNumNoDepthRec < 50 || iterCount < 25 || fabs(y2) < 2 * meanValueWithDepthRec / 10000) {
			//加强约束，加速下降
          double scale = 100;
          ipr2.x *= scale;
          ipr2.y *= scale;
          ipr2.z *= scale;
          ipr2.h *= scale;
          ipr2.s *= scale;
          ipr2.v *= scale;
          y2 *= scale;
          //ipr2分量[r1,r2,r3,t1,t2,t3]的偏导数
          ipRelations2->push_back(ipr2);
          //R_{(i-1)toi}的逆
          ipy2.push_back(y2);

          ptNumNoDepth++;
        } else {
          ipRelations->points[i].v = -1;
        }
		// ipr.v==1或者ipr.v==2,即source点已有深度,使用论文中(3-4)来产生误差项
      } else if (fabs(ipr.v - 1) < 0.5 || fabs(ipr.v - 2) < 0.5) {

		//d0设为深度
        double d0 = ipr.s;

		//以下为y3的偏导数
        ipr3.x = d0*(cry*srz*crx + cry*u1*srx) - d0*u0*(sry*srz*crx + sry*u1*srx) 
               - d0*v0*(u1*crx - srz*srx);

        ipr3.y = d0*(crz*cry + crx*u1*sry - srx*srz*sry) - d0*u0*(crz*sry - crx*u1*cry + srx*srz*cry);

        ipr3.z = -d0*(sry*srz - cry*srx*crz) - d0*u0*(cry*srz + srx*sry*crz) - crx*d0*v0*crz;

        ipr3.h = 1;

        ipr3.s = 0;

        ipr3.v = -u1;

        double y3 = tx - tz*u1 + d0*(crz*sry - crx*cry*u1 + cry*srx*srz) - d0*v0*(crx*srz + srx*u1) 
                  + d0*u0*(cry*crz + crx*sry*u1 - srx*sry*srz);

		//以下为y4的偏导数
        ipr4.x = d0*(cry*v1*srx - cry*crz*crx) + d0*u0*(crz*sry*crx - sry*v1*srx) 
               - d0*v0*(crz*srx + v1*crx);

        ipr4.y = d0*(srz*cry + crz*srx*sry + crx*v1*sry) + d0*u0*(crz*srx*cry - srz*sry + crx*v1*cry);


        ipr4.z = d0*(sry*crz + cry*srx*srz) + d0*u0*(cry*crz - srx*sry*srz) - crx*d0*v0*srz;

        ipr4.h = 0;

        ipr4.s = 1;

        ipr4.v = -v1;

        double y4 = ty - tz*v1 - d0*(cry*crz*srx - sry*srz + crx*cry*v1) + d0*v0*(crx*crz - srx*v1) 
                  + d0*u0*(cry*srz + crz*srx*sry + crx*sry*v1);

        if (ptNumWithDepthRec < 50 || iterCount < 25 || 
            sqrt(y3 * y3 + y4 * y4) < 2 * meanValueWithDepthRec) {
          ipRelations2->push_back(ipr3);
          ipy2.push_back(y3);

          ipRelations2->push_back(ipr4);
          ipy2.push_back(y4);

          ptNumWithDepth++;
          meanValueWithDepth += sqrt(y3 * y3 + y4 * y4);
        } else {
          ipRelations->points[i].v = -1;
        }
      }
    }
    meanValueWithDepth /= (ptNumWithDepth + 0.01);
    ptNumNoDepthRec = ptNumNoDepth;
    ptNumWithDepthRec = ptNumWithDepth;
    meanValueWithDepthRec = meanValueWithDepth;

	//ipRelations2 已经匹配的计算出约束的特征点
    int ipRelations2Num = ipRelations2->points.size();
    cout << "vi odom ipRelations2Num:\t" << ipRelations2Num << endl;

    if (ipRelations2Num > 10) {
      cv::Mat matA(ipRelations2Num, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matAt(6, ipRelations2Num, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matB(ipRelations2Num, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

      for (int i = 0; i < ipRelations2Num; i++) {
        ipr2 = ipRelations2->points[i];

		//A为jacobian矩阵
		//ipr2中的各个分量是公式(6)相对于各个分量[r1,r2,r3,t1,t2,t3]的偏导数
        matA.at<float>(i, 0) = ipr2.x;
        matA.at<float>(i, 1) = ipr2.y;
        matA.at<float>(i, 2) = ipr2.z;
        matA.at<float>(i, 3) = ipr2.h;
        matA.at<float>(i, 4) = ipr2.s;
        matA.at<float>(i, 5) = ipr2.v;
		//-0.2为优化步长,由于是下降，因此取负号
        matB.at<float>(i, 0) = -0.2 * ipy2[i];
      }
	  //高斯牛顿法，dX=-(J^T*J)^{-1}*(J^T*r)
	  //对应J^T*J对应程序中的AtA，J^T*r则对应AtB
	  //B阵已取了负号，因此得，AtA*dX=AtB
      cv::transpose(matA, matAt);
      matAtA = matAt * matA;
      matAtB = matAt * matB;

	  cout << "matAtA:\n" << matAtA << endl;
	  cout << "matAtB:\n" << matAtB << endl;
      //cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
	  //求解dX
      cv::solve(matAtA, matAtB, matX, cv::DECOMP_CHOLESKY);

      //if (fabs(matX.at<float>(0, 0)) < 0.1 && fabs(matX.at<float>(1, 0)) < 0.1 && 
      //    fabs(matX.at<float>(2, 0)) < 0.1) {
        transform[0] += matX.at<float>(0, 0);
        transform[1] += matX.at<float>(1, 0);
        transform[2] += matX.at<float>(2, 0);
        transform[3] += matX.at<float>(3, 0);
        transform[4] += matX.at<float>(4, 0);
        transform[5] += matX.at<float>(5, 0);
      //}

	cout << "******************INFO*****************" << endl;
	cout << "transform[0]:" << transform[0] << "\t" << endl;
	cout << "transform[1]:" << transform[1] << "\t" << endl;
	cout << "transform[2]:" << transform[2] << "\t" << endl;
	cout << "transform[3]:" << transform[3] << "\t" << endl;
	cout << "transform[4]:" << transform[4] << "\t" << endl;
	cout << "transform[5]:" << transform[5] << "\t" << endl;
	cout << "******************INFO*****************" << endl;

      float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
                   + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
                   + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
      float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
                   + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
                   + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);
      //这里的RT是上一帧到本帧的变换
      if (deltaR < 0.00001 && deltaT < 0.00001) {
        break;
      }

      ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
    }
  }

  if (!imuInited) {
	//imuxxxCur存放的imu在世界坐标系下的旋转，由于yaw的变化范围会超过(-pi,pi)，可能出现-pi~pi之间的跳变，因此yaw的初始值需单独存放
    imuYawInit = imuYawCur;
//作者定义的坐标系为左上前，imu推测坐标系为(右后上,未猜出三轴顺序)
    transform[0] = imuPitchCur;
    transform[2] = imuRollCur;

    imuInited = true;
  }

	//rx,ry,rz为相机累积旋转欧拉角
  double rx, ry, rz;
  accumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                    -transform[0], -transform[1], -transform[2], rx, ry, rz);

  if (imuPointerLast >= 0) {
    double drx, dry, drz;
	//比较imu和解算得出的相机旋转累积的差值
    diffRotation(imuPitchCur, imuYawCur - imuYawInit, imuRollCur, rx, ry, rz, drx, dry, drz);

	//旋转上减去小部分差值。0.1相当与权重，对imu旋转累积的置信程度
	//俯仰
    transform[0] -= 0.1 * drx;
    /*if (dry > PI) {
      transform[1] -= 0.1 * (dry - 2 * PI);
    } else if (imuYawCur - imuYawInit - ry < -PI) {
      transform[1] -= 0.1 * (dry + 2 * PI);
    } else {
      transform[1] -= 0.1 * dry;
    }*/
	//滚转
    transform[2] -= 0.1 * drz;

    accumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                      -transform[0], -transform[1], -transform[2], rx, ry, rz);
  }
    //首先将计算出来的两帧(Cur与Last帧)之间的偏移量通过rx,ry,rzz旋转到世界坐标系下，然后再累加得到当前时刻相机在世界坐标系下的累计偏移，即pose.pose
	//偏移量乘以滚转矩阵
  double x1 = cos(rz) * transform[3] - sin(rz) * transform[4];
  double y1 = sin(rz) * transform[3] + cos(rz) * transform[4];
  double z1 = transform[5];

	//偏移量乘以俯仰矩阵
  double x2 = x1;
  double y2 = cos(rx) * y1 - sin(rx) * z1;
  double z2 = sin(rx) * y1 + cos(rx) * z1;

	//计算出旋转后对应的偏移；偏移量再乘以偏航矩阵得到世界坐标系下的帧间偏移变化
	//同样，transform[3-5]是将上帧点变换到下帧点的变换矩阵对应的偏移，要计算相机的累积偏移，则需取负；
	//类似旋转，由上下帧点云旋转变换{transform[0-2]:{rpy}:R}得相机累积偏移：{-y,-p,-r}即R^{-1}
  //tx,ty,tz为当前时刻相机在世界坐标系下的累积偏移，transformSum存放偏移累积j值
  double tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
  double ty = transformSum[4] - y2;
  double tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

	//transformSum 相机累积旋转偏移
  transformSum[0] = rx;
  transformSum[1] = ry;
  transformSum[2] = rz;
  transformSum[3] = tx;
  transformSum[4] = ty;
  transformSum[5] = tz;

  pcl::PointXYZHSV spc;
  spc.x = transformSum[0];
  spc.y = transformSum[1];
  spc.z = transformSum[2];
  spc.h = transformSum[3];
  spc.s = transformSum[4];
  spc.v = transformSum[5];

  double crx = cos(transform[0]);
  double srx = sin(transform[0]);
  double cry = cos(transform[1]);
  double sry = sin(transform[1]);

  j = 0;
//遍寻上下帧匹配点
  for (int i = 0; i < imagePointsCurNum; i++) {
    bool ipFound = false;
    for (; j < imagePointsLastNum; j++) {
      if (imagePointsLast->points[j].ind == imagePointsCur->points[i].ind) {
        ipFound = true;
      }
      if (imagePointsLast->points[j].ind >= imagePointsCur->points[i].ind) {
        break;
      }
    }

    //如果有与cur点i 相匹配的last点j,
    if (ipFound) {
	//startTransCur只保存特征点第一次被发现的时刻相机的运动情况
	//startPointsCur在第一次被发现时刻的相机坐标系下的图像坐标
	//如果点被匹配上，startPointsCur对应的点继续沿用startPointsLast，同样startTransCur也沿用startTransLast
      startPointsCur->push_back(startPointsLast->points[j]);
      startTransCur->push_back(startTransLast->points[j]);

	//如果该点上一时刻有对应深度值，则利用计算出的transform计算出该特征点在当前时刻的深度值
      if ((*ipDepthLast)[j] > 0) {
		//找到图像点对应深度值，并恢复其在上帧相机坐标系下的三维坐标
        double ipz = (*ipDepthLast)[j];
        double ipx = imagePointsLast->points[j].u * ipz;
        double ipy = imagePointsLast->points[j].v * ipz;

		//将上帧相机坐标系下的图像点坐标变换到当前相机坐标系下
		//偏航恢复
        x1 = cry * ipx + sry * ipz;
        y1 = ipy;
        z1 = -sry * ipx + cry * ipz;
		//俯仰恢复
        x2 = x1;
        y2 = crx * y1 - srx * z1;
        z2 = srx * y1 + crx * z1;
		//滚转是绕前轴旋转，不变换不影响z，即z3=x3;
		//z3+transform[5]得到图像点的深度,并保存
		//得到该点当前时刻的深度值
        ipDepthCur->push_back(z2 + transform[5]);
      } else {
        ipDepthCur->push_back(-1);
      }
    //没有找到匹配点
    } else {
	//如果没有找到匹配点，则认为该点第一次被发现，将当前时刻的相机运动情况及该点的归一化坐标保存在startTransCur和startPointsCur中
      startPointsCur->push_back(imagePointsCur->points[i]);
	//spc存放当前相机的累积旋转偏移
      startTransCur->push_back(spc);
      ipDepthCur->push_back(-1);
    }
  }
  startPointsLast->clear();
  startTransLast->clear();
  ipDepthLast->clear();

  angleSum[0] -= transform[0];
  angleSum[1] -= transform[1];
  angleSum[2] -= transform[2];

  //相机坐标系左上前(西天北)
  //作者认为rollPitchYaw对应的坐标系是前右下(北东地)，因此作者在对应将rx，ry, rz调整到北东地坐标系下，再使用tf::createQuaternionMsgFromRollPitchYaw得到对应北东地坐标系下的四元数。因此下边给voData赋值时，需要再调整到西天北坐标系下。
//rz:roll; -rx:pitch; -ry:yaw
//tf::createQuaternionMsgFromRollPitchYaw 的输入为roll,pitch,yaw，定义的坐标系为NED，因此tf坐标系对应相机坐标系：z,-x,-y
//个人目前觉得此处虽程序运行未出问题，但是ros的坐标系是右手坐标系，北西天。而前边解析imu的数据得到roll,pitch,yaw，使用的是该坐标系
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);

  nav_msgs::Odometry voData;
  voData.header.frame_id = "/camera_init";
  voData.child_frame_id = "/camera";
  voData.header.stamp = imagePoints2->header.stamp;
//再将四元数对应到左前上的相机世界坐标系下
//相机坐标系:WUN(右手坐标系),
//相机坐标系对应tf坐标系：-y,-z,x
  voData.pose.pose.orientation.x = -geoQuat.y;
  voData.pose.pose.orientation.y = -geoQuat.z;
  voData.pose.pose.orientation.z = geoQuat.x;
  voData.pose.pose.orientation.w = geoQuat.w;
  voData.pose.pose.position.x = tx;
  voData.pose.pose.position.y = ty;
  voData.pose.pose.position.z = tz;
// 这里的twist存放的是世界坐标系下相机的旋转累积
  voData.twist.twist.angular.x = angleSum[0];
  voData.twist.twist.angular.y = angleSum[1];
  voData.twist.twist.angular.z = angleSum[2];
  voDataPubPointer->publish(voData);

  cout << "INFO **********************************" << endl;
  cout << "INFO orientation x y z w :" << endl;
  cout << "INFO " << -geoQuat.y << " " << -geoQuat.z << " " << geoQuat.x << " " << geoQuat.w << endl;
  cout << "INFO pose x y z :" << endl;
  cout << "INFO " << tx << " " << ty << " " << tz << endl;
  cout << "INFO **********************************" << endl;

  tf::StampedTransform voTrans;
  voTrans.frame_id_ = "/camera_init";
  voTrans.child_frame_id_ = "/camera";
  voTrans.stamp_ = imagePoints2->header.stamp;
  voTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  voTrans.setOrigin(tf::Vector3(tx, ty, tz));
  tfBroadcasterPointer->sendTransform(voTrans);

  pcl::PointCloud<DepthPoint>::Ptr depthPointsTemp = depthPointsLast;
  depthPointsLast = depthPointsCur;
  depthPointsCur = depthPointsTemp;

  DepthPoint ipd;
  depthPointsCur->clear();

//depthPointsCur也存放了相机z世界坐标系下的运动
//ipd.ind=-2对应的点的坐标为3轴旋转
  ipd.u = transformSum[0];
  ipd.v = transformSum[1];
  ipd.depth = transformSum[2];
  ipd.ind = -2;
  depthPointsCur->push_back(ipd);

//ipd.ind=-1对应的点的坐标为3轴偏移
  ipd.u = transformSum[3];
  ipd.v = transformSum[4];
  ipd.depth = transformSum[5];
  ipd.ind = -1;
  depthPointsCur->push_back(ipd);

  depthPointsLastNum = depthPointsCurNum;
//depthPointsCur前两个点存放了旋转和偏移，因此depthPointsCurNum初始为2
  depthPointsCurNum = 2;

  j = 0;
  pcl::PointXYZ ipp;
  depthPointsSend->clear();
  imagePointsProj->clear();
  for (int i = 0; i < ipRelationsNum; i++) {
	//无论点云是通过匹配深度还是三角测距获得的深度值
    if (fabs(ipRelations->points[i].v - 1) < 0.5 || fabs(ipRelations->points[i].v - 2) < 0.5) {

	//z h 对应cur
      ipd.u = ipRelations->points[i].z;
      ipd.v = ipRelations->points[i].h;
	//计算的s是last帧的深度值，恢复当前帧深度。
      ipd.depth = ipRelations->points[i].s + transform[5];
      ipd.label = ipRelations->points[i].v;
      ipd.ind = ipInd[i];

		//将点云都存放在depthPointsCur并计数
      depthPointsCur->push_back(ipd);
      depthPointsCurNum++;

      for (; j < depthPointsLastNum; j++) {
        if (depthPointsLast->points[j].ind < ipInd[i]) {
          depthPointsSend->push_back(depthPointsLast->points[j]);
        } else if (depthPointsLast->points[j].ind > ipInd[i]) {
          break;
        }
      }
	//x,y对应last
      ipd.u = ipRelations->points[i].x;
      ipd.v = ipRelations->points[i].y;
      ipd.depth = ipRelations->points[i].s;

      depthPointsSend->push_back(ipd);

	//恢复三维坐标
      ipp.x = ipRelations->points[i].x * ipRelations->points[i].s;
      ipp.y = ipRelations->points[i].y * ipRelations->points[i].s;
      ipp.z = ipRelations->points[i].s;

      imagePointsProj->push_back(ipp);
    }
  }

  sensor_msgs::PointCloud2 depthPoints2;
  pcl::toROSMsg(*depthPointsSend, depthPoints2);
  depthPoints2.header.frame_id = "camera2";
  depthPoints2.header.stamp = ros::Time().fromSec(imagePointsLastTime);
  depthPointsPubPointer->publish(depthPoints2);
  //发布恢复3维位置的点云
  sensor_msgs::PointCloud2 imagePointsProj2;
  pcl::toROSMsg(*imagePointsProj, imagePointsProj2);
  imagePointsProj2.header.frame_id = "camera2";
  imagePointsProj2.header.stamp = ros::Time().fromSec(imagePointsLastTime);
  imagePointsProjPubPointer->publish(imagePointsProj2);
}

//处理激光接收到的深度点云信息，存放到kdtree中
void depthCloudHandler(const sensor_msgs::PointCloud2ConstPtr& depthCloud2)
{
  depthCloudTime = depthCloud2->header.stamp.toSec();

  depthCloud->clear();
  pcl::fromROSMsg(*depthCloud2, *depthCloud);
  depthCloudNum = depthCloud->points.size();

  if (depthCloudNum > 10) {
    for (int i = 0; i < depthCloudNum; i++) {
    // 深度图的接收在于将传来的点云归一化到距离光心10米的平面上，之所以是z值为10是loam一向的做法，以相机坐标系为基准。
		//intensity 存放点云的原始深度信息,xyz存放深度为化为10的坐标
		//同样将点云归一化后放大10倍,与视觉特征点的存放相一致，即将激光采集到的点统一放在深度为10的归一化平面上
      depthCloud->points[i].intensity = depthCloud->points[i].z;
      depthCloud->points[i].x *= 10 / depthCloud->points[i].z;
      depthCloud->points[i].y *= 10 / depthCloud->points[i].z;
      depthCloud->points[i].z = 10;
    }

	//kdTree 中存放深度点
    kdTree->setInputCloud(depthCloud);
  }
}
//imu回调函数，将imu欧拉角数据存放在imuRoll、imuPitch、imuYaw队列中
void imuDataHandler(const sensor_msgs::Imu::ConstPtr& imuData)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuData->orientation, orientation);
//北西天下的roll, pitch, yaw，再赋值给transform[0]-[2]时没有符号变化，只有顺序变化
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  //imu回调函数，将imu欧拉角数据只存放imuQueLength长度队列
  imuTime[imuPointerLast] = imuData->header.stamp.toSec() - 0.1068;
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
}
//imageDataHandler将相关联的特征点用红绿蓝颜色圆圈发布出来
void imageDataHandler(const sensor_msgs::Image::ConstPtr& imageData) 
{
  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(imageData, "bgr8");

 for (const auto &k: kImage)
   ROS_ERROR_STREAM(k);
  int ipRelationsNum = ipRelations->points.size();
  for (int i = 0; i < ipRelationsNum; i++) {
    if (fabs(ipRelations->points[i].v) < 0.5) {
	  //没有深度点，显示红色
    // x,y对应上一帧图像点归一化坐标；z,h对应匹配上一帧xy的当前帧的图像点归一化坐标
      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(255, 0, 0), 2);
    } else if (fabs(ipRelations->points[i].v - 1) < 0.5) {
	  //通过点云获得深度的图像点，显示绿色
      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 255, 0), 2);
    } else if (fabs(ipRelations->points[i].v - 2) < 0.5) {
	  //通过相机运动获得深度的图像点，显示蓝色
      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 0, 255), 2);
    } /*else {
      cv::circle(bridge->image, cv::Point((kImage[2] - ipRelations->points[i].z * kImage[0]) / showDSRate,
                (kImage[5] - ipRelations->points[i].h * kImage[4]) / showDSRate), 1, CV_RGB(0, 0, 0), 2);
    }*/
  }

  sensor_msgs::Image::Ptr imagePointer = bridge->toImageMsg();
  imageShowPubPointer->publish(imagePointer);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualOdometry");
  ros::NodeHandle nh;

  ros::Subscriber imagePointsSub = nh.subscribe<sensor_msgs::PointCloud2>
                                   ("/image_points_last", 5, imagePointsHandler);

  ros::Subscriber depthCloudSub = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/depth_cloud", 5, depthCloudHandler);

  ros::Subscriber imuDataSub = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 5, imuDataHandler);

  ros::Publisher voDataPub = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
  voDataPubPointer = &voDataPub;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher depthPointsPub = nh.advertise<sensor_msgs::PointCloud2> ("/depth_points_last", 5);
  depthPointsPubPointer = &depthPointsPub;

  ros::Publisher imagePointsProjPub = nh.advertise<sensor_msgs::PointCloud2> ("/image_points_proj", 1);
  imagePointsProjPubPointer = &imagePointsProjPub;

  ros::Subscriber imageDataSub = nh.subscribe<sensor_msgs::Image>("/image/show", 1, imageDataHandler);

  ros::Publisher imageShowPub = nh.advertise<sensor_msgs::Image>("/image/show_2", 1);
  imageShowPubPointer = &imageShowPub;

  ros::spin();

  return 0;
}
