// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <cmath>
#include <vector>

#include <css_loam_velodyne/common.h>
#include <css_loam_velodyne/css.h>
#include <css_loam_velodyne/gaussians.h>

#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/sample_consensus/sac_model_circle.h>

#include <iostream>
#include <fstream>
using namespace std;

using std::sin;
using std::cos;
using std::atan2;

const double scanPeriod = 0.1;
//const double scanPeriod = 0.2;

const int systemDelay = 20;
//const int systemDelay = 10;
int systemInitCount = 0;
bool systemInited = false;

//const int N_SCANS = 16;
//const float ang_res_y = 2.0;
//const float ang_bottom = 15+0.1;

const int N_SCANS = 64;
const float ang_res_y = 0.427;
const float ang_bottom = 24.9;

std::vector<float> gaussian1[NUM_GAUSSIANS];
std::vector<float> gaussian2[NUM_GAUSSIANS];

float lowerBound = -24.9f;
float upperBound = 2.0f;
float factor = (64.0 - 1.0) / (upperBound - lowerBound);

/*float cloudCurvature[40000];
int cloudSortInd[40000];
int cloudNeighborPicked[40000];
int cloudLabel[40000];*/

float cloudCurvature[200000];
int cloudSortInd[200000];
int cloudNeighborPicked[200000];
int cloudLabel[200000];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubImuTrans;

/*typedef struct lcframe
{
  std::vector<CSSKeypoint> features;
}LCFrame;

std::vector<LCFrame> lcTrajectory;*/

void ShiftToStartIMU(float pointTime)
{
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

void VeloToStartIMU()
{
  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}

void TransformToStartIMU(PointType *p)
{
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < scanPeriod) {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}


int getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - lowerBound) * factor + 0.5);
}

void extractFeatures(pcl::PointCloud<PointType>::Ptr laserCloud,
  int cloudSize,
  pcl::PointCloud<PointType>& cornerPointsSharp,
  pcl::PointCloud<PointType>& cornerPointsLessSharp,
  pcl::PointCloud<PointType>& surfPointsFlat,
  pcl::PointCloud<PointType>& surfPointsLessFlat)
{

	std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
	int scanCount = -1;
  	for (int i = 5; i < cloudSize - 5; i++) {
	    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
	                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
	                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
	                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
	                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
	                + laserCloud->points[i + 5].x;
	    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
	                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
	                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
	                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
	                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
	                + laserCloud->points[i + 5].y;
	    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
	                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
	                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
	                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
	                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
	                + laserCloud->points[i + 5].z;
	    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
	    cloudSortInd[i] = i;
	    cloudNeighborPicked[i] = 0;
	    cloudLabel[i] = 0;

	    if (int(laserCloud->points[i].intensity) != scanCount) {
	      scanCount = int(laserCloud->points[i].intensity);

	      if (scanCount > 0 && scanCount < N_SCANS) {
	        scanStartInd[scanCount] = i + 5;
	        scanEndInd[scanCount - 1] = i - 5;
	      }
	    }
  	}

  //std::cout<<"HolaMundo 779\n";//Aqui no llega con mi point cloud -> ERROR
  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;

  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i - 0] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  for (int i = 0; i < N_SCANS; i++) {
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) { //6 regiones (como 6 clusters) por ring
      int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6; //indice de donde empieza la region
      int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1; //indice de donde termina la region

      //Bubble sort
      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) { //2 por region
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(laserCloud->points[ind]);
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) { //+1
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {//-1
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;
          surfPointsFlat.push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;
  }

  //End Feature Extraction

}

/*void extractFeaturesCSS(std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans,
  int cloudSize,
  pcl::PointCloud<PointType>& cornerPointsSharp,
  pcl::PointCloud<PointType>& cornerPointsLessSharp,
  pcl::PointCloud<PointType>& surfPointsFlat,
  pcl::PointCloud<PointType>& surfPointsLessFlat)*/
void extractFeaturesCSS(std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans,
  int cloudSize,
  pcl::PointCloud<PointType>& cornerPointsSharp,
  pcl::PointCloud<PointType>& cornerPointsLessSharp,
  pcl::PointCloud<PointType>& surfPointsFlat,
  pcl::PointCloud<PointType>& surfPointsLessFlat)
{

  int ringIdx = 0;

  //std::vector<CSSKeypoint> currentSharpKeypoints;
  std::vector<CSSKeypoint> currentSharpKeypointsAll;
  std::vector<CSSKeypoint> previousSharpKeypoints;

  pcl::PointCloud<PointType> cornerPointsSharpAux;
  pcl::PointCloud<PointType> cornerPointsLessSharpAux;


  //for(std::vector< pcl::PointCloud<pcl::PointXYZI> >::iterator it = laserCloudScans.begin()+8; it != laserCloudScans.end(); ++it)
  //for(std::vector< pcl::PointCloud<pcl::PointXYZI> >::iterator it = laserCloudScans.begin()+16; it != laserCloudScans.end(); ++it)
  //for(std::vector< pcl::PointCloud<pcl::PointXYZI> >::iterator it = laserCloudScans.begin()+38; it != laserCloudScans.end(); ++it)
  //for(std::vector< pcl::PointCloud<pcl::PointXYZI> >::iterator it = laserCloudScans.begin()+16; it != laserCloudScans.end(); ++it)
  for(std::vector< pcl::PointCloud<pcl::PointXYZI> >::iterator it = laserCloudScans.begin(); it != laserCloudScans.end(); ++it)
  {
    std::vector< pcl::PointCloud<pcl::PointXYZI> > clusters;
    std::vector< std::vector<int> > clusterIndices;

    std::vector<int> pointPicked(it->size(),0);
    std::vector<int> cloudLabel2(it->size(),0);


    if(it->size()>39)
    //if(it->size()>99)
      clusterize(*it, clusters, clusterIndices, pointPicked, 40, 400);
      //clusterize(*it, clusters, clusterIndices, pointPicked, 100, 400);
    //if(it->size()>69)
      //clusterize(*it, clusters, 70, 400);
    else
      continue;

  	
    //std::cout<<"cluster size: "<<clusters.size()<<", idx size: "<<clusterIndices.size()<<std::endl;

    int clusterIdx = 0;
    //int previousClusterSize = 0;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
    
    //Begin clusters
    for (std::vector< pcl::PointCloud<pcl::PointXYZI> >::iterator it2 = clusters.begin() ; it2 != clusters.end(); ++it2)
    {
      //std::vector<int> pointPicked(it2->size(),0);
      //std::vector<int> cloudLabel2(it2->size(),0);

      //Begin get cornerPointsSharp and cornerPointsLessSharp features (CSS keypoints)
      std::vector<CurvatureTriplet> keypoints[NUM_SCALES];
      std::vector<float> s;
      std::vector<CSSKeypoint> sharpKeypointsPerCluster;
      computeScaleSpace(*it2, gaussian1, gaussian2, keypoints, s);

      getFinalKeypointsAtMinScale(*it2, clusterIndices[clusterIdx], keypoints, s, sharpKeypointsPerCluster); //Modificar el index de CSSKeypoint por el index del ring
      
      //Begin get CSS cornerPointsSharp and cornerPointsLessSharp features (CSS keypoints)
      if(sharpKeypointsPerCluster.size()!=0)
      {

        sort(sharpKeypointsPerCluster.begin(), sharpKeypointsPerCluster.end(), compareCSSKeypoint);

        if(sharpKeypointsPerCluster.size()<=2)
        //if(sharpKeypointsPerCluster.size()<=1)
        {
          for(int i=0; i<sharpKeypointsPerCluster.size(); ++i)
          {
              if(pointPicked[sharpKeypointsPerCluster[i].index] == 0)
              {
                cornerPointsSharp.push_back(sharpKeypointsPerCluster[i].point);
                cornerPointsLessSharp.push_back(sharpKeypointsPerCluster[i].point);
                pointPicked[sharpKeypointsPerCluster[i].index] = 1;
                cloudLabel2[sharpKeypointsPerCluster[i].index] = 2;
              }
          }
        }
        else
        {
          for(int i=0; i<2; ++i)
          //for(int i=0; i<1; ++i)
          {
              if(pointPicked[sharpKeypointsPerCluster[i].index] == 0)
              {
                cornerPointsSharp.push_back(sharpKeypointsPerCluster[i].point);
                cornerPointsLessSharp.push_back(sharpKeypointsPerCluster[i].point);
                pointPicked[sharpKeypointsPerCluster[i].index] = 1;
                cloudLabel2[sharpKeypointsPerCluster[i].index] = 2;
              }
          }

          for(int i=2; i<sharpKeypointsPerCluster.size(); ++i)
          //for(int i=1; i<sharpKeypointsPerCluster.size(); ++i)
          {
            if(pointPicked[sharpKeypointsPerCluster[i].index] == 0)
            {
              cornerPointsLessSharp.push_back(sharpKeypointsPerCluster[i].point);
              pointPicked[sharpKeypointsPerCluster[i].index] = 1;
              cloudLabel2[sharpKeypointsPerCluster[i].index] = 1;
            }
          }

          //Add start point of cluster
          if(pointPicked[clusterIndices[clusterIdx][0]] == 0)
          {
            cornerPointsLessSharp.push_back((*it2)[0]);
            pointPicked[clusterIndices[clusterIdx][0]] = 1;
            cloudLabel2[clusterIndices[clusterIdx][0]] = 1;
          }
          
          //Add end point of cluster
          if(pointPicked[clusterIndices[clusterIdx][it2->size()-1]] == 0)
          {
            cornerPointsLessSharp.push_back((*it2)[it2->size()-1]);
            pointPicked[clusterIndices[clusterIdx][it2->size()-1]] = 1;
            cloudLabel2[clusterIndices[clusterIdx][it2->size()-1]] = 1;
          }
        }
      }
      //End get CSS cornerPointsSharp and cornerPointsLessSharp features (CSS keypoints)


      clusterIdx++;
    }
    //Begin clusters

    //Begin extract flat and less flat keypoints
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < clusterIndices.size(); ++j) { //6 regiones (como 6 clusters) por ring
      int sp = clusterIndices[j][0] + 5;//(scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6; //indice de donde empieza la region
      int ep = clusterIndices[j][clusterIndices[j].size()-1] - 5;//(scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1; //indice de donde termina la region

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) 
      {
        if (pointPicked[k] == 0) {

          cloudLabel2[k] = -1;
          surfPointsFlat.push_back(it->points[k]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          pointPicked[k] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = it->points[k + l].x
                        - it->points[k + l - 1].x;
            float diffY = it->points[k + l].y
                        - it->points[k + l - 1].y;
            float diffZ = it->points[k + l].z
                        - it->points[k + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            pointPicked[k + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = it->points[k + l].x
                        - it->points[k + l + 1].x;
            float diffY = it->points[k + l].y
                        - it->points[k + l + 1].y;
            float diffZ = it->points[k + l].z
                        - it->points[k + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            pointPicked[k + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel2[k] <= 0) {
          surfPointsLessFlatScan->push_back(it->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;
    //End extract flat and less flat keypoints

    ringIdx++;
  }
}

long frame = 0;
long sharpFeaturesNum = 0;
long lessSharpFeaturesNum = 0;

//ofstream featuresNumber("/home/claydergc/featuresNumber02.txt");
//ofstream featuresNumber;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    }
    return;
  }

  //std::vector<int> scanStartInd(N_SCANS, 0);
  //std::vector<int> scanEndInd(N_SCANS, 0);

  double timeScanCur = laserCloudMsg->header.stamp.toSec();
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  int cloudSize = laserCloudIn.points.size();

  //std::cout<<"Tamanio"<<cloudSize<<std::endl;

  float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x); //orientation of the first point
  float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                        laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI; //orientation of the last point

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }
  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].y;
    point.y = laserCloudIn.points[i].z;
    point.z = laserCloudIn.points[i].x;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    /*float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
    int scanID;
    int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5));
    if (roundedAngle > 0){
      scanID = roundedAngle;
    }
    else {
      scanID = roundedAngle + (N_SCANS - 1);
    }*/

    // calculate vertical point angle and scan ID
    //float angle = atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    //int scanID = getRingForAngle(angle);

    float verticalAngle = atan2(point.y , sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
    int scanID;
    int roundedAngle = (verticalAngle + ang_bottom) / ang_res_y;
    scanID = roundedAngle;

    if (scanID > (N_SCANS - 1) || scanID < 0 ){
      count--;
      continue;
    }



    /*if (scanID > (N_SCANS - 1) || scanID < 0 ){
      count--;
      continue;
    }*/

    float ori = -atan2(point.x, point.z); //orientation
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + scanPeriod * relTime;

    if (imuPointerLast >= 0) {
      float pointTime = relTime * scanPeriod;
      while (imuPointerFront != imuPointerLast) {
        if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
          break;
        }
        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }

      if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
        imuRollCur = imuRoll[imuPointerFront];
        imuPitchCur = imuPitch[imuPointerFront];
        imuYawCur = imuYaw[imuPointerFront];

        imuVeloXCur = imuVeloX[imuPointerFront];
        imuVeloYCur = imuVeloY[imuPointerFront];
        imuVeloZCur = imuVeloZ[imuPointerFront];

        imuShiftXCur = imuShiftX[imuPointerFront];
        imuShiftYCur = imuShiftY[imuPointerFront];
        imuShiftZCur = imuShiftZ[imuPointerFront];
      } else {
        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack])
                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime)
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

        imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
        imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
        if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
        } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
        } else {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
        }

        imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
        imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
        imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

        imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
        imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
        imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
      }
      if (i == 0) {
        imuRollStart = imuRollCur;
        imuPitchStart = imuPitchCur;
        imuYawStart = imuYawCur;

        imuVeloXStart = imuVeloXCur;
        imuVeloYStart = imuVeloYCur;
        imuVeloZStart = imuVeloZCur;

        imuShiftXStart = imuShiftXCur;
        imuShiftYStart = imuShiftYCur;
        imuShiftZStart = imuShiftZCur;
      } else {
        ShiftToStartIMU(pointTime);
        VeloToStartIMU();
        TransformToStartIMU(&point);
      }
    }
    laserCloudScans[scanID].push_back(point);


  }

  //std::cout<<"HolaMundo 777\n";

  cloudSize = count;

  //std::cout<<"cloud size:"<<cloudSize<<std::endl;

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < N_SCANS; i++) {
    *laserCloud += laserCloudScans[i];
  }

  //std::cout<<"laser cloud size:"<<laserCloud->size()<<std::endl;

  //std::cout<<"laser cloud pt1:"<<laserCloud->points[0].x<<std::endl;
  //std::cout<<"laser cloud pt2:"<<laserCloud->points[cloudSize-1].x<<std::endl;

  //std::cout<<"HolaMundo 778\n";

  

  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  //extractFeatures(laserCloud, cloudSize, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat);
  extractFeaturesCSS(laserCloudScans, cloudSize, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat);
  
  //sharpFeaturesNum += cornerPointsSharp.size();
  //lessSharpFeaturesNum += cornerPointsLessSharp.size();
  //frame++;

  //featuresNumber<<cornerPointsSharp.size()<<" "<<cornerPointsLessSharp.size()<<"\n";

  //extractFeaturesCSS(laserCloudScans, cloudSize, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat);
  //std::cout<<"Sharp points: "<<cornerPointsSharp.size()<<std::endl;
  //std::cout<<"Less sharp points: "<<cornerPointsLessSharp.size()<<std::endl;



  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/camera";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "/camera";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsLessSharpMsg.header.frame_id = "/camera";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "/camera";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/camera";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

  pcl::PointCloud<pcl::PointXYZ> imuTrans(4, 1);
  imuTrans.points[0].x = imuPitchStart;
  imuTrans.points[0].y = imuYawStart;
  imuTrans.points[0].z = imuRollStart;

  imuTrans.points[1].x = imuPitchCur;
  imuTrans.points[1].y = imuYawCur;
  imuTrans.points[1].z = imuRollCur;

  imuTrans.points[2].x = imuShiftFromStartXCur;
  imuTrans.points[2].y = imuShiftFromStartYCur;
  imuTrans.points[2].z = imuShiftFromStartZCur;

  imuTrans.points[3].x = imuVeloFromStartXCur;
  imuTrans.points[3].y = imuVeloFromStartYCur;
  imuTrans.points[3].z = imuVeloFromStartZCur;

  sensor_msgs::PointCloud2 imuTransMsg;
  pcl::toROSMsg(imuTrans, imuTransMsg);
  imuTransMsg.header.stamp = laserCloudMsg->header.stamp;
  imuTransMsg.header.frame_id = "/camera";
  pubImuTrans.publish(imuTransMsg);


}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  AccumulateIMUShift();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;


  //string fileName(argv[1]);
  //featuresNumber = ofstream("/home/claydergc/"+fileName);

  //std::cout<<"HolaMundo\n";
  initKernels(gaussian1, gaussian2);

  /*ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/velodyne_points", 2, laserCloudHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("/velodyne_cloud_2", 2);

  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>
                                        ("/laser_cloud_sharp", 2);

  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_less_sharp", 2);

  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>
                                       ("/laser_cloud_flat", 2);

  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_less_flat", 2);

  pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);*/

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/kitti/velo/pointcloud", 1, laserCloudHandler);

  //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
  //                                ("/velodyne_points", 1, laserCloudHandler);                                  

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("/velodyne_cloud_2", 1);

  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>
                                        ("/laser_cloud_sharp", 1);

  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_less_sharp", 1);

  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>
                                       ("/laser_cloud_flat", 1);

  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_less_flat", 1);

  pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);

  //std::cout<<"ChauMundo\n";

  ros::spin();

  //featuresNumber.close();

  //std::cout<<"Sharp Features Number: "<<sharpFeaturesNum/frame<<std::endl;
  //std::cout<<"Less Sharp Features Number: "<<lessSharpFeaturesNum/frame<<std::endl;

  return 0;
}
