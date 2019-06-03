// util.h
#ifndef CSS_H
#define CSS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>

const float FLOAT_MIN = std::numeric_limits<float>::min();
const float FLOAT_MAX = std::numeric_limits<float>::max();
const int NUM_SCALES = 16;
const int NUM_GAUSSIANS = 160;

typedef struct curvatureTriplet
{
	int index;
	float sIndex;
	float curvature;
}CurvatureTriplet;

bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j);

typedef struct curvatureCounter
{
	float index;
	int counter;
	float curvature;
}CurvatureCounter;

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j);

typedef struct cssKeypoint
{
	int index;
	float curvature;
	pcl::PointXYZI point;
}CSSKeypoint;

bool compareCSSKeypoint (CSSKeypoint i, CSSKeypoint j);


void printKeypoints(std::vector<CurvatureTriplet> keypoints);
int findByThresholdAtMinScale(CurvatureTriplet a, std::vector<CurvatureTriplet> vec, float threshold);
int findByThreshold(CurvatureTriplet a, std::vector<CurvatureTriplet> vec, float threshold);
int findCurvatureTriplet (std::vector<CurvatureTriplet> vec, CurvatureTriplet c);

void removeConstantCurvature(std::vector<CurvatureTriplet>& keypoints);

void parametrizeCurve(pcl::PointCloud<pcl::PointXYZI> in, std::vector<float> &s);
void getCurvatureExtrema(std::vector<float> curvature, std::vector<float> s, std::vector<CurvatureTriplet>& keypoints, float min, float max, bool isMaxScale);
void getCurvatureExtremaAtMaxScale(std::vector<float> curvature, std::vector<float> s, float min, float max, std::vector<CurvatureTriplet>& keypoints);

//void computeCurvature(PointCloud<PointXYZI> in, float kernelFactor, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0, bool isMaxScale);
void computeCurvature(pcl::PointCloud<pcl::PointXYZI> in, std::vector<float> gaussian1[NUM_GAUSSIANS], std::vector<float> gaussian2[NUM_GAUSSIANS], float kernelFactor, 
	std::vector<float>& curvature, std::vector<float>& s, std::vector<CurvatureTriplet>& keypoints, bool isMaxScale);

//void computeScaleSpace(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES]);

//void computeScaleSpace(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float>& s);
void computeScaleSpace(pcl::PointCloud<pcl::PointXYZI> in, std::vector<float> gaussian1[NUM_GAUSSIANS], std::vector<float> gaussian2[NUM_GAUSSIANS], 
	std::vector<CurvatureTriplet> keypoints[NUM_SCALES], std::vector<float>& s);


//void getFinalKeypointsAtMinScale(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], PointCloud<PointXYZI>& keypointsCloud);
void getFinalKeypointsAtMinScale(pcl::PointCloud<pcl::PointXYZI> in, std::vector<int> clusterIndices, std::vector<CurvatureTriplet> keypoints[NUM_SCALES], std::vector<float> s, std::vector<CSSKeypoint>& sharpKeypoints);

//void clusterize(pcl::PointCloud<pcl::PointXYZI> cloud, std::vector< pcl::PointCloud<pcl::PointXYZI> >& clusters, int minClusterPoints, int maxClusterPoints);
void clusterize(pcl::PointCloud<pcl::PointXYZI> cloud, std::vector< pcl::PointCloud<pcl::PointXYZI> >& clusters, std::vector< std::vector<int> >& clusterIndices, std::vector<int>& pointPicked, int minClusterPoints, int maxClusterPoints);

#endif
