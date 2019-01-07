// css.cpp
#include <css_loam_velodyne/css.h>

using namespace std;
using namespace pcl;
using namespace Eigen;


bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j)
{
	return (i.sIndex<j.sIndex);
}

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j)
{ 
	return (i.counter>j.counter);
}

bool compareCSSKeypoint (CSSKeypoint i, CSSKeypoint j)
{
	return (i.curvature>j.curvature);//Descendent order
}

int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	int idx1 = -1;
	float diff;
	float minDiff = FLOAT_MAX;

	int sumIdx = 0;
	int counter = 0;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		
		if(diff<=threshold && diff<minDiff)
		//if(diff<threshold && diff<minDiff)
		{
			idx1 = vec[i].index;
			minDiff = diff;//store the minimum difference
		}
	}

	return idx1;
}

int findByThresholdAtMinScale(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	int idx1 = -1;
	float diff;

	int sumIdx = 0;
	int counter = 0;

	float maxCurvature = FLOAT_MIN;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		
		if(diff<1e-6)
		{
			return vec[i].index;
		}
		else if(diff<=threshold)
		{
			if(i<2 || i>vec.size()-2)
				return vec[i].index;

			sumIdx += vec[i].index;
			counter++;
		}
	}

	idx1 = (counter==0)?-1:(sumIdx/counter);
	
	return idx1;
}

int findCurvatureTriplet (vector<CurvatureTriplet> vec, CurvatureTriplet c)
{
  for(int i=0; i<vec.size(); ++i)
  	if(vec[i].sIndex==c.sIndex)
  		return i;
  return -1;
}

void parametrizeCurve(PointCloud<PointXYZI> in, vector<float> &s)
{
	s = vector<float>(in.size());
	vector<float> d(in.size());

	float Dp = 0.0;
	float sumDistances = 0.0;

	d[0] = 0.0;
	s[0] = 0.0;	

	PointXYZI a, b;

	for(int i=1; i<in.size(); ++i)
	{
		/*a.x = in[i].x; b.x = in[i-1].x;
		a.y = in[i].y; b.y = in[i-1].y;
		a.z = 0.0   ;  b.z = 0.0;*/
		a.x = in[i].x; b.x = in[i-1].x;
		a.z = in[i].z; b.z = in[i-1].z;
		a.y = 0.0   ;  b.y = 0.0;
		d[i] = pcl::euclideanDistance(a, b);
		Dp += d[i];
	}

	for(int i=1; i<in.size(); ++i)
	{
		for(int j=1; j<=i; ++j)
			sumDistances += d[j];

		s[i] = sumDistances/Dp;
		sumDistances = 0.0;
	}
}

void printKeypoints(vector<CurvatureTriplet> keypoints)
{
	for(int k=0; k<keypoints.size(); ++k)
		cout<<keypoints[k].index<<"->"<<keypoints[k].sIndex<<":"<<keypoints[k].curvature<<", ";

	cout<<endl<<endl;
}

void printVector(vector<float> v)
{
	for(int i=0; i<v.size(); ++i)
		cout<<v[i]<<endl;

}

void printArray(float v[], int size)
{
	for(int i=0; i<size; ++i)
		cout<<v[i]<<endl;
}


void getCurvatureExtrema(vector<float> curvature, vector<float> s, vector<CurvatureTriplet>& keypoints, float min, float max, bool isMaxScale)
{
	for(int i=2; i<curvature.size()-2; ++i)
	{
		if( abs(curvature[i])>abs(curvature[i-1]) && abs(curvature[i])>abs(curvature[i+1]) &&
			abs(curvature[i])>abs(curvature[i-2]) && abs(curvature[i])>abs(curvature[i+2])
		  )
		{
		
			CurvatureTriplet c;
				
			if(isMaxScale)
			{
				if( (curvature[i]>0 && curvature[i]>(max/2.3) && curvature[i]>abs(min/4.0)) ||
				    (curvature[i]<0 && curvature[i]<(min/2.3) && abs(curvature[i])>(max/4.0))
				  )
				{
					if( abs(curvature[i])>1.2e-3 )
					{
						c.index = i;
						c.sIndex = s[i];
						c.curvature = curvature[i];
						keypoints.push_back(c);
					}
				}
			}
			else
			{
				c.index = i;
				c.sIndex = s[i];
				c.curvature = curvature[i];
				keypoints.push_back(c);
			}

		}
	}
}

void computeCurvature(PointCloud<PointXYZI> in, vector<float> gaussian1[NUM_GAUSSIANS], vector<float> gaussian2[NUM_GAUSSIANS],
	float kernelFactor, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, bool isMaxScale)
{
	//kernel factor = percentage of the points to define the width and std deviation of gaussian kernel
	int m = in.size();
	int kernelWidth = ((int)((float)(m)*kernelFactor)%2==0)?(m*kernelFactor+1):(m*kernelFactor);
	float sigma;

	float sum = 0.0;

	int extension = kernelWidth/2;
	PointCloud<PointXYZI> tmp;
	tmp.width = m + extension*2;
	tmp.height = 1;
	tmp.resize(tmp.width * tmp.height);

	float x1=0;
	float y1=0;
	float x2=0;
	float y2=0;
	float a,b;

	int i, j, k;
	curvature = vector<float>(in.points.size());

	for(int i=0; i<extension; ++i)
	{
		tmp[i] = PointXYZI(in.points[0]);
	}

	for(int i=0; i<in.points.size(); ++i)
	{
		tmp[i+extension] = PointXYZI(in.points[i]);
	}

	for(int i=0; i<extension; ++i)
	{
		tmp[i+m+extension] = PointXYZI(in.points[in.points.size()-1]);
	}


	float min = FLOAT_MAX;
	float max = FLOAT_MIN;

	//bool containsZeroCurvature = false;

	//convolvedCurveX = vector<float>(m);
	//convolvedCurveY = vector<float>(m);
  	
	for(i = 0; i < in.size(); ++i)
	{
		//Convolution
		for(j = 0; j<kernelWidth; ++j)
		{	
			/*x1 += tmp.points[i+j].x * gaussian1[kernelWidth/2-1][j];
			y1 += tmp.points[i+j].y * gaussian1[kernelWidth/2-1][j];
			x2 += tmp.points[i+j].x * gaussian2[kernelWidth/2-1][j];
			y2 += tmp.points[i+j].y * gaussian2[kernelWidth/2-1][j];*/
			
			x1 += tmp.points[i+j].x * gaussian1[kernelWidth/2-1][j];
			y1 += tmp.points[i+j].z * gaussian1[kernelWidth/2-1][j];
			x2 += tmp.points[i+j].x * gaussian2[kernelWidth/2-1][j];
			y2 += tmp.points[i+j].z * gaussian2[kernelWidth/2-1][j];
		}

		//convolvedCurveX[i] = x0;
		//convolvedCurveY[i] = y0;

		//curvature[i] = (x1*y2-y1*x2);
		curvature[i] = (x1*y2-y1*x2) / pow((x1*x1+y1*y1), 1.5);	

		x1 = 0.0;
		y1 = 0.0;
		x2 = 0.0;
		y2 = 0.0;

		//if(isMaxScale && abs(curvature[i])<1e6)
			//containsZeroCurvature=true;

		if(curvature[i]<min)
			min = curvature[i];
		if(curvature[i]>max)
			max = curvature[i];

	}

	getCurvatureExtrema(curvature, s, keypoints, min, max, isMaxScale);
}

void computeScaleSpace(PointCloud<PointXYZI> in, vector<float> gaussian1[NUM_GAUSSIANS], vector<float> gaussian2[NUM_GAUSSIANS],
	vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float>& s)
{
	float scales[NUM_SCALES] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	//float scales[NUM_SCALES] = {0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
	//float scales[NUM_SCALES] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75};

	//float scales[NUM_SCALES] = {0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};

	vector<float> curvature;
	//vector<float> s;
 	
 	parametrizeCurve(in, s);

 	vector<float> gauss;
	vector<float> kernel0;

 	//iterate all scales
 	for(int i=0; i<NUM_SCALES; ++i)
	{
		curvature.clear();

		if(i==0 && in.size()<40)
			continue;

		//compute curvatures and store keypoints of current scale in keypoints[i]
		if(i!=NUM_SCALES-1)
			computeCurvature(in, gaussian1, gaussian2, scales[i], curvature, s, keypoints[i], false);
		else
			computeCurvature(in, gaussian1, gaussian2, scales[i], curvature, s, keypoints[i], true);
		
		//printKeypoints(keypoints[i]);
	}
}

void getFinalKeypointsAtMinScale(PointCloud<PointXYZI> in, std::vector<int> clusterIndices, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float> s, vector<CSSKeypoint>& sharpKeypoints)
{
	vector<CurvatureTriplet> keypointsScaleIterator; //the scale iterator will always contain the number of elements of the final keypoints
	keypointsScaleIterator = keypoints[NUM_SCALES-1];//initialize the keypointsScaleIterator = keypoints at highest scale, which are the final keypoints
	
	vector<CurvatureCounter> keypointsFinalCounter(keypointsScaleIterator.size());

	//initialize keypointsFinalCounter
	for(int i=0; i<keypointsFinalCounter.size(); ++i)
	{
		CurvatureCounter cc;
		cc.index = keypointsScaleIterator[i].index;
		cc.curvature = keypointsScaleIterator[i].curvature;
		cc.counter = 0;

		keypointsFinalCounter[i] = cc;
	}

	int idx;
	//int idxAux = -1;
	float threshold = 0.0;
	
	//Serch from the highest scale to the finest to find the final keypoints
	for(int i=NUM_SCALES-1; i>0; --i)
	{
		for(int j=0; j<keypointsScaleIterator.size(); ++j)
		{
			if(keypointsScaleIterator[j].index>1 && keypointsScaleIterator[j].index<in.size()-2)
			{
				//see if we choose as threshold, the difference to the left or to the right
				threshold = abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index-2])>abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index+2])?
						abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index-2]):abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index+2]);
			}

			//cout<<"threshold final: "<<threshold<<endl;

			if(i==1)
				idx = findByThresholdAtMinScale(keypointsScaleIterator[j], keypoints[i-1], threshold);
			else
				idx = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], threshold);
	
			if(idx!=-1)
			{
				keypointsScaleIterator[j].index = idx; //update the new keypoint selected in a lower scale
				keypointsScaleIterator[j].sIndex = s[idx]; //update the new keypoint selected in a lower scale

				keypointsFinalCounter[j].index = idx;
				keypointsFinalCounter[j].counter = keypointsFinalCounter[j].counter + 1;
				
			}
			
			//cout<<keypointsFinalCounter[j].index<<"->"<<keypointsFinalCounter[j].counter<<", ";	
		}

		//cout<<endl;
	}

	//keypointsCloud.clear();

	//First point of the cluster
	/*CSSKeypoint cssKp;
	cssKp.index = clusterIndices[0];
	cssKp.curvature = 0;
	cssKp.point = in[0];
	sharpKeypoints.push_back(cssKp);*/

	for(int i=0; i<keypointsFinalCounter.size(); ++i)
	{
		//if(keypointsFinalCounter[j].counter>6)
		//if(keypointsFinalCounter[j].counter>10) //keypoints must have been found in at least 10 scales out of 16
			//keypointsCloud.push_back(in[keypointsFinalCounter[j].index]);
		//if(keypointsFinalCounter[i].counter>14) //keypoints must have been found in at least 10 scales out of 16
		//if(keypointsFinalCounter[i].counter>3) //keypoints must have been found in at least 10 scales out of 16
		//if(keypointsFinalCounter[i].counter>13) //keypoints must have been found in at least 10 scales out of 16
		//if(keypointsFinalCounter[i].counter>13) //keypoints must have been found in at least 10 scales out of 16

		//if(keypointsFinalCounter[i].counter>13) //keypoints must have been found in at least 10 scales out of 16
		if(keypointsFinalCounter[i].counter>13) //keypoints must have been found in at least 10 scales out of 16
		{
			CSSKeypoint cssKp;
			//cssKp.index = keypointsFinalCounter[i].index;
			cssKp.index = clusterIndices[keypointsFinalCounter[i].index];
			//cssKp.curvature = abs(keypointsFinalCounter[i].curvature);
			cssKp.curvature = keypointsFinalCounter[i].counter;
			cssKp.point = in[keypointsFinalCounter[i].index];
			sharpKeypoints.push_back(cssKp);
		}
	}

	//Last point of the cluster
	/*cssKp.index = clusterIndices[in.size()-1];
	cssKp.curvature = 0;
	cssKp.point = in[in.size()-1];
	sharpKeypoints.push_back(cssKp);*/
}

//void clusterize(pcl::PointCloud<pcl::PointXYZI> cloud, std::vector< pcl::PointCloud<pcl::PointXYZI> >& clusters, int minClusterPoints, int maxClusterPoints)
void clusterize(pcl::PointCloud<pcl::PointXYZI> cloud, std::vector< pcl::PointCloud<pcl::PointXYZI> >& clusters, std::vector< std::vector<int> >& clusterIndices,
	std::vector<int>& pointPicked, int minClusterPoints, int maxClusterPoints)
{
    bool isDiscontinuous[cloud.size()];
    float Dmax;
    float r;
    float deltaPhi;
    float stdDev = 0.01;
    float lambda = 10 * M_PI / 180.0;
    pcl::PointXYZI p0,p1;
    int numClusters = 0;

    /*for(int i = cloud->size() ; i >= 0 ; i--){
    	if(pcl::euclideanDistance((*cloud)[i],origin)<1.0)
      		cloud->erase(cloud->begin()+i);
  	}*/

  	//Begin Remove occluded points
	//std::cout<<"cloud size: "<<cloud.size()<<std::endl;
	for (int i = 5; i < cloud.size() - 6; ++i)
	{
		//std::cout<<"HOLA!!: "<<cloud.size()<<std::endl;
	    float diffX = (cloud)[i + 1].x - (cloud)[i].x;
	    float diffY = (cloud)[i + 1].y - (cloud)[i].y;
	    float diffZ = (cloud)[i + 1].z - (cloud)[i].z;
	    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

	    if (diff > 0.1) {

	      float depth1 = sqrt((cloud)[i].x * (cloud)[i].x +
	                     (cloud)[i].y * (cloud)[i].y +
	                     (cloud)[i].z * (cloud)[i].z);

	      float depth2 = sqrt((cloud)[i + 1].x * (cloud)[i + 1].x +
	                     (cloud)[i + 1].y * (cloud)[i + 1].y +
	                     (cloud)[i + 1].z * (cloud)[i + 1].z);

	      if (depth1 > depth2) {
	        diffX = (cloud)[i + 1].x - (cloud)[i].x * depth2 / depth1;
	        diffY = (cloud)[i + 1].y - (cloud)[i].y * depth2 / depth1;
	        diffZ = (cloud)[i + 1].z - (cloud)[i].z * depth2 / depth1;

	        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
	          pointPicked[i - 5] = 1;
          	  pointPicked[i - 4] = 1;
              pointPicked[i - 3] = 1;
              pointPicked[i - 2] = 1;
              pointPicked[i - 1] = 1;
          	  pointPicked[i - 0] = 1;
	        }
	      } else {
	        diffX = (cloud)[i + 1].x * depth1 / depth2 - (cloud)[i].x;
	        diffY = (cloud)[i + 1].y * depth1 / depth2 - (cloud)[i].y;
	        diffZ = (cloud)[i + 1].z * depth1 / depth2 - (cloud)[i].z;

	        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
	          pointPicked[i + 1] = 1;
          	  pointPicked[i + 2] = 1;
              pointPicked[i + 3] = 1;
              pointPicked[i + 4] = 1;
              pointPicked[i + 5] = 1;
          	  pointPicked[i + 6] = 1;
	        }
	      }
	    }

	    float diffX2 = (cloud)[i].x - (cloud)[i - 1].x;
	    float diffY2 = (cloud)[i].y - (cloud)[i - 1].y;
	    float diffZ2 = (cloud)[i].z - (cloud)[i - 1].z;
	    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

	    float dis = (cloud)[i].x * (cloud)[i].x
	              + (cloud)[i].y * (cloud)[i].y
	              + (cloud)[i].z * (cloud)[i].z;

	    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
	      pointPicked[i] = 1;
	    }
	}
	//End Remove ocludded points


    for(int i = 0 ; i < cloud.size() ; ++i)
    //for(int i = 5 ; i < cloud.size()-5 ; ++i)
    {
      p0.x = (cloud)[i].x;
      p0.y = 0;
      p0.z = (cloud)[i].z;
      p0.intensity = (cloud)[i].intensity;

      p1.x = (cloud)[(i+1)%cloud.size()].x;
      p1.y = 0;
      p1.z = (cloud)[(i+1)%cloud.size()].z;
      p1.intensity = (cloud)[(i+1)%cloud.size()].intensity;


      Vector3f v0(p0.x, p0.y, p0.z);
      Vector3f v1(p1.x, p1.y, p1.z);
      deltaPhi = atan2( (v1.cross(v0)).norm(), v1.dot(v0) );

      r = sqrt( p0.x*p0.x + p0.z*p0.z );
      Dmax = r * (sin(deltaPhi)/sin(lambda-deltaPhi)) + 3*stdDev;

        //if(pcl::euclideanDistance(p0,p1)>Dmax)
      if(pcl::euclideanDistance(p0,p1)-Dmax>0.01)
      {
        isDiscontinuous[(i+1)%cloud.size()] = true;
        numClusters++;
      }else{
        isDiscontinuous[(i+1)%cloud.size()] = false;
      }
    }

    int clustersArray[numClusters];
    int n = 0;

    pcl::PointCloud<pcl::PointXYZI> cloudCluster;
    std::vector<int> cloudIdx;

    //int newSize = 0;
    //Para que cuando es 1 solo cluster lo guarde
    if(numClusters==1)
    {
      if(cloud.size()>=minClusterPoints && cloud.size()<=maxClusterPoints)
      {
      	for(int i=0; i<cloud.size(); ++i)
      	{
        	//cloud_cluster = cloud;
        	cloudCluster.push_back(cloud[i]);
        	cloudIdx.push_back(i);
       		//newSize++;
        }
      	//cloudCluster = cloud;
        clusters.push_back(cloudCluster);
        clusterIndices.push_back(cloudIdx);
      }

      return;
    }
      
    for(int i = 0; i < cloud.size(); ++i)
      if(isDiscontinuous[i])
          clustersArray[n++]=i;

    for(int i=0; i<numClusters; ++i)
    {
      int pointsPerCluster = (clustersArray[(i+1)%numClusters]-clustersArray[i]+cloud.size())%cloud.size();//numero de puntos de cada cluster
      
      if (pointsPerCluster<minClusterPoints || pointsPerCluster>maxClusterPoints)
        continue;
    
      cloudCluster.clear();
      cloudIdx.clear();
      
      for(int j=clustersArray[i]; j!=clustersArray[(i+1)%numClusters]; j=(j+1)%cloud.size())
      {
        cloudCluster.push_back((cloud)[j]);
        cloudIdx.push_back(j);
        //newSize++;
      }

      clusters.push_back(cloudCluster);
      clusterIndices.push_back(cloudIdx);
    }
}