/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

template<typename PointT>
class DistanceModel{
public:
    virtual void fit(std::vector<PointT> points) = 0;
    virtual float distance(PointT p) = 0;
    virtual int in_points() = 0;
};

template<typename PointT>
class Line : public DistanceModel<PointT>
        {
private:
    float a;
    float b;
    float c;
    float sqrtab;

public:
    void fit(std::vector<PointT> points);
    float distance(PointT p);
    int in_points(){return 2;};
};

template<typename PointT>
void Line<PointT>::fit(std::vector<PointT> points){
    auto p1 = points[0];
    auto p2 = points[1];
        a = p1.y-p2.y;
        b = p2.x-p1.x;
        c = p1.x*p2.y-p2.x*p1.y;
        sqrtab = sqrt(a*a+b*b);
};

template<typename PointT>
float Line<PointT>::distance(PointT p){
        float sp = a*p.x + b*p.y;
        float d = fabs(sp+c)/sqrtab;
        return d;
};

template<typename PointT>
class Plane: public DistanceModel<PointT>
        {
private:
    float a;
    float b;
    float c;
    float d;
    float sqrtabc;
public:
    void fit(std::vector<PointT> points);
    float distance(PointT p);
    int in_points(){return 3;};
};

template<typename PointT>
void Plane<PointT>::fit(std::vector<PointT> points){
    auto p1 = points[0];
    auto p2 = points[1];
    auto p3 = points[2];

    float v1x = p2.x - p1.x;
    float v1y = p2.y - p1.y;
    float v1z = p2.z - p1.z;
    float v2x = p3.x - p1.x;
    float v2y = p3.y - p1.y;
    float v2z = p3.z - p1.z;

    a = v1y*v2z - v2y*v1z;
    b = v1z*v2x - v2z*v1x;
    c = v1x*v2y - v2x*v1y;
    d = -(a*p1.x + b*p1.y + c*p1.z);
    sqrtabc = sqrt(a*a + b*b + c*c);
};

template<typename PointT>
float Plane<PointT>::distance(PointT p){
    float sp = a*p.x + b*p.y + c*p.z;
    float dist = fabs(sp+d)/sqrtabc;
    return dist;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        DistanceModel<pcl::PointXYZ> & model,
        int maxIterations,
        float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    auto startTime = std::chrono::steady_clock::now();

	// For max iterations 
    while(maxIterations--) {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        while(inliers.size() < model.in_points())
            inliers.insert(rand()%(cloud->points.size()));

//        auto itr = inliers.begin();
        std::vector<pcl::PointXYZ> points;
        for(int index: inliers)
            points.push_back(cloud->points[index]);
        model.fit(points);

        for(int index=0; index < cloud->points.size(); index++){
            // If distance is smaller than threshold count it as inlier
            if(inliers.count(index)>0)
                continue;

            if(model.distance(cloud->points[index])<=distanceTol)
                inliers.insert(index);
            }
        if(inliers.size()>inliersResult.size())
            inliersResult = inliers;
    };
	// Return indicies of inliers from fitted line with most inliers
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "My RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
    //Line<pcl::PointXYZ> line;
    Plane<pcl::PointXYZ> model;
	std::unordered_set<int> inliers = Ransac(cloud, model, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  	    std::cout << "Mysterious fail!";
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
