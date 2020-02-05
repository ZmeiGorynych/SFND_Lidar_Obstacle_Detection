//
// Created by Egor Kraev on 2/5/20.
//
#include <vector>
#include <unordered_set>
#include <pcl/visualization/pcl_visualizer.h>
#include "../../processPointClouds.h"
#ifndef PLAYBACK_MY_RANSAC_H
#define PLAYBACK_MY_RANSAC_H


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

template<typename PointT>
std::unordered_set<int> MyRansac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                 DistanceModel<PointT> & model,
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
        std::vector<PointT> points;
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
#endif //PLAYBACK_MY_RANSAC_H
