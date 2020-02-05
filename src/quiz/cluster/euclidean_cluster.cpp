//
// Created by ubuntu on 2/5/20.
//

#include "euclidean_cluster.h"
#include <vector>
#include <memory>
#include "kdtree.h"

void clusterHelper(int i,
                   std::vector<int>& cluster,
                   bool* processed,
                   const std::vector<std::vector<float>>& points,
                   std::unique_ptr<KdTree>& tree,
                   float distanceTol) {
    processed[i] = true;
    cluster.push_back(i);
    auto nearby = tree->search(points[i], distanceTol);
    for (int j: nearby) {
        if (!processed[j])
            clusterHelper(j, cluster, processed, points, tree, distanceTol);
    }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                               std::unique_ptr<KdTree>& tree,
                                               float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    //std::vector<bool> processed(points.size(), false);
    bool* processed = new bool[points.size()];
    for(int j=0; j<points.size(); j++)
        processed[j]=false;

    int i = 0;
    while(i<points.size()){
//find the next unprocessed point
        if(processed[i]){
            i++;
            continue;
        }
        // and do the cluster around it
        std::vector<int> next_cluster;
        clusterHelper(i, next_cluster, processed, points, tree, distanceTol);
        clusters.push_back(next_cluster);
        i++;
    }

    delete[] processed;
    return clusters;

}

template<>
std::vector<float> to_vector(pcl::PointXYZI x){
    std::vector<float> out = {x.x, x.y, x.z};
    return out;
}

template<>
std::vector<float> to_vector(pcl::PointXYZ x){
    std::vector<float> out = {x.x, x.y, x.z};
    return out;
}