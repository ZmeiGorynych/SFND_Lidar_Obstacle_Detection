//
// Created by ubuntu on 2/5/20.
//

#ifndef PLAYBACK_EUCLIDEAN_CLUSTER_H
#define PLAYBACK_EUCLIDEAN_CLUSTER_H
#include <vector>
#include <memory>
#include "kdtree.h"

template<typename PointT>
std::vector<float> to_vector(PointT x){
    return std::vector<float>();
}



std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                               std::unique_ptr<KdTree>& tree,
        float distanceTol);


#endif //PLAYBACK_EUCLIDEAN_CLUSTER_H
