/**
 * KdTreeNanoSearchPclWrapper.cpp
 *
 *  Created on: Jul 18, 2020
 *      Author: ayush.garg
 */

#include <amcl3d/KdTreeNanoSearchPclWrapper.h>
#include <pcl/search/impl/search.hpp>

///////////////////////////////////////////////////////////////////////////////////////////
pcl::search::KdTreeNanoSearch::KdTreeNanoSearch (bool sorted)
  : pcl::search::Search<pcl::PointXYZ> ("KdTreeNanoSearch", sorted)
  , tree_ (new pcl::KdTreeNano<pcl::PointXYZ> (sorted))
{}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::search::KdTreeNanoSearch::setPointRepresentation (
    const PointRepresentationConstPtr &point_representation)
{
  tree_->setPointRepresentation (point_representation);
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::search::KdTreeNanoSearch::setSortedResults (bool sorted_results)
{
  sorted_results_ = sorted_results;
  tree_->setSortedResults (sorted_results);
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::search::KdTreeNanoSearch::setEpsilon (float eps)
{
  tree_->setEpsilon (eps);
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::search::KdTreeNanoSearch::setInputCloud (
    const PointCloudConstPtr& cloud, 
    const IndicesConstPtr& indices)
{
  tree_->setInputCloud (cloud, indices);
  input_ = cloud;
  indices_ = indices;
}

///////////////////////////////////////////////////////////////////////////////////////////
int pcl::search::KdTreeNanoSearch::nearestKSearch (
    const pcl::PointXYZ &point, int k, Indices &k_indices,
    std::vector<float> &k_sqr_distances) const
{
  return (tree_->nearestKSearch (point, k, k_indices, k_sqr_distances));
}

///////////////////////////////////////////////////////////////////////////////////////////
int pcl::search::KdTreeNanoSearch::radiusSearch (
    const pcl::PointXYZ& point, double radius, 
    Indices &k_indices, std::vector<float> &k_sqr_distances,
    unsigned int max_nn) const
{
  return (tree_->radiusSearch (point, radius, k_indices, k_sqr_distances, max_nn));
}