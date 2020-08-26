/*
 * NOTICE: This software  source code and any of  its derivatives are the
 * confidential  and  proprietary   information  of  Vecna  Technologies,
 * Inc. (such source  and its derivatives are hereinafter  referred to as
 * "Confidential Information"). The  Confidential Information is intended
 * to be  used exclusively by  individuals or entities that  have entered
 * into either  a non-disclosure agreement or license  agreement (or both
 * of  these agreements,  if  applicable) with  Vecna Technologies,  Inc.
 * ("Vecna")   regarding  the  use   of  the   Confidential  Information.
 * Furthermore,  the  Confidential  Information  shall be  used  only  in
 * accordance  with   the  terms   of  such  license   or  non-disclosure
 * agreements.   All  parties using  the  Confidential Information  shall
 * verify that their  intended use of the Confidential  Information is in
 * compliance  with and  not in  violation of  any applicable  license or
 * non-disclosure  agreements.  Unless expressly  authorized by  Vecna in
 * writing, the Confidential Information  shall not be printed, retained,
 * copied, or  otherwise disseminated,  in part or  whole.  Additionally,
 * any party using the Confidential  Information shall be held liable for
 * any and  all damages incurred  by Vecna due  to any disclosure  of the
 * Confidential  Information (including  accidental disclosure).   In the
 * event that  the applicable  non-disclosure or license  agreements with
 * Vecna  have  expired, or  if  none  currently  exists, all  copies  of
 * Confidential Information in your  possession, whether in electronic or
 * printed  form, shall be  destroyed or  returned to  Vecna immediately.
 * Vecna  makes no  representations  or warranties  hereby regarding  the
 * suitability  of  the   Confidential  Information,  either  express  or
 * implied,  including  but not  limited  to  the  implied warranties  of
 * merchantability,    fitness    for    a   particular    purpose,    or
 * non-infringement. Vecna  shall not be liable for  any damages suffered
 * by  licensee as  a result  of  using, modifying  or distributing  this
 * Confidential Information.  Please email [info@vecnatech.com]  with any
 * questions regarding the use of the Confidential Information.
 */

/**
 * KdTreeNanoPclWrapper.h
 *
 *  Created on: Jul 18, 2020
 *      Author: ayush.garg
 */

#ifndef PCL_KDTREE_NANO_PCL_WRAPPER_H_
#define PCL_KDTREE_NANO_PCL_WRAPPER_H_

// pcl inlcudes
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

// Nano includes
// #include <amcl3d/nanoflann.hpp>
#include <amcl3d/nanoflann.hpp>

namespace pcl
{
// Forward declarations
template <typename T> class PointRepresentation;

template <typename PointT>
class NanoPointCloudAdaptor {
 public:
  typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
  typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef boost::shared_ptr<PointCloud> PointCloudPtr;
  typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

  typedef pcl::PointRepresentation<PointT> PointRepresentation;
  typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;

  NanoPointCloudAdaptor(){}

  NanoPointCloudAdaptor(const NanoPointCloudAdaptor &k){
    m_inputCloud = k.m_inputCloud;
  }

  void setInputCloud(PointCloudConstPtr cloud){
    m_inputCloud = cloud;
  }

  size_t kdtree_get_point_count() const {
    return m_inputCloud->points.size();
  }

  float kdtree_get_pt(const size_t idx, int dim) const {
    const PointT& p = m_inputCloud->points[idx];
    if (dim == 0) {
    return p.x;
    } else if (dim == 1) {
    return p.y;
    } else if (dim == 2) {
    return p.z;
    }
    return 0.0;
  }

  template<class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

 private:
  PointCloudConstPtr m_inputCloud;
};

template <typename PointT>
class KdTreeNano { 
 public:

  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, NanoPointCloudAdaptor<PointT> > , NanoPointCloudAdaptor<PointT>, 3> KdTree_t;
  typedef pcl::PointRepresentation<PointT> PointRepresentation;
  typedef boost::shared_ptr<const PointRepresentation> PointRepresentationConstPtr;
  typedef boost::shared_ptr<KdTreeNano> Ptr;
  typedef boost::shared_ptr<const KdTreeNano> ConstPtr;

  /** \brief Default Constructor for KdTreeNabo.
   * \param[in] sorted set to true if the application that the tree will be used for requires sorted nearest neighbor indices (default). False otherwise.
   *
   * By setting sorted to false, the \ref radiusSearch operations will be faster.
   */
  KdTreeNano(bool sorted = true) : m_inputCloud(), m_adaptor(), m_point_representation(new DefaultFeatureRepresentation<PointT>), m_sorted(sorted) {}

  /** \brief Copy constructor
   * \param[in] k the tree to copy into this
   */
  KdTreeNano(const KdTreeNano &k){
    m_inputCloud = k.m_inputCloud;
    m_nanoTree = k.m_nanoTree;
    m_point_representation = k.m_point_representation;
    m_adaptor = k.m_adaptor;
    m_epsilon = k.m_epsilon;
    m_sorted = k.m_sorted;
  }

  /** \brief Copy operator
   * \param[in] k the tree to copy into this
   */
  KdTreeNano& operator =(const KdTreeNano& k){
    m_inputCloud = k.m_inputCloud;
    m_nanoTree = k.m_nanoTree;
    m_point_representation = k.m_point_representation;
    m_adaptor = k.m_adaptor;
    m_epsilon = k.m_epsilon;
  }

  inline Ptr makeShared() {
    return Ptr(new KdTreeNano(*this));
  }

  /** \brief Destructor for KdTreeNabo.
   * Deletes all allocated data arrays and destroys the kd-tree structures.
   */
  virtual ~KdTreeNano() {
    m_nanoTree.reset();
    m_inputCloud.reset();
  }

  /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors.
    * \param[in] point_representation the const boost shared pointer to a PointRepresentation
    */
  // void setPointRepresentation(const boost::shared_ptr<const pcl::PointRepresentation<pcl::PointXYZ>> &point_representation);
  void setPointRepresentation(const PointRepresentationConstPtr &point_representation) const {
    // m_point_representation = point_representation;
  }

  /** \brief Sets whether the results have to be sorted or not.
    * \param[in] sorted_results set to true if the radius search results should be sorted
    */
  void setSortedResults(bool sorted_results) {
    m_sorted = sorted_results;
  }

  /** \brief Provide a pointer to the input dataset.
   * \param[in] cloud the const boost shared pointer to a PointCloud message
   * \param[in] indices the point indices subset that is to be used from \a cloud - if NULL the whole cloud is used
   */
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const IndicesConstPtr &indices = IndicesConstPtr()) {
    if(indices == IndicesConstPtr()){
      m_inputCloud = cloud;
    }
    else{
      pcl::PointCloud<pcl::PointXYZ>::Ptr tpcl(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(indices);
      extract.filter(*tpcl);
      m_inputCloud = tpcl;
    }

    m_adaptor.setInputCloud(m_inputCloud);
    m_nanoTree.reset(new KdTree_t(3 , m_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
    m_nanoTree->buildIndex();
  }

  /** \brief Search for k-nearest neighbors for the given query point.
   *
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
   *
   * \param[in] point a given \a valid (i.e., finite) query point
   * \param[in] k the number of neighbors to search for
   * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
   * a priori!)
   * \return number of neighbors found
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
   */
  int nearestKSearch(const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances) const {
    std::vector<size_t> k_indices_(k);
    k_sqr_distances.resize(k);
    size_t k_ = k;
    float out_dist_sqr[k];
    const float query[3] = {point.x, point.y, point.z};
    k = (int) m_nanoTree->knnSearch(&query[0], k_, &k_indices_[0], &k_sqr_distances[0]);
    k_indices = std::vector<int>(k_indices_.begin(), k_indices_.end());
    return k;
  }

  /** \brief Search for all the nearest neighbors of the query point in a given radius.
   *
   * \attention This method does not do any bounds checking for the input index
   * (i.e., index >= cloud.points.size () || index < 0), and assumes valid (i.e., finite) data.
   *
   * \param[in] point a given \a valid (i.e., finite) query point
   * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
   * \param[out] k_indices the resultant indices of the neighboring points
   * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
   * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
   * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
   * returned.
   * \return number of neighbors found in radius
   *
   * \exception asserts in debug mode if the index is not between 0 and the maximum number of points
   */
  int radiusSearch(const PointT &point, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
                   unsigned int max_nn = 0) const {
    const float query[3] = {point.x, point.y, point.z};
    const float search_radius = static_cast<float>(radius);
    std::vector<std::pair<size_t, float>>   ret_matches;
    nanoflann::SearchParams params;
    params.sorted = m_sorted;
    const int nMatches = m_nanoTree->radiusSearch(&query[0], search_radius, ret_matches, params);
    k_indices.resize(nMatches);
    k_sqr_distances.resize(nMatches);
    for(int i=0;i < nMatches; i++){
      k_indices[i] = ret_matches[i].first;
      k_sqr_distances[i] = ret_matches[i].second;
    }
  }

  void setEpsilon(float eps) {
    m_epsilon = eps;
  }

  float getEpsilon() const {
    return m_epsilon;
  }

  PointRepresentationConstPtr getPointRepresentation() const {
    return m_point_representation;
  }

 private:

  int m_knnRadiusSearchMaxK = 30;
  float m_epsilon;
  bool m_sorted;
  PointRepresentationConstPtr m_point_representation;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr m_inputCloud;
  NanoPointCloudAdaptor<PointT> m_adaptor;
  boost::shared_ptr<KdTree_t> m_nanoTree;
  // KdTree_t m_nanoTree(3 /*dim*/, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );


  /** \brief Class getName method. */
  virtual std::string getName() const {
    return ("KdTreeNano");
  }
};
}

#endif /* PCL_KDTREE_NANO_PCL_WRAPPER_H_ */
