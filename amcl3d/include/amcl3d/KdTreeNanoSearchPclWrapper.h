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
 * KdTreeNanoSearchPclWrapper.h
 *
 *  Created on: Jul 18, 2020
 *      Author: ayush.garg
 */

#ifndef PCL_KDTREE_NANO_SEARCH_PCL_WRAPPER_H_
#define PCL_KDTREE_NANO_SEARCH_PCL_WRAPPER_H_

#include <amcl3d/KdTreeNanoPclWrapper.h>
#include <pcl/search/search.h>

namespace pcl{
  namespace search{
    class KdTreeNanoSearch : public Search<pcl::PointXYZ> {
     public:
      using PointCloud = typename Search<pcl::PointXYZ>::PointCloud;
      using PointCloudConstPtr = typename Search<pcl::PointXYZ>::PointCloudConstPtr;

      using pcl::search::Search<pcl::PointXYZ>::indices_;
      using pcl::search::Search<pcl::PointXYZ>::input_;
      using pcl::search::Search<pcl::PointXYZ>::getIndices;
      using pcl::search::Search<pcl::PointXYZ>::getInputCloud;
      using pcl::search::Search<pcl::PointXYZ>::nearestKSearch;
      using pcl::search::Search<pcl::PointXYZ>::radiusSearch;
      using pcl::search::Search<pcl::PointXYZ>::sorted_results_;

      using Ptr = boost::shared_ptr<KdTreeNanoSearch>;
      using ConstPtr = boost::shared_ptr<const KdTreeNanoSearch>;
      using Indices = std::vector<int>;

      using KdTreeNanoPtr = typename pcl::KdTreeNano<pcl::PointXYZ>::Ptr;
      using KdTreeNanoConstPtr = typename pcl::KdTreeNano<pcl::PointXYZ>::ConstPtr;
      using PointRepresentationConstPtr = typename PointRepresentation<pcl::PointXYZ>::ConstPtr;

      /** \brief Constructor for KdTreenanoSearch. 
       *
       * \param[in] sorted set to true if the nearest neighbor search results
       * need to be sorted in ascending order based on their distance to the
       * query point
       *
       */
      KdTreeNanoSearch (bool sorted = true); 

      /** \brief Destructor for KdTreenanoSearch. */
      
      ~KdTreeNanoSearch ()
      {
      }

      /** \brief Provide a pointer to the point representation to use to convert points into k-D vectors. 
       * \param[in] point_representation the const boost shared pointer to a PointRepresentation
       */
      void
      setPointRepresentation (const PointRepresentationConstPtr &point_representation);

      /** \brief Get a pointer to the point representation used when converting points into k-D vectors. */
      inline PointRepresentationConstPtr
      getPointRepresentation () const
      {
      return (tree_->getPointRepresentation ());
      }

      /** \brief Sets whether the results have to be sorted or not.
       * \param[in] sorted_results set to true if the radius search results should be sorted
       */
      void 
      setSortedResults (bool sorted_results) override;
      
      /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
       * \param[in] eps precision (error bound) for nearest neighbors searches
       */
      void
      setEpsilon (float eps);

      /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
      inline float
      getEpsilon () const
      {
      return (tree_->getEpsilon ());
      }

      /** \brief Provide a pointer to the input dataset.
       * \param[in] cloud the const boost shared pointer to a PointCloud message
       * \param[in] indices the point indices subset that is to be used from \a cloud 
       */
      void
      setInputCloud (const PointCloudConstPtr& cloud, 
                      const IndicesConstPtr& indices = IndicesConstPtr ()) override;

      /** \brief Search for the k-nearest neighbors for the given query point.
       * \param[in] point the given query point
       * \param[in] k the number of neighbors to search for
       * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
       * a priori!)
       * \return number of neighbors found
       */
      int
      nearestKSearch (const pcl::PointXYZ &point, int k, 
                      Indices &k_indices,
                      std::vector<float> &k_sqr_distances) const override;

      /** \brief Search for all the nearest neighbors of the query point in a given radius.
       * \param[in] point the given query point
       * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
       * \param[out] k_indices the resultant indices of the neighboring points
       * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
       * \param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to
       * 0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be
       * returned.
       * \return number of neighbors found in radius
       */
      int
      radiusSearch (const pcl::PointXYZ& point, double radius, 
                  Indices &k_indices,
                  std::vector<float> &k_sqr_distances,
                  unsigned int max_nn = 0) const override;
     protected:
      /** \brief A pointer to the internal KdTree object. */
      KdTreeNanoPtr tree_;
    };
  };
}

#endif /* PCL_KDTREE_NANO_SEARCH_PCL_WRAPPER_H_ */