#include <ros/ros.h>
#include <Eigen/Core>
#include <amcl3d/KdTreeNanoPclWrapper.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/plane_clipper3D.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <algorithm>
#include <queue>
#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <limits>

#define PointType pcl::PointXYZI

int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_registration_node");
    ros::NodeHandle n;

    ros::Rate t(100);

    Eigen::Matrix4f m;
    m << 0,  0,  1, 0,
         1,  0,  0, 0,
         0,  1,  0, 0,
         0,  0,  0, 1;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

    pcl::io::loadPCDFile("/home/ayush/Downloads/finalCloudCorrected.pcd", *cloud);
    pcl::PassThrough<PointType> m_passThroughZ;
    m_passThroughZ.setFilterFieldName("z");
    m_passThroughZ.setFilterLimits(-1.0, 10.0);
    m_passThroughZ.setInputCloud(cloud);
    m_passThroughZ.filter(*cloud);
    // pcl::transformPointCloud(*cloud, *cloud, m);

    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ())
    {
    }
    pcl::io::savePCDFileASCII("/home/ayush/Downloads/finalCloud.pcd", *cloud);

}