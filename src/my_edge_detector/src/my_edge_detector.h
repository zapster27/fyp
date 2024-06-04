#ifndef MY_EDGE_DETECTOR_H
#define MY_EDGE_DETECTOR_H

#include <pcl/surface/mls.h>  
#include <pcl/filters/statistical_outlier_removal.h>  
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/don.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <pcl/features/normal_3d_omp.h>

class MyEdgeDetector {
public:
    MyEdgeDetector(ros::NodeHandle& nh);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

private:
    ros::Publisher pub_planes_;
    ros::Publisher pub_intersections_;
    ros::Publisher pub_combined_edges_;
    ros::Publisher pub_doncloud_;
    ros::Publisher pub_fitted_curves_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_clouds_;
    std::vector<Eigen::Vector4f> plane_coefficients_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr don_edges_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_lines_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr synthesized_edges_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_edges_;

    void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered);
    void detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, const std::string& frame_id);
    void detectIntersectionEdges(const std::string& frame_id);
    void detectDoNEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id);
    void synthesizeEdges(const std::string& frame_id);
    void combineEdges(const std::string& frame_id);
    void clusterAndFitLines(const std::string& frame_id);
    void applyMovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_smoothed);
    void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered);
};

#endif // MY_EDGE_DETECTOR_H