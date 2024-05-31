#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

ros::Publisher pub_planes;
ros::Publisher pub_intersections;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // Downsample the cloud using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int plane_count = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<Eigen::Vector4f> plane_coefficients;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_clouds;

    while (cloud_filtered->points.size() > 50) {  // Ensure there are enough points to form a plane
        seg.setInputCloud(cloud_filtered);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_INFO("No more planes detected.");
            break;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        // Add color to the plane
        uint8_t r = (rand() % 256);
        uint8_t g = (rand() % 256);
        uint8_t b = (rand() % 256);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& point : cloud_plane->points) {
            pcl::PointXYZRGB point_rgb;
            point_rgb.x = point.x;
            point_rgb.y = point.y;
            point_rgb.z = point.z;
            point_rgb.r = r;
            point_rgb.g = g;
            point_rgb.b = b;
            colored_plane->points.push_back(point_rgb);
        }

        *colored_cloud += *colored_plane;
        plane_clouds.push_back(cloud_plane);

        // Remove the plane from the remaining cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*cloud_remaining);
        cloud_filtered.swap(cloud_remaining);

        plane_coefficients.push_back(Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));
        plane_count++;
    }

    // Publish all colored planes in one topic for visualization
    sensor_msgs::PointCloud2 all_planes_output;
    pcl::toROSMsg(*colored_cloud, all_planes_output);
    all_planes_output.header.frame_id = input->header.frame_id;
    pub_planes.publish(all_planes_output);

    // Find and publish intersection lines for each pair of planes within the segmented planes
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_lines(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < plane_coefficients.size(); ++i) {
        for (size_t j = i + 1; j < plane_coefficients.size(); ++j) {
            Eigen::Vector4f plane1 = plane_coefficients[i];
            Eigen::Vector4f plane2 = plane_coefficients[j];

            Eigen::Vector3f normal1(plane1[0], plane1[1], plane1[2]);
            Eigen::Vector3f normal2(plane2[0], plane2[1], plane2[2]);

            Eigen::Vector3f direction = normal1.cross(normal2);
            if (direction.norm() < 1e-6) {
                continue;  // Planes are parallel and do not intersect
            }

            Eigen::Matrix3f A;
            A << normal1[0], normal1[1], normal1[2],
                 normal2[0], normal2[1], normal2[2],
                 direction[0], direction[1], direction[2];

            Eigen::Vector3f b;
            b << -plane1[3], -plane2[3], 0.0;

            Eigen::Vector3f point_on_line = A.colPivHouseholderQr().solve(b);

            // Determine the range of t values for which the line stays within both segmented planes
            std::vector<float> t_values;

            for (const auto& point : plane_clouds[i]->points) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                float t = (point_vec - point_on_line).dot(direction) / direction.squaredNorm();
                t_values.push_back(t);
            }

            for (const auto& point : plane_clouds[j]->points) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                float t = (point_vec - point_on_line).dot(direction) / direction.squaredNorm();
                t_values.push_back(t);
            }

            if (t_values.empty()) {
                continue;
            }

            float min_t = *std::min_element(t_values.begin(), t_values.end());
            float max_t = *std::max_element(t_values.begin(), t_values.end());

            // Generate points along the intersection line within the valid range of t values
            for (float t = min_t; t <= max_t; t += 0.01) {
                pcl::PointXYZ point;
                point.x = point_on_line[0] + t * direction[0];
                point.y = point_on_line[1] + t * direction[1];
                point.z = point_on_line[2] + t * direction[2];

                if (point.x >= std::min(min_t, max_t) && point.x <= std::max(min_t, max_t) &&
                    point.y >= std::min(min_t, max_t) && point.y <= std::max(min_t, max_t) &&
                    point.z >= std::min(min_t, max_t) && point.z <= std::max(min_t, max_t)) {
                    intersection_lines->points.push_back(point);
                }
            }
        }
    }

    // Convert the intersection lines to ROS message
    sensor_msgs::PointCloud2 intersection_output;
    pcl::toROSMsg(*intersection_lines, intersection_output);
    intersection_output.header.frame_id = input->header.frame_id;
    pub_intersections.publish(intersection_output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 10, cloud_cb);
    pub_planes = nh.advertise<sensor_msgs::PointCloud2>("colored_planes", 1);
    pub_intersections = nh.advertise<sensor_msgs::PointCloud2>("intersection_lines", 1);

    ros::spin();
}
