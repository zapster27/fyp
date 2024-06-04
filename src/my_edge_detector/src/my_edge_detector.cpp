#include "my_edge_detector.h"

MyEdgeDetector::MyEdgeDetector(ros::NodeHandle& nh) {
    pub_planes_ = nh.advertise<sensor_msgs::PointCloud2>("colored_planes", 1);
    pub_intersections_ = nh.advertise<sensor_msgs::PointCloud2>("intersection_lines", 1);
    pub_combined_edges_ = nh.advertise<sensor_msgs::PointCloud2>("combined_edges", 1);
    pub_doncloud_ = nh.advertise<sensor_msgs::PointCloud2>("don_edges", 1);  // Added publisher for DoN edges
    pub_fitted_curves_ = nh.advertise<sensor_msgs::PointCloud2>("fitted_curves", 1);  // Added publisher for fitted curves
}

void MyEdgeDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    downsamplePointCloud(cloud, cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    applyMovingLeastSquares(cloud_filtered, cloud_smoothed);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers_removed(new pcl::PointCloud<pcl::PointXYZ>);
    removeOutliers(cloud_smoothed, cloud_outliers_removed);

    std::string frame_id = input->header.frame_id;
    detectPlanes(cloud_outliers_removed, frame_id);
    detectIntersectionEdges(frame_id);
    detectDoNEdges(cloud, frame_id);
    combineEdges(frame_id);
    synthesizeEdges(frame_id);
}

void MyEdgeDetector::downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
}

void MyEdgeDetector::detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, const std::string& frame_id) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    colored_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    plane_coefficients_.clear();
    plane_clouds_.clear();

    while (cloud_filtered->points.size() > 50) {
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

        *colored_cloud_ += *colored_plane;
        plane_clouds_.push_back(cloud_plane);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*cloud_remaining);
        cloud_filtered.swap(cloud_remaining);

        plane_coefficients_.push_back(Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));
    }

    sensor_msgs::PointCloud2 all_planes_output;
    pcl::toROSMsg(*colored_cloud_, all_planes_output);
    all_planes_output.header.frame_id = frame_id;
    pub_planes_.publish(all_planes_output);
}

void MyEdgeDetector::detectIntersectionEdges(const std::string& frame_id) {
    intersection_lines_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < plane_coefficients_.size(); ++i) {
        for (size_t j = i + 1; j < plane_coefficients_.size(); ++j) {
            Eigen::Vector4f plane1 = plane_coefficients_[i];
            Eigen::Vector4f plane2 = plane_coefficients_[j];

            Eigen::Vector3f normal1(plane1[0], plane1[1], plane1[2]);
            Eigen::Vector3f normal2(plane2[0], plane2[1], plane2[2]);

            Eigen::Vector3f direction = normal1.cross(normal2);
            if (direction.norm() < 1e-6) {
                continue;
            }

            Eigen::Matrix3f A;
            A << normal1[0], normal1[1], normal1[2],
                 normal2[0], normal2[1], normal2[2],
                 direction[0], direction[1], direction[2];

            Eigen::Vector3f b;
            b << -plane1[3], -plane2[3], 0.0;

            Eigen::Vector3f point_on_line = A.colPivHouseholderQr().solve(b);

            std::vector<float> t_values;
            for (const auto& point : plane_clouds_[i]->points) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                float t = (point_vec - point_on_line).dot(direction) / direction.squaredNorm();
                t_values.push_back(t);
            }

            for (const auto& point : plane_clouds_[j]->points) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                float t = (point_vec - point_on_line).dot(direction) / direction.squaredNorm();
                t_values.push_back(t);
            }

            if (t_values.empty()) {
                continue;
            }

            float min_t = *std::min_element(t_values.begin(), t_values.end());
            float max_t = *std::max_element(t_values.begin(), t_values.end());

            for (float t = min_t; t <= max_t; t += 0.01) {
                pcl::PointXYZ point;
                point.x = point_on_line[0] + t * direction[0];
                point.y = point_on_line[1] + t * direction[1];
                point.z = point_on_line[2] + t * direction[2];
                intersection_lines_->points.push_back(point);
            }
        }
    }

    sensor_msgs::PointCloud2 intersection_output;
    pcl::toROSMsg(*intersection_lines_, intersection_output);
    intersection_output.header.frame_id = frame_id;
    pub_intersections_.publish(intersection_output);
}

void MyEdgeDetector::detectDoNEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id) {
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.02);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.compute(*normals_small);

    ne.setRadiusSearch(0.04);
    ne.compute(*normals_large);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud, *doncloud);
    for (size_t i = 0; i < doncloud->points.size(); ++i) {
        doncloud->points[i].normal_x = normals_large->points[i].normal_x - normals_small->points[i].normal_x;
        doncloud->points[i].normal_y = normals_large->points[i].normal_y - normals_small->points[i].normal_y;
        doncloud->points[i].normal_z = normals_large->points[i].normal_z - normals_small->points[i].normal_z;
    }

    don_edges_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : doncloud->points) {
        if (std::sqrt(point.normal_x * point.normal_x + point.normal_y * point.normal_y + point.normal_z * point.normal_z) > 0.2) {
            pcl::PointXYZ edge_point;
            edge_point.x = point.x;
            edge_point.y = point.y;
            edge_point.z = point.z;
            don_edges_->points.push_back(edge_point);
        }
    }

    // Publish the DoN edges
    sensor_msgs::PointCloud2 don_output;
    pcl::toROSMsg(*don_edges_, don_output);
    don_output.header.frame_id = frame_id;
    pub_doncloud_.publish(don_output);
}

void MyEdgeDetector::synthesizeEdges(const std::string& frame_id) {
    synthesized_edges_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(don_edges_);

    for (const auto& point : intersection_lines_->points) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(point, 0.05, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            pcl::PointXYZ avg_point;
            avg_point.x = avg_point.y = avg_point.z = 0.0;
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                avg_point.x += don_edges_->points[pointIdxRadiusSearch[i]].x;
                avg_point.y += don_edges_->points[pointIdxRadiusSearch[i]].y;
                avg_point.z += don_edges_->points[pointIdxRadiusSearch[i]].z;
            }
            avg_point.x /= pointIdxRadiusSearch.size();
            avg_point.y /= pointIdxRadiusSearch.size();
            avg_point.z /= pointIdxRadiusSearch.size();
            synthesized_edges_->points.push_back(avg_point);
        }
    }

    sensor_msgs::PointCloud2 synthesized_output;
    pcl::toROSMsg(*synthesized_edges_, synthesized_output);
    synthesized_output.header.frame_id = frame_id;
    pub_combined_edges_.publish(synthesized_output);  // Reusing combined_edges_ publisher for synthesized edges
}

void MyEdgeDetector::combineEdges(const std::string& frame_id) {
    combined_edges_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    *combined_edges_ += *don_edges_;
    *combined_edges_ += *intersection_lines_;

    sensor_msgs::PointCloud2 combined_edges_output;
    pcl::toROSMsg(*combined_edges_, combined_edges_output);
    combined_edges_output.header.frame_id = frame_id;
    pub_combined_edges_.publish(combined_edges_output);
}

void MyEdgeDetector::applyMovingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_smoothed) {
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.03);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005);
    mls.setUpsamplingStepSize(0.003);
    mls.process(*cloud_smoothed);
}

void MyEdgeDetector::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_edge_detector_node");
    ros::NodeHandle nh;

    MyEdgeDetector detector(nh);

    // ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 100, &MyEdgeDetector::cloudCallback, &detector);
    ros::Subscriber sub = nh.subscribe("/camera/ahat", 100, &MyEdgeDetector::cloudCallback, &detector);

    ros::spin();
}
