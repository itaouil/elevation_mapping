////
//// Created by itaouil on 15/07/2021.
////
//

/**
 * Makes all the non-planar things green, publishes the final result on the /obstacles topic.
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
ros::Publisher pub_plane;
ros::Publisher pub_filtered;
ros::Publisher centroid_pub;

tf::TransformListener* listener;

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    //ROS_INFO("Received pointcloud from frame-ID %s. timestamp: %f s.", msg->header.frame_id.c_str(), msg->header.stamp.toSec());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_temp);

    // Create the filtering object
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_temp);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.setFilterFieldName("z");
    sor.setFilterLimits(0.0f, 2.0f); // Filter by height 1 - 3 m //TODO: this needs to be adjusted for ground plane !
    sor.filter(*cloud);

//    pcl::PassThrough <pcl::PointXYZRGB> pass;
//    pass.setInputCloud(cloud_temp);
//    pass.setFilterFieldName("z");
//    pass.setFilterLimits(0.0f, 2.0f); // Filter by height 1 - 3 m //TODO: this needs to be adjusted for ground plane !
//    pass.filter(*cloud);

    static float coeff[4] = {0, 0, 0, 0};
    static int idx = 0;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation <pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
//    seg.setAxis(plane_normal); // Assumed normal of ground-plane
//    seg.setEpsAngle(5.0 * M_PI / 180.0);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return;
    }

    coeff[0] = (coeff[0] * idx + coefficients->values[0]) / (float) (idx + 1);
    coeff[1] = (coeff[1] * idx + coefficients->values[1]) / (float) (idx + 1);
    coeff[2] = (coeff[2] * idx + coefficients->values[2]) / (float) (idx + 1);
    coeff[3] = (coeff[3] * idx + coefficients->values[3]) / (float) (idx + 1);
    cout << "Moving-average coefficients: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << " " << coeff[3]
         << endl;
    ++idx;
    coefficients->values[0] = coeff[0];
    coefficients->values[1] = coeff[1];
    coefficients->values[2] = coeff[2];
    coefficients->values[3] = coeff[3];

//   cout << "Model coefficients: " << coefficients->values[0] << " "
//                                         << coefficients->values[1] << " "
//                                         << coefficients->values[2] << " "
//                                         << coefficients->values[3] << std::endl;

    cout << "Model inliers: " << inliers->indices.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud <pcl::PointXYZRGB>);
    for (size_t i = 0; i < inliers->indices.size(); ++i) {
        cloud_plane->push_back(cloud->points[inliers->indices[i]]);
        cloud_plane->points.back().r = 165;
        cloud_plane->points.back().g = 165;
        cloud_plane->points.back().b = 100;
    }

    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector3f eigen_values;
    Eigen::Matrix3f eigen_vectors;
    pcl::compute3DCentroid(*cloud_plane, centroid);
    pcl::computeCovarianceMatrix(*cloud_plane, centroid, covariance_matrix); // Compute the 3x3 covariance matrix
    pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values); // Extract the eigenvalues and eigenvectors

    // Publish planar only point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_plane, cloud_msg);
    cloud_msg.header = msg->header;
    pub_plane.publish(cloud_msg);

    // Subtract inliers from input cloud
    pcl::ExtractIndices <pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZRGB>);
    extract.filter(*cloud_filtered);

    // Color non plane objects green
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
        cloud_filtered->points[i].r = 0;
        cloud_filtered->points[i].g = 255;
        cloud_filtered->points[i].b = 0;
    }

    // Publisher non-planar point cloud
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    cloud_filtered_msg.header = msg->header;
    pub_filtered.publish(cloud_filtered_msg);

    // Convert point cloud to robot frame
    sensor_msgs::PointCloud2 cloud_filtered_msg_robot_frame;
    pcl_ros::transformPointCloud("/base_footprint", cloud_filtered_msg, cloud_filtered_msg_robot_frame, *listener);

    // Replace previous filtered point cloud
    // with new one in robot frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_robot_frame(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_filtered_msg_robot_frame, *cloud_filtered_robot_frame);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered_robot_frame);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.01); // 1cm // TODO: adapt parameters
    ec.setMinClusterSize (100); // TODO: adapt parameters
    ec.setMaxClusterSize (2500); // TODO: adapt parameters
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered_robot_frame);
    ec.extract (cluster_indices);

    std::cout << "Number of points for centroid comp. : " << cloud_filtered_robot_frame->points.size() << std::endl;

    int j = 0;
    geometry_msgs::PoseArray poses_msg;
    poses_msg.header = msg->header;
    poses_msg.header.frame_id = "base_footprint";
    //poses_msg.header.frame_id = "base";
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            centroid.add (cloud_filtered_robot_frame->points[*pit]);
            cloud_cluster->points.push_back(cloud_filtered_robot_frame->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ cc;
        centroid.get(cc);

        //TODO: Covariance and orientation might not be necessary...
        //TODO: Determine height (and width) of cluster, i.e. min/max x, min/max y, min/max z

        Eigen::Vector4f centroid_eigen(cc.x, cc.y, cc.z, 1.0);
        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix (*cloud_cluster, centroid_eigen, covariance_matrix);

        // Extract the eigenvalues and eigenvectors
        Eigen::Vector3f eigen_values;
        Eigen::Matrix3f eigen_vectors;
        pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

        Eigen::Vector3f principal_axis = eigen_vectors.col(2); // eigenvalues are sorted in ascending order -> last ev is the principal axis.
        Eigen::Vector3f normal(0, 0, -1.0); // z
        float dist = principal_axis.dot(normal);
        Eigen::Vector3f principal_axis_proj = principal_axis - dist * normal; // x
        //if (principal_axis_proj.x() > 0){ // ensure that principal axis is always pointing towards the robot base (for objects in front of the robot..) (-> for correct gripper orientation..)
        //  principal_axis_proj = -principal_axis_proj;
        //  ROS_WARN("Switched sign of principal axis.");
        //}
        principal_axis_proj.normalize();
        Eigen::Vector3f y_axis = normal.cross(principal_axis_proj);
        y_axis.normalize();

        Eigen::Matrix3f object_orientation;
        object_orientation.col(0) = principal_axis_proj;
        object_orientation.col(1) = y_axis;
        object_orientation.col(2) = normal;

        Eigen::Quaternionf qq(object_orientation);
        qq.normalize();

        geometry_msgs::Pose pose;
        pose.position.x = cc.x;
        pose.position.y = cc.y;
        pose.position.z = cc.z;
        pose.orientation.x = qq.x(); //cluster_rot.x();
        pose.orientation.y = qq.y(); //cluster_rot.y();
        pose.orientation.z = qq.z(); //cluster_rot.z();
        pose.orientation.w = qq.w(); //cluster_rot.w();
        poses_msg.poses.push_back(pose);

        ROS_INFO("Cluster %d: centroid: [%f, %f, %f]; %zu data points.\n", j++, cc.x, cc.y, cc.z, centroid.getSize());
    }
    ROS_INFO("\n");

    centroid_pub.publish(poses_msg);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "obstacles");
    ros::NodeHandle nh;

    // Initialize listener
    tf::TransformListener lr(ros::Duration(10));
    listener=&lr;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2>("obstacles", 1);
    pub_plane = nh.advertise<pcl::PCLPointCloud2>("plane", 1);
    pub_filtered = nh.advertise<pcl::PCLPointCloud2>("filtered", 1);
    centroid_pub = nh.advertise<geometry_msgs::PoseArray>("centroids", 1);

    // Spin
    ros::spin();

    return 0;
}