#include "elevation_mapping/PCLClustering.hpp"

/**
 * Default constructor.
 */
PCLClustering::PCLClustering(ros::NodeHandle &nodeHandle, tf::TransformListener &listener)
        : nodeHandle_(nodeHandle), listener_(listener) {
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nodeHandle_.subscribe("/camera/depth/points", 1, &PCLClustering::cloud_cb, this);

    // Create a ROS publisher for the output point cloud
    pub = nodeHandle_.advertise<pcl::PCLPointCloud2>("obstacles", 1);
    pub_plane = nodeHandle_.advertise<pcl::PCLPointCloud2>("plane", 1);
    pub_filtered = nodeHandle_.advertise<pcl::PCLPointCloud2>("filtered", 1);
    centroid_pub = nodeHandle_.advertise<geometry_msgs::PoseArray>("centroids", 1);
}

/**
 * Destructor.
 */
PCLClustering::~PCLClustering() {}

/*
 * PCL clustering.
 */
void PCLClustering::cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    //ROS_INFO("Received pointcloud from frame-ID %s. timestamp: %f s.", msg->header.frame_id.c_str(), msg->header.stamp.toSec());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_temp);

    // Create the filtering object
    pcl::PassThrough <pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_temp);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0f,
                         2.0f); // Filter by height 1 - 3 m //TODO: this needs to be adjusted for ground plane !
    pass.filter(*cloud);

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

    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    cloud_filtered_msg.header = msg->header;
    pub_filtered.publish(cloud_filtered_msg);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);

    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction <pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01); // 1cm // TODO: adapt parameters
    ec.setMinClusterSize(400); // TODO: adapt parameters
    ec.setMaxClusterSize(25000); // TODO: adapt parameters
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    geometry_msgs::PoseArray poses_msg;
    poses_msg.header = msg->header;
    //poses_msg.header.frame_id = "base";
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        pcl::CentroidPoint <pcl::PointXYZRGB> centroid;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            centroid.add(cloud_filtered->points[*pit]);
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ cc;
        centroid.get(cc);

        //TODO: Covariance and orientation might not be necessary...
        //TODO: Determine height (and width) of cluster, i.e. min/max x, min/max y, min/max z

        Eigen::Vector4f centroid_eigen(cc.x, cc.y, cc.z, 1.0);
        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix(*cloud_cluster, centroid_eigen, covariance_matrix);

        // Extract the eigenvalues and eigenvectors
        Eigen::Vector3f eigen_values;
        Eigen::Matrix3f eigen_vectors;
        pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);

        Eigen::Vector3f principal_axis = eigen_vectors.col(
                2); // eigenvalues are sorted in ascending order -> last ev is the principal axis.
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

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = msg->header;
        poseStamped.pose.position.x = cc.x;
        poseStamped.pose.position.y = cc.y;
        poseStamped.pose.position.z = cc.z;
        poseStamped.pose.orientation.x = qq.x(); //cluster_rot.x();
        poseStamped.pose.orientation.y = qq.y(); //cluster_rot.y();
        poseStamped.pose.orientation.z = qq.z(); //cluster_rot.z();
        poseStamped.pose.orientation.w = qq.w(); //cluster_rot.w();

        // Transform centroid pose into robot-frame, (with z-Axis pointing upward, normal to ground-plane)
        geometry_msgs::PoseStamped poseStamped_transformed;
        listener_.transformPose("/base_footprint", ros::Time(0), poseStamped, "camera_depth_optical_frame",
                               poseStamped_transformed);

        poses_msg.poses.push_back(poseStamped_transformed.pose);

        ROS_INFO("Cluster %d: centroid: [%f, %f, %f]; %zu data points.\n", j++, cc.x, cc.y, cc.z,
                 centroid.getSize());
    }
    ROS_INFO("\n");

    centroid_pub.publish(poses_msg);
}
