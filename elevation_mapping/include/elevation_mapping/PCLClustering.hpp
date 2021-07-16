//
// Created by itaouil on 15/07/2021.
//

#pragma once

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

class PCLClustering {
public:
    /*!
     * Constructor.
     */
    explicit PCLClustering(ros::NodeHandle &nodeHandle, tf::TransformListener &listener);

    /*!
     * Destructor.
     */
    virtual ~PCLClustering();

    /*!
     * Performs ground plane detection and subtraction.
     */
    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
    //! ROS nodehandle.
    ros::NodeHandle nodeHandle_;

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Publisher pub_plane;
    ros::Publisher pub_filtered;
    ros::Publisher centroid_pub;

    tf::TransformListener &listener_;
};
