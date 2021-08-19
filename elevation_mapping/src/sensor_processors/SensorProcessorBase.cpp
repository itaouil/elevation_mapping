/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

// PCL
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>

// TF
#include <tf_conversions/tf_eigen.h>

// STL
#include <cmath>
#include <limits>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, const GeneralParameters& generalConfig)
    : nodeHandle_(nodeHandle),
      ignorePointsUpperThreshold_(std::numeric_limits<double>::infinity()),
      ignorePointsLowerThreshold_(-std::numeric_limits<double>::infinity()),
      applyVoxelGridFilter_(false),
      firstTfAvailable_(false) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  transformationSensorToMap_.setIdentity();
  generalParameters_ = generalConfig;
  ROS_DEBUG(
      "Sensor processor general parameters are:"
      "\n\t- robot_base_frame_id: %s"
      "\n\t- map_frame_id: %s",
      generalConfig.robotBaseFrameId_.c_str(), generalConfig.mapFrameId_.c_str());
}

SensorProcessorBase::~SensorProcessorBase() = default;

bool SensorProcessorBase::readParameters() {
  nodeHandle_.param("sensor_processor/ignore_points_above", ignorePointsUpperThreshold_, std::numeric_limits<double>::infinity());
  nodeHandle_.param("sensor_processor/ignore_points_below", ignorePointsLowerThreshold_, -std::numeric_limits<double>::infinity());

  nodeHandle_.param("sensor_processor/apply_voxelgrid_filter", applyVoxelGridFilter_, false);
  nodeHandle_.param("sensor_processor/voxelgrid_filter_size", sensorParameters_["voxelgrid_filter_size"], 0.0);
  return true;
}

bool SensorProcessorBase::process(const PointCloudType::ConstPtr pointCloudInput, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                  const PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame) {
  sensorFrameId_ = sensorFrame;
  ROS_DEBUG("Sensor Processor processing for frame %s", sensorFrameId_.c_str());

    const ros::WallTime methodStartTime1(ros::WallTime::now());
  // Update transformation at timestamp of pointcloud
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
  if (!updateTransformations(timeStamp)) {
    return false;
  }
    ros::WallDuration duration1(ros::WallTime::now() - methodStartTime1);
    //std::cout << "Time to update transformation: " << duration1.toSec() << std::endl;

  // Transform into sensor frame.
    const ros::WallTime methodStartTime2(ros::WallTime::now());
  PointCloudType::Ptr pointCloudSensorFrame(new PointCloudType);
  transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);
    ros::WallDuration duration2(ros::WallTime::now() - methodStartTime2);
    //std::cout << "Time to transform into sensor frame: " << duration2.toSec() << std::endl;

  // Remove Nans (optional voxel grid filter)
    const ros::WallTime methodStartTime3(ros::WallTime::now());
  filterPointCloud(pointCloudSensorFrame);
    ros::WallDuration duration3(ros::WallTime::now() - methodStartTime3);
    //std::cout << "Time to remove NANS: " << duration3.toSec() << std::endl;

  // Specific filtering per sensor type
    const ros::WallTime methodStartTime4(ros::WallTime::now());
  filterPointCloudSensorType(pointCloudSensorFrame);
    ros::WallDuration duration4(ros::WallTime::now() - methodStartTime4);
    //std::cout << "Time to perform specific filtering: " << duration4.toSec() << std::endl;

  // Remove outside limits in map frame
    const ros::WallTime methodStartTime5(ros::WallTime::now());
  if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, generalParameters_.mapFrameId_)) {
    return false;
  }
    ros::WallDuration duration5(ros::WallTime::now() - methodStartTime5);
    //std::cout << "Time to transform point cloud: " << duration5.toSec() << std::endl;

    const ros::WallTime methodStartTime6(ros::WallTime::now());
  std::vector<PointCloudType::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);
    ros::WallDuration duration6(ros::WallTime::now() - methodStartTime6);
    //std::cout << "Time to remove outside points: " << duration6.toSec() << std::endl;

  // Compute variances
  return computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances);
}

bool SensorProcessorBase::updateTransformations(const ros::Time& timeStamp) {
  try {
    transformListener_.waitForTransform(sensorFrameId_, generalParameters_.mapFrameId_, timeStamp, ros::Duration(1.0));

    tf::StampedTransform transformTf;
    transformListener_.lookupTransform(generalParameters_.mapFrameId_, sensorFrameId_, timeStamp, transformTf);
    poseTFToEigen(transformTf, transformationSensorToMap_);

    transformListener_.lookupTransform(generalParameters_.robotBaseFrameId_, sensorFrameId_, timeStamp,
                                       transformTf);  // TODO(max): Why wrong direction?
    Eigen::Affine3d transform;
    poseTFToEigen(transformTf, transform);
    rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

    transformListener_.lookupTransform(generalParameters_.mapFrameId_, generalParameters_.robotBaseFrameId_, timeStamp,
                                       transformTf);  // TODO(max): Why wrong direction?
    poseTFToEigen(transformTf, transform);
    rotationMapToBase_.setMatrix(transform.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

    if (!firstTfAvailable_) {
      firstTfAvailable_ = true;
    }

    return true;
  } catch (tf::TransformException& ex) {
    if (!firstTfAvailable_) {
      return false;
    }
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SensorProcessorBase::transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed,
                                              const std::string& targetFrame) {
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);
  const std::string inputFrameId(pointCloud->header.frame_id);

  tf::StampedTransform transformTf;
  try {
    transformListener_.waitForTransform(targetFrame, inputFrameId, timeStamp, ros::Duration(1.0), ros::Duration(0.001));
    transformListener_.lookupTransform(targetFrame, inputFrameId, timeStamp, transformTf);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d transform;
  poseTFToEigen(transformTf, transform);
  pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
  pointCloudTransformed->header.frame_id = targetFrame;

  ROS_DEBUG_THROTTLE(5, "Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
                     pointCloudTransformed->header.stamp / 1000.0);
  return true;
}

void SensorProcessorBase::removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds) {
  if (!std::isfinite(ignorePointsLowerThreshold_) && !std::isfinite(ignorePointsUpperThreshold_)) {
    return;
  }
  ROS_DEBUG("Limiting point cloud to the height interval of [%f, %f] relative to the robot base.", ignorePointsLowerThreshold_,
            ignorePointsUpperThreshold_);

  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z");  // TODO(max): Should this be configurable?
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    PointCloudType tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  ROS_DEBUG("removePointsOutsideLimits() reduced point cloud to %i points.", (int)pointClouds[0]->size());
}

bool SensorProcessorBase::filterPointCloud(const PointCloudType::Ptr pointCloud) {
  PointCloudType tempPointCloud;

    const ros::WallTime methodStartTime1(ros::WallTime::now());
  // Remove nan points.
  std::vector<int> indices;
  if (!pointCloud->is_dense) {
    pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
    tempPointCloud.is_dense = true;
    pointCloud->swap(tempPointCloud);
  }
    ros::WallDuration duration1(ros::WallTime::now() - methodStartTime1);
    //std::cout << "Time to remove nan points: " << duration1.toSec() << std::endl;

    const ros::WallTime methodStartTime2(ros::WallTime::now());
  // Reduce points using VoxelGrid filter.
  if (applyVoxelGridFilter_) {
      //std::cout << "Filtering...." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio> voxelGridFilter;
    voxelGridFilter.setInputCloud(pointCloud);
    double filter_size = sensorParameters_.at("voxelgrid_filter_size");
    voxelGridFilter.setLeafSize(filter_size, filter_size, filter_size);
    voxelGridFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
    ros::WallDuration duration2(ros::WallTime::now() - methodStartTime2);
    //std::cout << "Time to filter point cloud with voxel: " << duration2.toSec() << std::endl;

  ROS_DEBUG_THROTTLE(2, "cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}

bool SensorProcessorBase::filterPointCloudSensorType(const PointCloudType::Ptr /*pointCloud*/) {
  return true;
}

} /* namespace elevation_mapping */
