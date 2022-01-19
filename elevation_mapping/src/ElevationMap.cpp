/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <cstring>

#include <Eigen/Dense>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

#include "elevation_mapping/Instrumentor.h"

#define PROFILING 1 // 0 if profiling should be disabled
#if PROFILING
#define PROFILE_SCOPE(name) InstrumentationTimer timer##__LINE__(name)
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)// grab function name
//#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCSIG__) // grab function signature
#else
#define PROFILE_SCOPE(name)
#define PROFILE_FUNCTION()
#endif

namespace {
    /**
 * Store an unsigned integer value in a float
 * @param input integer
 * @return A float with the bit pattern of the input integer
 */
    float intAsFloat(const uint32_t input) {
        float output;
        std::memcpy(&output, &input, sizeof(uint32_t));
        return output;
    }
}// namespace

namespace elevation_mapping {

    ElevationMap::ElevationMap(ros::NodeHandle nodeHandle)
            : nodeHandle_(nodeHandle),
              rawMap_({"elevation", "costmap", "processed_elevation", "variance", "distance",
                       "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "time",
                       "dynamic_time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan",
                       "sensor_z_at_lowest_scan"}),
              fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
              postprocessorPool_(nodeHandle.param("postprocessor_num_threads", 1), nodeHandle_),
              hasUnderlyingMap_(false),
              minVariance_(0.000009),
              maxVariance_(0.0009),
              mahalanobisDistanceThreshold_(2.5),
              multiHeightNoise_(0.000009),
              minHorizontalVariance_(0.0001),
              maxHorizontalVariance_(0.05),
              enableVisibilityCleanup_(true),
              enableContinuousCleanup_(false),
              visibilityCleanupDuration_(0.0),
              scanningDuration_(1.0) {
        rawMap_.setBasicLayers({"elevation", "variance", "costmap", "processed_elevation", "distance"});
        fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
        clear();

        elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
        if (!underlyingMapTopic_.empty()) {
            underlyingMapSubscriber_ = nodeHandle_.subscribe(underlyingMapTopic_, 1,
                                                             &ElevationMap::underlyingMapCallback, this);
        }
        // TODO(max): if (enableVisibilityCleanup_) when parameter cleanup is ready.
        visibilityCleanupMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("visibility_cleanup_map", 1);

        initialTime_ = ros::Time::now();
    }

    ElevationMap::~ElevationMap() = default;

    void ElevationMap::setGeometry(const grid_map::Length &length, const double &resolution,
                                   const grid_map::Position &position) {
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
        boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
        rawMap_.setGeometry(length, resolution, position);
        fusedMap_.setGeometry(length, resolution, position);
        ROS_INFO_STREAM("Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and " << rawMap_.getSize()(1)
                                                         << " columns.");
    }

    bool ElevationMap::add(const PointCloudType::Ptr pointCloud, Eigen::VectorXf &pointCloudVariances,
                           const ros::Time &timestamp,
                           const Eigen::Affine3d &transformationSensorToMap) {
//        if (static_cast<unsigned int>(pointCloud->size()) != static_cast<unsigned int>(pointCloudVariances.size())) {
//            ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.", (int) pointCloud->size(),
//                      (int) pointCloudVariances.size());
//            return false;
//        }

        // Initialization for time calculation.
        const ros::WallTime methodStartTime(ros::WallTime::now());
        const ros::Time currentTime(ros::Time::now());
        const float currentTimeSecondsPattern{
                intAsFloat(static_cast<uint32_t>(static_cast<uint64_t>(currentTime.toSec())))};
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

        // Update initial time if it is not initialized.
        if (initialTime_.toSec() == 0) {
            initialTime_ = timestamp;
        }
        const float scanTimeSinceInitialization = (timestamp - initialTime_).toSec();

        // Store references for efficient interaction.
        auto &elevationLayer = rawMap_["elevation"];
        auto &varianceLayer = rawMap_["variance"];
        auto &horizontalVarianceXLayer = rawMap_["horizontal_variance_x"];
        auto &horizontalVarianceYLayer = rawMap_["horizontal_variance_y"];
        auto &horizontalVarianceXYLayer = rawMap_["horizontal_variance_xy"];
        auto &colorLayer = rawMap_["color"];
        auto &timeLayer = rawMap_["time"];
        auto &dynamicTimeLayer = rawMap_["dynamic_time"];
        auto &lowestScanPointLayer = rawMap_["lowest_scan_point"];
        auto &sensorXatLowestScanLayer = rawMap_["sensor_x_at_lowest_scan"];
        auto &sensorYatLowestScanLayer = rawMap_["sensor_y_at_lowest_scan"];
        auto &sensorZatLowestScanLayer = rawMap_["sensor_z_at_lowest_scan"];

        // Push all basic map layers to vector
        std::vector<Eigen::Ref<const grid_map::Matrix>> basicLayers_;
        for (const std::string &layer: rawMap_.getBasicLayers()) {
            basicLayers_.push_back(rawMap_.get(layer));
        }

        // Iterate over every point in the pointcloud
        for (unsigned int i = 0; i < pointCloud->size(); ++i) {
            // Access 3D point
            auto &point = pointCloud->points[i];

            // Get corresponding 2D index
            // in the grid of the cloud point
            grid_map::Index index;
            grid_map::Position position(point.x, point.y);// NOLINT(cppcoreguidelines-pro-type-union-access)
            if (!rawMap_.getIndex(position, index)) {
                continue;// Skip this point if it does not lie within the elevation map.
            }

            // Extract point elevation
            auto &elevation = elevationLayer(index(0), index(1));

            // Can I possibly remove all this?
            auto &variance = varianceLayer(index(0), index(1));
            auto &horizontalVarianceX = horizontalVarianceXLayer(index(0), index(1));
            auto &horizontalVarianceY = horizontalVarianceYLayer(index(0), index(1));
            auto &horizontalVarianceXY = horizontalVarianceXYLayer(index(0), index(1));
            auto &color = colorLayer(index(0), index(1));
            auto &time = timeLayer(index(0), index(1));
            auto &dynamicTime = dynamicTimeLayer(index(0), index(1));
            auto &lowestScanPoint = lowestScanPointLayer(index(0), index(1));
            auto &sensorXatLowestScan = sensorXatLowestScanLayer(index(0), index(1));
            auto &sensorYatLowestScan = sensorYatLowestScanLayer(index(0), index(1));
            auto &sensorZatLowestScan = sensorZatLowestScanLayer(index(0), index(1));

//            const float &pointVariance = pointCloudVariances(i);
            const float &pointVariance = 0;
            bool isValid = std::all_of(basicLayers_.begin(), basicLayers_.end(),
                                       [&](Eigen::Ref<const grid_map::Matrix> layer) {
                                           return std::isfinite(layer(index(0), index(1)));
                                       });
            if (!isValid) {
                // No prior information in elevation map, use measurement.
                elevation = point.z;// NOLINT(cppcoreguidelines-pro-type-union-access)
                variance = pointVariance;
                horizontalVarianceX = minHorizontalVariance_;
                horizontalVarianceY = minHorizontalVariance_;
                horizontalVarianceXY = 0.0;
                grid_map::colorVectorToValue(point.getRGBVector3i(), color);
                continue;
            }

            // Deal with multiple heights in one cell.
            const double mahalanobisDistance =
                    fabs(point.z - elevation) / sqrt(variance);// NOLINT(cppcoreguidelines-pro-type-union-access)
            if (mahalanobisDistance > mahalanobisDistanceThreshold_) {
                if (scanTimeSinceInitialization - time <= scanningDuration_ &&
                    elevation > point.z) {// NOLINT(cppcoreguidelines-pro-type-union-access)
                    // Ignore point if measurement is from the same point cloud (time comparison) and
                    // if measurement is lower then the elevation in the map.
                } else if (scanTimeSinceInitialization - time <= scanningDuration_) {
                    // If point is higher.
                    elevation = 0.9 * elevation + 0.1 * point.z;// NOLINT(cppcoreguidelines-pro-type-union-access)
                    variance = pointVariance;
                } else {
                    variance += multiHeightNoise_;
                }
            }

            // Store lowest points from scan for visibility checking.
            const float pointHeightPlusUncertainty =
                    point.z + 3.0 * sqrt(pointVariance);// 3 sigma. // NOLINT(cppcoreguidelines-pro-type-union-access)
            if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint) {
                lowestScanPoint = pointHeightPlusUncertainty;
                const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
                sensorXatLowestScan = sensorTranslation.x();
                sensorYatLowestScan = sensorTranslation.y();
                sensorZatLowestScan = sensorTranslation.z();
            }

            // Fuse measurement with elevation map data.
//            elevation =
//                    (variance * point.z + pointVariance * elevation) / (variance + pointVariance);// NOLINT(cppcoreguidelines-pro-type-union-access)
//            variance = (pointVariance * variance) / (pointVariance + variance);
            // TODO(max): Add color fusion.
            grid_map::colorVectorToValue(point.getRGBVector3i(), color);
            time = scanTimeSinceInitialization;
            dynamicTime = currentTimeSecondsPattern;

            // Horizontal variances are reset.
            horizontalVarianceX = minHorizontalVariance_;
            horizontalVarianceY = minHorizontalVariance_;
            horizontalVarianceXY = 0.0;
        }

        clean();
        rawMap_.setTimestamp(timestamp.toNSec());// Point cloud stores time in microseconds.

        return true;
    }

    void ElevationMap::postProcessElevationMap() {
        // Keep track of which entries are NaNs
        std::vector<int> l_xIndices;
        std::vector<int> l_yIndices;
        for (int i = 0; i < rawMap_.getSize()(0); i++) {
            for (int j = 0; j < rawMap_.getSize()(1); j++) {
                if (std::isnan(rawMap_["elevation"](i, j))) {
                    l_xIndices.push_back(i);
                    l_yIndices.push_back(j);
                }
            }
        }

        // Convert elevation map to OpenCV
        cv::Mat l_elevationMapImage;
        if (!grid_map::GridMapCvConverter::toImage<float, 1>(rawMap_,
                                                             "elevation",
                                                             CV_32F,
                                                             rawMap_.get("elevation").minCoeffOfFinites(),
                                                             rawMap_.get("elevation").maxCoeffOfFinites(),
                                                             l_elevationMapImage)) {
            ROS_ERROR("ElevationMapProcessor: Could not convert grid_map to cv::Mat.");
        }

        // Dilate elevation map image to fill sparse regions
        cv::Mat l_elevationMapImageDilated;
        cv::dilate(l_elevationMapImage, l_elevationMapImageDilated, cv::Mat());

        // Erode elevation map to eliminate extraneous sensor returns
        cv::Mat l_elevationMapImageEroded;
        cv::erode(l_elevationMapImageDilated, l_elevationMapImageEroded, cv::Mat());

        // Apply median filter to smooth while preserving borders
        cv::Mat l_elevationMapImageFiltered;
        cv::medianBlur(l_elevationMapImageEroded, l_elevationMapImageFiltered, 3);

        // Change elevation layer to processed image
        grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(l_elevationMapImageFiltered,
                                                                  "processed_elevation",
                                                                  rawMap_,
                                                                  rawMap_.get("elevation").minCoeffOfFinites(),
                                                                  rawMap_.get("elevation").maxCoeffOfFinites());

        // Place back NaNs in processed elevation layer
        for (int i = 0; i < l_xIndices.size(); i++) {
            if (l_elevationMapImageFiltered.at<float>(l_xIndices[i], l_yIndices[i]) == 0.0) {
                rawMap_["processed_elevation"](l_xIndices[i], l_yIndices[i]) = NAN;
            }
        }
    }

    bool ElevationMap::update(const grid_map::Matrix &varianceUpdate, const grid_map::Matrix &horizontalVarianceUpdateX,
                              const grid_map::Matrix &horizontalVarianceUpdateY,
                              const grid_map::Matrix &horizontalVarianceUpdateXY,
                              const ros::Time &time) {
        boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

        const auto &size = rawMap_.getSize();

        if (!((grid_map::Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all() &&
              (grid_map::Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all() &&
              (grid_map::Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all() &&
              (grid_map::Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
            ROS_ERROR("The size of the update matrices does not match.");
            return false;
        }

        rawMap_.get("variance") += varianceUpdate;
        rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
        rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
        rawMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
        clean();
        rawMap_.setTimestamp(time.toNSec());

        return true;
    }

    bool ElevationMap::fuseAll() {
        ROS_DEBUG("Requested to fuse entire elevation map.");
        ROS_INFO("Requested to fuse entire elevation map.");
        boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
        return fuse(grid_map::Index(0, 0), fusedMap_.getSize());
    }

    bool ElevationMap::fuseArea(const Eigen::Vector2d &position, const Eigen::Array2d &length) {
        ROS_DEBUG("Requested to fuse an area of the elevation map with center at (%f, %f) and side lengths (%f, %f)",
                  position[0], position[1],
                  length[0], length[1]);

        grid_map::Index topLeftIndex;
        grid_map::Index submapBufferSize;

        // These parameters are not used in this function.
        grid_map::Position submapPosition;
        grid_map::Length submapLength;
        grid_map::Index requestedIndexInSubmap;

        boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

        grid_map::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength,
                                       requestedIndexInSubmap, position, length,
                                       rawMap_.getLength(), rawMap_.getPosition(), rawMap_.getResolution(),
                                       rawMap_.getSize(),
                                       rawMap_.getStartIndex());

        return fuse(topLeftIndex, submapBufferSize);
    }

    bool ElevationMap::fuse(const grid_map::Index &topLeftIndex, const grid_map::Index &size) {
        ROS_DEBUG("Fusing elevation map...");
        //  ROS_INFO("Fusing elevation map...");

        // Nothing to do.
        if ((size == 0).any()) {
            return false;
        }

        // Initializations.
        const ros::WallTime methodStartTime(ros::WallTime::now());

        // Copy raw elevation map data for safe multi-threading.
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
        auto rawMapCopy = rawMap_;
        scopedLockForRawData.unlock();

        boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

        // More initializations.
        const double halfResolution = fusedMap_.getResolution() / 2.0;
        const float minimalWeight = std::numeric_limits<float>::epsilon() * static_cast<float>(2.0);
        // Conservative cell inclusion for ellipse iterator.
        const double ellipseExtension = M_SQRT2 * fusedMap_.getResolution();

        // Check if there is the need to reset out-dated data.
        if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) {
            resetFusedData();
        }

        // Align fused map with raw map.
        if (rawMapCopy.getPosition() != fusedMap_.getPosition()) {
            fusedMap_.move(rawMapCopy.getPosition());
        }

        // For each cell in requested area.
        for (grid_map::SubmapIterator areaIterator(rawMapCopy, topLeftIndex,
                                                   size); !areaIterator.isPastEnd(); ++areaIterator) {
            // Check if fusion for this cell has already been done earlier.
            if (fusedMap_.isValid(*areaIterator)) {
                continue;
            }

            if (!rawMapCopy.isValid(*areaIterator)) {
                // This is an empty cell (hole in the map).
                // TODO(max):
                continue;
            }

            // Get size of error ellipse.
            const float &sigmaXsquare = rawMapCopy.at("horizontal_variance_x", *areaIterator);
            const float &sigmaYsquare = rawMapCopy.at("horizontal_variance_y", *areaIterator);
            const float &sigmaXYsquare = rawMapCopy.at("horizontal_variance_xy", *areaIterator);

            Eigen::Matrix2d covarianceMatrix;
            covarianceMatrix << sigmaXsquare, sigmaXYsquare, sigmaXYsquare, sigmaYsquare;
            // 95.45% confidence ellipse which is 2.486-sigma for 2 dof problem.
            // http://www.reid.ai/2012/09/chi-squared-distribution-table-with.html
            const double uncertaintyFactor = 2.486;// sqrt(6.18)
            Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
            Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());

            Eigen::Array2d::Index maxEigenvalueIndex;
            eigenvalues.maxCoeff(&maxEigenvalueIndex);
            Eigen::Array2d::Index minEigenvalueIndex;
            maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
            const grid_map::Length ellipseLength =
                    2.0 * uncertaintyFactor *
                    grid_map::Length(eigenvalues(maxEigenvalueIndex), eigenvalues(minEigenvalueIndex)).sqrt() +
                    ellipseExtension;
            const double ellipseRotation(
                    atan2(solver.eigenvectors().col(maxEigenvalueIndex).real()(1),
                          solver.eigenvectors().col(maxEigenvalueIndex).real()(0)));

            // Requested length and position (center) of submap in map.
            grid_map::Position requestedSubmapPosition;
            rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);
            grid_map::EllipseIterator ellipseIterator(rawMapCopy, requestedSubmapPosition, ellipseLength,
                                                      ellipseRotation);

            // Prepare data fusion.
            Eigen::ArrayXf means, weights;
            const unsigned int maxNumberOfCellsToFuse = ellipseIterator.getSubmapSize().prod();
            means.resize(maxNumberOfCellsToFuse);
            weights.resize(maxNumberOfCellsToFuse);
            WeightedEmpiricalCumulativeDistributionFunction<float> lowerBoundDistribution;
            WeightedEmpiricalCumulativeDistributionFunction<float> upperBoundDistribution;

            float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
            float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
            Eigen::Rotation2Dd rotationMatrix(ellipseRotation);
            std::string maxEigenvalueLayer, minEigenvalueLayer;
            if (maxEigenvalueIndex == 0) {
                maxEigenvalueLayer = "horizontal_variance_x";
                minEigenvalueLayer = "horizontal_variance_y";
            } else {
                maxEigenvalueLayer = "horizontal_variance_y";
                minEigenvalueLayer = "horizontal_variance_x";
            }

            // For each cell in error ellipse.
            size_t i = 0;
            for (; !ellipseIterator.isPastEnd(); ++ellipseIterator) {
                if (!rawMapCopy.isValid(*ellipseIterator)) {
                    // Empty cell in submap (cannot be center cell because we checked above).
                    continue;
                }

                means[i] = rawMapCopy.at("elevation", *ellipseIterator);

                // Compute weight from probability.
                grid_map::Position absolutePosition;
                rawMapCopy.getPosition(*ellipseIterator, absolutePosition);
                Eigen::Vector2d distanceToCenter = (rotationMatrix *
                                                    (absolutePosition - requestedSubmapPosition)).cwiseAbs();

                float probability1 = cumulativeDistributionFunction(distanceToCenter.x() + halfResolution, 0.0,
                                                                    maxStandardDeviation) -
                                     cumulativeDistributionFunction(distanceToCenter.x() - halfResolution, 0.0,
                                                                    maxStandardDeviation);
                float probability2 = cumulativeDistributionFunction(distanceToCenter.y() + halfResolution, 0.0,
                                                                    minStandardDeviation) -
                                     cumulativeDistributionFunction(distanceToCenter.y() - halfResolution, 0.0,
                                                                    minStandardDeviation);

                const float weight = std::max(minimalWeight, probability1 * probability2);
                weights[i] = weight;
                const float standardDeviation = sqrt(rawMapCopy.at("variance", *ellipseIterator));
                lowerBoundDistribution.add(means[i] - 2.0 * standardDeviation, weight);
                upperBoundDistribution.add(means[i] + 2.0 * standardDeviation, weight);

                i++;
            }

            if (i == 0) {
                // Nothing to fuse.
                fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
                fusedMap_.at("lower_bound", *areaIterator) =
                        rawMapCopy.at("elevation", *areaIterator) -
                        2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
                fusedMap_.at("upper_bound", *areaIterator) =
                        rawMapCopy.at("elevation", *areaIterator) +
                        2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
                fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
                continue;
            }

            // Fuse.
            means.conservativeResize(i);
            weights.conservativeResize(i);

            float mean = (weights * means).sum() / weights.sum();

            if (!std::isfinite(mean)) {
                ROS_ERROR("Something went wrong when fusing the map: Mean = %f", mean);
                continue;
            }

            // Add to fused map.
            fusedMap_.at("elevation", *areaIterator) = mean;
            lowerBoundDistribution.compute();
            upperBoundDistribution.compute();
            fusedMap_.at("lower_bound", *areaIterator) = lowerBoundDistribution.quantile(0.01);// TODO(max):
            fusedMap_.at("upper_bound", *areaIterator) = upperBoundDistribution.quantile(0.99);// TODO(max):
            // TODO(max): Add fusion of colors.
            fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
        }

        fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

        const ros::WallDuration duration(ros::WallTime::now() - methodStartTime);
        ROS_DEBUG("Elevation map has been fused in %f s.", duration.toSec());
        std::cout << "Fusion " << duration.toSec() << std::endl;

        return true;
    }

    bool ElevationMap::clear() {
        // Lock raw and fused map object in different scopes to prevent deadlock.
        {
            boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
            rawMap_.clearAll();
            rawMap_.resetTimestamp();
            rawMap_.get("dynamic_time").setZero();
        }

        return true;
    }

    void ElevationMap::visibilityCleanup(const ros::Time &updatedTime) {
        PROFILE_FUNCTION();

        // Get current time to compute calculation time.
        const ros::WallTime methodStartTime(ros::WallTime::now());
        const double timeSinceInitialization = (updatedTime - initialTime_).toSec();

        // Copy raw elevation map data for safe multi-threading.
        boost::recursive_mutex::scoped_lock scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
        visibilityCleanupMap_ = rawMap_;
        rawMap_.clear("lowest_scan_point");
        rawMap_.clear("sensor_x_at_lowest_scan");
        rawMap_.clear("sensor_y_at_lowest_scan");
        rawMap_.clear("sensor_z_at_lowest_scan");
        scopedLockForRawData.unlock();
        visibilityCleanupMap_.add("max_height");

        // Create max. height layer with ray tracing.
        for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
            if (!visibilityCleanupMap_.isValid(*iterator)) {
                continue;
            }
            const auto &lowestScanPoint = visibilityCleanupMap_.at("lowest_scan_point", *iterator);
            const auto &sensorXatLowestScan = visibilityCleanupMap_.at("sensor_x_at_lowest_scan", *iterator);
            const auto &sensorYatLowestScan = visibilityCleanupMap_.at("sensor_y_at_lowest_scan", *iterator);
            const auto &sensorZatLowestScan = visibilityCleanupMap_.at("sensor_z_at_lowest_scan", *iterator);
            if (std::isnan(lowestScanPoint)) {
                continue;
            }
            grid_map::Index indexAtSensor;
            if (!visibilityCleanupMap_.getIndex(grid_map::Position(sensorXatLowestScan, sensorYatLowestScan),
                                                indexAtSensor)) {
                continue;
            }
            grid_map::Position point;
            visibilityCleanupMap_.getPosition(*iterator, point);
            float pointDiffX = point.x() - sensorXatLowestScan;
            float pointDiffY = point.y() - sensorYatLowestScan;
            float distanceToPoint = sqrt(pointDiffX * pointDiffX + pointDiffY * pointDiffY);
            if (distanceToPoint > 0.0) {
                for (grid_map::LineIterator iterator(visibilityCleanupMap_, indexAtSensor,
                                                     *iterator); !iterator.isPastEnd(); ++iterator) {
                    grid_map::Position cellPosition;
                    visibilityCleanupMap_.getPosition(*iterator, cellPosition);
                    const float cellDiffX = cellPosition.x() - sensorXatLowestScan;
                    const float cellDiffY = cellPosition.y() - sensorYatLowestScan;
                    const float distanceToCell = distanceToPoint - sqrt(cellDiffX * cellDiffX + cellDiffY * cellDiffY);
                    const float maxHeightPoint = lowestScanPoint +
                                                 (sensorZatLowestScan - lowestScanPoint) / distanceToPoint *
                                                 distanceToCell;
                    auto &cellMaxHeight = visibilityCleanupMap_.at("max_height", *iterator);
                    if (std::isnan(cellMaxHeight) || cellMaxHeight > maxHeightPoint) {
                        cellMaxHeight = maxHeightPoint;
                    }
                }
            }
        }

        // Vector of indices that will be removed.
        std::vector<grid_map::Position> cellPositionsToRemove;
        for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
            if (!visibilityCleanupMap_.isValid(*iterator)) {
                continue;
            }
            const auto &time = visibilityCleanupMap_.at("time", *iterator);
            if (timeSinceInitialization - time > scanningDuration_) {
                // Only remove cells that have not been updated during the last scan duration.
                // This prevents a.o. removal of overhanging objects.
                const auto &elevation = visibilityCleanupMap_.at("elevation", *iterator);
                const auto &variance = visibilityCleanupMap_.at("variance", *iterator);
                const auto &maxHeight = visibilityCleanupMap_.at("max_height", *iterator);
                if (!std::isnan(maxHeight) && elevation - 3.0 * sqrt(variance) > maxHeight) {
                    grid_map::Position position;
                    visibilityCleanupMap_.getPosition(*iterator, position);
                    cellPositionsToRemove.push_back(position);
                }
            }
        }

        // Remove points in current raw map.
        scopedLockForRawData.lock();
        for (const auto &cellPosition: cellPositionsToRemove) {
            grid_map::Index index;
            if (!rawMap_.getIndex(cellPosition, index)) {
                continue;
            }
            if (rawMap_.isValid(index)) {
                rawMap_.at("elevation", index) = NAN;
                rawMap_.at("dynamic_time", index) = 0.0f;
            }
        }
        scopedLockForRawData.unlock();

        // Publish visibility cleanup map for debugging.
        publishVisibilityCleanupMap();

        ros::WallDuration duration(ros::WallTime::now() - methodStartTime);
        ROS_DEBUG("Visibility cleanup has been performed in %f s (%d points).", duration.toSec(),
                  (int) cellPositionsToRemove.size());
        if (duration.toSec() > visibilityCleanupDuration_) {
            ROS_WARN("Visibility cleanup duration is too high (current rate is %f).", 1.0 / duration.toSec());
        }
    }

    void ElevationMap::move(const Eigen::Vector2d &position) {
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
        std::vector<grid_map::BufferRegion> newRegions;

        if (rawMap_.move(position, newRegions)) {
            ROS_DEBUG("Elevation map has been moved to position (%f, %f).", rawMap_.getPosition().x(),
                      rawMap_.getPosition().y());

            // The "dynamic_time" layer is meant to be interpreted as integer values, therefore nan:s need to be zeroed.
            grid_map::Matrix &dynTime{rawMap_.get("dynamic_time")};
            dynTime = dynTime.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynTime.array());

            if (hasUnderlyingMap_) {
                rawMap_.addDataFrom(underlyingMap_, false, false, true);
            }
        }
    }

    bool ElevationMap::publishRawElevationMap() {
        if (!hasRawMapSubscribers()) {
            return false;
        }

        // Get current time to compute calculation time.
        const ros::WallTime methodStartTime(ros::WallTime::now());

        boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
        grid_map::GridMap rawMapCopy = rawMap_;
        scopedLock.unlock();

        // TODO (magnus) This postprocessing should be moved to the postprocessor pipeline.
        rawMapCopy.erase("lowest_scan_point");
        rawMapCopy.erase("sensor_x_at_lowest_scan");
        rawMapCopy.erase("sensor_y_at_lowest_scan");
        rawMapCopy.erase("sensor_z_at_lowest_scan");
        rawMapCopy.add("standard_deviation", rawMapCopy.get("variance").array().sqrt().matrix());
        rawMapCopy.add("horizontal_standard_deviation",
                       (rawMapCopy.get("horizontal_variance_x") +
                        rawMapCopy.get("horizontal_variance_y")).array().sqrt().matrix());
        rawMapCopy.add("two_sigma_bound",
                       rawMapCopy.get("elevation") + 2.0 * rawMapCopy.get("variance").array().sqrt().matrix());

        bool out = postprocessorPool_.runTask(rawMapCopy);
        ros::WallDuration duration(ros::WallTime::now() - methodStartTime);

        return out;
    }

    bool ElevationMap::publishFusedElevationMap() {
        if (!hasFusedMapSubscribers()) {
            return false;
        }
        // Get current time to compute calculation time.
        const ros::WallTime methodStartTime(ros::WallTime::now());

        boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
        grid_map::GridMap fusedMapCopy = fusedMap_;
        scopedLock.unlock();
        fusedMapCopy.add("uncertainty_range", fusedMapCopy.get("upper_bound") - fusedMapCopy.get("lower_bound"));
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(fusedMapCopy, message);
        elevationMapFusedPublisher_.publish(message);
        ROS_DEBUG("Elevation map (fused) has been published.");

        ros::WallDuration duration(ros::WallTime::now() - methodStartTime);
        std::cout << "publishFusedElevationMap " << duration.toSec() << std::endl;

        return true;
    }

    bool ElevationMap::publishVisibilityCleanupMap() {
        if (visibilityCleanupMapPublisher_.getNumSubscribers() < 1) {
            return false;
        }
        boost::recursive_mutex::scoped_lock scopedLock(visibilityCleanupMapMutex_);
        grid_map::GridMap visibilityCleanupMapCopy = visibilityCleanupMap_;
        scopedLock.unlock();
        visibilityCleanupMapCopy.erase("elevation");
        visibilityCleanupMapCopy.erase("variance");
        visibilityCleanupMapCopy.erase("horizontal_variance_x");
        visibilityCleanupMapCopy.erase("horizontal_variance_y");
        visibilityCleanupMapCopy.erase("horizontal_variance_xy");
        visibilityCleanupMapCopy.erase("color");
        visibilityCleanupMapCopy.erase("time");
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(visibilityCleanupMapCopy, message);
        visibilityCleanupMapPublisher_.publish(message);
        ROS_DEBUG("Visibility cleanup map has been published.");
        return true;
    }

    grid_map::GridMap &ElevationMap::getRawGridMap() {
        return rawMap_;
    }

    void ElevationMap::setRawGridMap(const grid_map::GridMap &map) {
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
        rawMap_ = map;
    }

    grid_map::GridMap &ElevationMap::getFusedGridMap() {
        return fusedMap_;
    }

    void ElevationMap::setFusedGridMap(const grid_map::GridMap &map) {
        boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
        fusedMap_ = map;
    }

    ros::Time ElevationMap::getTimeOfLastUpdate() {
        return ros::Time().fromNSec(rawMap_.getTimestamp());
    }

    ros::Time ElevationMap::getTimeOfLastFusion() {
        boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
        return ros::Time().fromNSec(fusedMap_.getTimestamp());
    }

    const kindr::HomTransformQuatD &ElevationMap::getPose() {
        return pose_;
    }

    bool ElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i &index, kindr::Position3D &position) {
        kindr::Position3D positionInGridFrame;
        if (!rawMap_.getPosition3("elevation", index, positionInGridFrame.vector())) {
            return false;
        }
        position = pose_.transform(positionInGridFrame);
        return true;
    }

    boost::recursive_mutex &ElevationMap::getFusedDataMutex() {
        return fusedMapMutex_;
    }

    boost::recursive_mutex &ElevationMap::getRawDataMutex() {
        return rawMapMutex_;
    }

    bool ElevationMap::clean() {
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
        rawMap_.get("variance") = rawMap_.get("variance").unaryExpr(
                VarianceClampOperator<float>(minVariance_, maxVariance_));
        rawMap_.get("horizontal_variance_x") =
                rawMap_.get("horizontal_variance_x").unaryExpr(
                        VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
        rawMap_.get("horizontal_variance_y") =
                rawMap_.get("horizontal_variance_y").unaryExpr(
                        VarianceClampOperator<float>(minHorizontalVariance_, maxHorizontalVariance_));
        return true;
    }

    void ElevationMap::resetFusedData() {
        boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
        fusedMap_.clearAll();
        fusedMap_.resetTimestamp();
    }

    void ElevationMap::setFrameId(const std::string &frameId) {
        rawMap_.setFrameId(frameId);
        fusedMap_.setFrameId(frameId);
    }

    void ElevationMap::setTimestamp(ros::Time timestamp) {
        rawMap_.setTimestamp(timestamp.toNSec());
        fusedMap_.setTimestamp(timestamp.toNSec());
    }

    const std::string &ElevationMap::getFrameId() {
        return rawMap_.getFrameId();
    }

    bool ElevationMap::hasRawMapSubscribers() const {
        return postprocessorPool_.pipelineHasSubscribers();
    }

    bool ElevationMap::hasFusedMapSubscribers() const {
        return elevationMapFusedPublisher_.getNumSubscribers() >= 1;
    }

    void ElevationMap::underlyingMapCallback(const grid_map_msgs::GridMap &underlyingMap) {
        ROS_INFO("Updating underlying map.");
        grid_map::GridMapRosConverter::fromMessage(underlyingMap, underlyingMap_);
        if (underlyingMap_.getFrameId() != rawMap_.getFrameId()) {
            ROS_ERROR_STREAM("The underlying map does not have the same map frame ('" << underlyingMap_.getFrameId()
                                                                                      << "') as the elevation map ('"
                                                                                      << rawMap_.getFrameId() << "').");
            return;
        }
        if (!underlyingMap_.exists("elevation")) {
            ROS_ERROR_STREAM("The underlying map does not have an 'elevation' layer.");
            return;
        }
        if (!underlyingMap_.exists("variance")) {
            underlyingMap_.add("variance", minVariance_);
        }
        if (!underlyingMap_.exists("horizontal_variance_x")) {
            underlyingMap_.add("horizontal_variance_x", minHorizontalVariance_);
        }
        if (!underlyingMap_.exists("horizontal_variance_y")) {
            underlyingMap_.add("horizontal_variance_y", minHorizontalVariance_);
        }
        if (!underlyingMap_.exists("color")) {
            underlyingMap_.add("color", 0.0);
        }
        underlyingMap_.setBasicLayers(rawMap_.getBasicLayers());
        hasUnderlyingMap_ = true;
        rawMap_.addDataFrom(underlyingMap_, false, false, true);
    }

    void
    ElevationMap::setRawSubmapHeight(const grid_map::Position &initPosition, float mapHeight, double lengthInXSubmap,
                                     double lengthInYSubmap, double margin) {
        // Set a submap area (lengthInYSubmap + margin, lengthInXSubmap + margin) with a constant height (mapHeight).
        boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

        // Calculate submap iterator start index.
        const grid_map::Position topLeftPosition(initPosition(0) + lengthInXSubmap / 2,
                                                 initPosition(1) + lengthInYSubmap / 2);
        grid_map::Index submapTopLeftIndex;
        rawMap_.getIndex(topLeftPosition, submapTopLeftIndex);

        // Calculate submap area.
        const double resolution = rawMap_.getResolution();
        const int lengthInXSubmapI = static_cast<int>(lengthInXSubmap / resolution + 2 * margin);
        const int lengthInYSubmapI = static_cast<int>(lengthInYSubmap / resolution + 2 * margin);
        const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);

        // Iterate through submap and fill height values.
        grid_map::Matrix &elevationData = rawMap_["elevation"];
        grid_map::Matrix &varianceData = rawMap_["variance"];
        for (grid_map::SubmapIterator iterator(rawMap_, submapTopLeftIndex,
                                               submapBufferSize); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index index(*iterator);
            elevationData(index(0), index(1)) = mapHeight;
            varianceData(index(0), index(1)) = 0.0;
        }
    }

    float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
        return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
    }

}// namespace elevation_mapping
