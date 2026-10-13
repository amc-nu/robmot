// ROS 2 mode: DetectedObjects in -> RobMOT pipeline -> TrackedObjects out.
// Built only when rclcpp + autoware_perception_msgs are found (see CMakeLists.txt).
#include <cmath>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include "datatype/data_structure.cpp"
#include "calibration_module/PointCloudCalibration.h"
#include "memory_module/MemoryManagement.h"
#include "association_module/ObjectAssociation.h"
#include "tracking_module/KF_Tracking.h"
#include "util.cpp"
#include "config/config.h"
#include "ros2_tracker.h"

using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrackedObject;

int runRos2(const std::string& inputTopic, const std::string& outputTopic)
{
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("robmot");
    auto pub  = node->create_publisher<TrackedObjects>(outputTopic, 10);

    float dt = 1. / 10.;
    float qv = parameters::selectedMotionModel->getVelQ();
    float qa = parameters::selectedMotionModel->getAccQ();
    Eigen::Matrix<double, 2, 2> D;
    D << selectedDetector->getXVar(), 0,
         0, selectedDetector->getYVar();

    DataFrame dataHolder;
    MemoryManagement memoryManagement(true);
    KF_Tracking KF_tracker(dt, qa, qv, D);
    PointCloudCalibration calib("", ""); // unused in this mode (KF only passes it through); no files are opened
    std::vector<int> confirmedObjects;   // only needed by updateObjectStatus; cleared every frame
    int streamId = 0;

    auto sub = node->create_subscription<DetectedObjects>(inputTopic, 10,
        [&](DetectedObjects::ConstSharedPtr msg)
    {
        // Detections are used as-is: the input frame must be a fixed world frame (e.g. map).
        for (const auto& d : msg->objects) {
            const auto& p = d.kinematics.pose_with_covariance.pose;
            const auto& q = p.orientation;
            const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

            float conf = d.existence_probability;
            int label  = 0;
            float best = -1;
            for (const auto& c : d.classification)
                if (c.probability > best) { best = c.probability; label = c.label; conf = c.probability; }

            Eigen::Vector3d loc(p.position.x, p.position.y, p.position.z);
            if (!Helper::acceptDetection(conf, loc, memoryManagement)) continue;

            auto object = new Object();
            object->_3dBoxLiDAR = Helper::get3DBoundingBox(d.shape.dimensions.z, d.shape.dimensions.y, d.shape.dimensions.x,
                                                           p.position.x, p.position.y, p.position.z, yaw);
            object->_3dBoxLiDAR->found = true;
            object->_3dBoxLiDAR->observationNumber += 1;
            object->rt = yaw;
            object->classId = label;
            object->confidenceScore = conf;
            dataHolder.objects.push_back(object);
        }

        std::map<int, int> matches;
        ObjectAssociation::runAssociation(dataHolder.objects, memoryManagement.getSavedObjects(), matches,
                                          { ObjectAssociation::AssociationType::LIDAR_3D_ASSOCIATION },
                                          { selectedDetector->getEclTh() }, false);
        memoryManagement.updateAssociatedObjects(matches, dataHolder.objects, streamId);
        KF_tracker.run({KF_Tracking::Type::KALMAN_FILTER_3D_LIDAR}, memoryManagement.getSavedObjects(), calib, streamId);

        TrackedObjects out;
        out.header = msg->header;
        for (auto object : memoryManagement.getSavedObjects()) {
            // Online ghost filter: only tracks already confirmed and detected in this frame.
            if (!object->isConfirmed || !object->_3dBoxLiDAR->found) continue;
            const auto& b = *object->_3dBoxLiDAR;
            const double yaw = b.dim(3);
            const double vx = b.stateEstimation(2), vy = b.stateEstimation(3); // world frame

            TrackedObject t;
            std::memcpy(t.object_id.uuid.data(), &object->trackId, sizeof(object->trackId)); // 8 of 16 bytes, rest zero
            t.existence_probability = object->confidenceScore;
            autoware_perception_msgs::msg::ObjectClassification cls;
            cls.label = object->classId;
            cls.probability = object->confidenceScore;
            t.classification.push_back(cls);
            auto& pose = t.kinematics.pose_with_covariance.pose;
            pose.position.x = b.estimatedLocation(0);
            pose.position.y = b.estimatedLocation(1);
            pose.position.z = b.estimatedLocation(2);
            pose.orientation.z = std::sin(yaw / 2);
            pose.orientation.w = std::cos(yaw / 2);
            t.kinematics.orientation_availability = autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
            // Autoware twist is in the object frame.
            t.kinematics.twist_with_covariance.twist.linear.x =  std::cos(yaw) * vx + std::sin(yaw) * vy;
            t.kinematics.twist_with_covariance.twist.linear.y = -std::sin(yaw) * vx + std::cos(yaw) * vy;
            t.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
            t.shape.dimensions.x = b.dim(0); // length
            t.shape.dimensions.y = b.dim(1); // width
            t.shape.dimensions.z = b.dim(2); // height
            out.objects.push_back(t);
        }
        pub->publish(out);

        dataHolder.reset();
        memoryManagement.updateObjectStatus(streamId, selectedDetector->getGhostRemTh(), confirmedObjects);
        confirmedObjects.clear();
        streamId++;
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
