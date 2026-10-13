#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include "datatype/data_structure.cpp"
#include "calibration_module/PointCloudCalibration.h"
#include "memory_module/MemoryManagement.h"
#include "association_module/ObjectAssociation.h"
#include "tracking_module/KF_Tracking.h"
#include "util.cpp"
#include "config/config.h"
#include <filesystem>
#include <eigen3/Eigen/Core>
#include "args.hxx"
#ifdef ROBMOT_ROS2
#include "ros2_tracker.h"
#endif
using namespace std;
namespace fs = std::filesystem;


std::vector<std::string> extractTimestamps(const std::string& filePath) {
    std::vector<std::string> timestamps;
    std::ifstream file(filePath);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filePath << std::endl;
        return timestamps;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string timestamp;
        if (ss >> timestamp) {
            timestamps.push_back(timestamp);
        }
    }

    file.close();
    return timestamps;
}


int main(int argc, char** argv) {
    args::ArgumentParser parser("RobMOT online 3D multi-object tracker. File mode needs --dataset-name, --split, --detector, "
                                "--base-path, --dataset-dir; --ros2 overrides them and needs --input-topic and --output-topic.");
    args::HelpFlag help(parser, "help", "Show this help", {'h', "help"});
    args::Flag ros2Arg(parser, "ros2", "ROS 2 mode: DetectedObjects in, TrackedObjects out (ignores the dataset options; input must be in a fixed world frame)", {"ros2"});
    args::ValueFlag<string> inputTopicArg(parser, "topic", "[--ros2] autoware_perception_msgs/msg/DetectedObjects topic to subscribe to", {"input-topic"});
    args::ValueFlag<string> outputTopicArg(parser, "topic", "[--ros2] autoware_perception_msgs/msg/TrackedObjects topic to publish", {"output-topic"});
    args::ValueFlag<string> datasetNameArg(parser, "kitti|waymo", "Dataset name", {"dataset-name"});
    args::ValueFlag<string> splitArg(parser, "training|testing", "Dataset split", {"split"});
    args::ValueFlag<string> detectorArg(parser, "name", "Detector: Virconv, Casc, Pointrcnn, Pvcnn, SecondIou (KITTI); CascWaymo, Ctrl (Waymo). Default with --ros2: Ctrl", {"detector"});
    args::ValueFlag<string> basePathArg(parser, "dir", "Path to RobMOT/src (holds advanced_detection/)", {"base-path"});
    args::ValueFlag<string> datasetDirArg(parser, "dir", "Dataset root (contains kitti/ and waymo/)", {"dataset-dir"});
    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&) {
        std::cout << parser;
        return 0;
    } catch (const args::Error& e) {
        std::cerr << e.what() << "\n" << parser;
        return 1;
    }

    const bool ros2 = ros2Arg;
    std::vector<std::pair<const char*, bool>> required;
    if (ros2) required = {{"--input-topic", (bool)inputTopicArg}, {"--output-topic", (bool)outputTopicArg}};
    else      required = {{"--dataset-name", (bool)datasetNameArg}, {"--split", (bool)splitArg}, {"--detector", (bool)detectorArg},
                          {"--base-path", (bool)basePathArg}, {"--dataset-dir", (bool)datasetDirArg}};
    for (auto& [name, present] : required)
        if (!present) { std::cerr << name << " is required" << (ros2 ? " with --ros2" : "") << "\n" << parser; return 1; }

    if (ros2) {
#ifdef ROBMOT_ROS2
        parameters::selectedDetector = parameters::makeDetector(detectorArg ? args::get(detectorArg) : "Ctrl");
        if (!parameters::selectedDetector) {
            std::cerr << "Unknown --detector '" << args::get(detectorArg) << "'\n"; return 1;
        }
        return runRos2(args::get(inputTopicArg), args::get(outputTopicArg));
#else
        std::cerr << "Built without ROS 2 support (rclcpp/autoware_perception_msgs not found at configure time)\n"; return 1;
#endif
    }

    Settings::dataset_name = args::get(datasetNameArg);
    Settings::dataset      = args::get(splitArg);
    Settings::basePath     = args::get(basePathArg);
    if (Settings::dataset_name != "kitti" && Settings::dataset_name != "waymo") {
        std::cerr << "--dataset-name must be kitti or waymo\n"; return 1;
    }
    if (Settings::dataset != "training" && Settings::dataset != "testing") {
        std::cerr << "--split must be training or testing\n"; return 1;
    }
    parameters::selectedDetector = parameters::makeDetector(args::get(detectorArg));
    if (!parameters::selectedDetector) {
        std::cerr << "Unknown --detector '" << args::get(detectorArg) << "'\n"; return 1;
    }

    float sum = 0;
    float totalFrames = 0;
    string BASE_DIR = args::get(datasetDirArg);
    if (BASE_DIR.back() != '/') BASE_DIR += '/';
    const string BASE_DIR_DATABASE      = BASE_DIR + Settings::dataset_name;

    // Camera Images Settings

    const string IMAGE_DATASET_PATH     = BASE_DIR_DATABASE + "/Camera/"+ Settings::dataset +"/";

    // LiDAR PointCloud Settings
    const string POINTCLOUD_DATASET_PATH    = BASE_DIR_DATABASE +  "/LiDAR/"+ Settings::dataset +"/velodyne/";
    const string Calib_POSE_BASE_PATH       = BASE_DIR_DATABASE +"/LiDAR/"+ Settings::dataset +"/pose/";

    // Kalman Filter Settings
    float&& dt = 1./10.;
    float &&qv = parameters::selectedMotionModel->getVelQ();
    float &&qa = parameters::selectedMotionModel->getAccQ();

    Eigen::Matrix<double, 2, 2> D;
    D << selectedDetector->getXVar(), 0,
            0, selectedDetector->getYVar();

    /// Forms Title ///
    const std::string imgStreamTitle            = "Camera Stream";
    const std::string lidarStreamTitle          = "LiDAR Stream";
    const std::string imgWithDetectionTitle     = "Camera Stream: Detection";


    std::string fileName = "";

    for(const auto& entry : fs::directory_iterator(Calib_POSE_BASE_PATH)) {

        /**
         * Setup initial settings for the system.
         */
        const string CURRENT_STREAM = entry.path().filename().string().substr(0, entry.path().filename().string().find('.'));
        if (Settings::selectedStreamsActivation) {
            bool acceptedStream = false;
            for (const auto& stream : Settings::selectedStreams) {
                if (stream == CURRENT_STREAM) {
                    acceptedStream = true;
                    break;
                }
            }
            if (!acceptedStream) {continue;}
        }

        /// Paths ///
        const string IMG_MAIN_PATH      = IMAGE_DATASET_PATH + CURRENT_STREAM +"/";
        const string LIDAR_MAIN_PATH    = POINTCLOUD_DATASET_PATH + CURRENT_STREAM + "/";

        // Calibration Settings
        const string CALIB_BASE_PATH            = BASE_DIR_DATABASE + "/LiDAR/"+ Settings::dataset +"/calib/";
        const string CALIB_FILE_PATH            = CURRENT_STREAM +".txt";

        std::unique_ptr<DataFrame> dataHolder = std::make_unique<DataFrame>();
        std::vector<int> confirmedObjects;
        /// Initialization ///

        std::cout<<"pose path: "<<Calib_POSE_BASE_PATH + CALIB_FILE_PATH<<std::endl;
        PointCloudCalibration pointCloudCalibration(
                CALIB_BASE_PATH + CALIB_FILE_PATH,
                Calib_POSE_BASE_PATH + CALIB_FILE_PATH
                );


        // Initialize memory
        MemoryManagement memoryManagement(true);

        // // Initialize Kalman Filter.
        KF_Tracking KF_tracker(dt, qa, qv, D);

        int streamId = 0;

        auto timestamps = extractTimestamps(entry.path().string());

        //TODO: while (imgNext || lNext)
        while(streamId < timestamps.size())
        {
            std::string currentTimestamp = timestamps[streamId];
            totalFrames++;

            pointCloudCalibration.updateWorldMat(currentTimestamp);

            /// PointCloud Projection ...///
            std::cout<<"\n(3) Point Cloud Calibration & Projection: Begin"<<std::endl;

            Helper::addPretrainedDetection3D(currentTimestamp, dataHolder.get(), CURRENT_STREAM, pointCloudCalibration ,memoryManagement);
            auto t_start = std::chrono::high_resolution_clock::now();

            pointCloudCalibration.objectsProjectionToWorld(dataHolder->objects);

            std::cout<<"\n(3) Point Cloud Calibration & Projection: End"<<std::endl;

            /// Object Association ..///
            std::cout<<"\n(6) Object Association: Begin"<<std::endl;

            std::map<int, int> matches;
            //            std::cout<<"# objects is "<< memoryManagement.getSavedObjects().size() << std::endl;

            ObjectAssociation::runAssociation(dataHolder->objects,
                                              memoryManagement.getSavedObjects(),
                                              matches,
                                              {
                            ObjectAssociation::AssociationType::LIDAR_3D_ASSOCIATION
                    },
                                              {
                            selectedDetector->getEclTh(),
                    },
                                              false
            );


            std::cout<<"\n(6) Object Association: End"<<std::endl;

            std::cout<<"\n(7) Memory Management: Begin"<<std::endl;

            // /// Memory Update ..///
            memoryManagement.updateAssociatedObjects(matches, dataHolder->objects, streamId);

            std::cout<<"\n(7) Memory Management: End"<<std::endl;

            std::cout<<"\n(8) Kalman Filter Update: Begin"<<std::endl;


            KF_tracker.run({KF_Tracking::Type::KALMAN_FILTER_3D_LIDAR}, memoryManagement.getSavedObjects(), pointCloudCalibration, streamId);

            if (Settings::dataset_name == "kitti")
                pointCloudCalibration.boundingBoxProjection(memoryManagement.getSavedObjects());


            std::cout<<"\n(8) Kalman Filter Update: End"<<std::endl;

            std::cout<<"\n(8) Visualization: Begin"<<std::endl;

            auto t_end = std::chrono::high_resolution_clock::now();


            Helper::saveTrackingData(currentTimestamp, memoryManagement.getSavedObjects(), CURRENT_STREAM, pointCloudCalibration, fileName);

            dataHolder->reset();
            memoryManagement.updateObjectStatus(streamId,selectedDetector->getGhostRemTh(), confirmedObjects);

            double duration = std::chrono::duration<double, std::milli>(t_end-t_start).count();

            sum += duration;

            streamId++;


        }

        Helper::ghostTracksElimination(confirmedObjects, CURRENT_STREAM, fileName);
        std::cout<<"stream end "<<streamId<<std::endl;

        std::cout<<"time in seconds" << totalFrames/(sum /1000.);
    }
    return 0;

}


