#include <QCoreApplication>
#include <QTcpSocket>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

using namespace std;

// Function to calibrate the camera using OpenCV
void calibrateCamera(cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    // Object points and image points are needed for calibration
    vector<vector<cv::Point3f>> objectPoints; // Real-world 3D points
    vector<vector<cv::Point2f>> imagePoints; // Corresponding image 2D points

    // Specify the size of the captured images
    cv::Size imageSize(640, 480);

    // Perform camera calibration
    cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray());
}

// Function to load a point cloud from a file
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const string& filePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filePath, *cloud) == -1) {
        cerr << "Failed to load point cloud file: " << filePath << endl;
        exit(-1); // Exit if the file cannot be loaded
    }
    return cloud;
}

// Function to downsample the point cloud using a voxel grid
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f); // Leaf size controls the resolution
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}

// Function to extract a Region of Interest (ROI) from the point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr extractROI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::CropBox<pcl::PointXYZ> cropFilter;
    cropFilter.setMin(Eigen::Vector4f(-1.0, -1.0, -1.0, 1.0)); // Define the minimum corner of the ROI
    cropFilter.setMax(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));   // Define the maximum corner of the ROI
    cropFilter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr roiCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropFilter.filter(*roiCloud);
    return roiCloud;
}

// Function to compute normal vectors for the point cloud
pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setRadiusSearch(0.03); // Neighborhood radius for normal estimation
    normalEstimator.compute(*normals);
    return normals;
}

// Function to cluster points using Euclidean clustering
void clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // Distance threshold for clustering
    ec.setMinClusterSize(100);   // Minimum number of points in a cluster
    ec.setMaxClusterSize(25000); // Maximum number of points in a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);

    cout << "Number of clusters found: " << clusterIndices.size() << endl;
}

// Function to send coordinates to a master system via TCP/IP
void sendCoordinates(const string& ip, int port, const Eigen::Vector3f& coordinates) {
    QTcpSocket socket;
    socket.connectToHost(QString::fromStdString(ip), port);
    if (socket.waitForConnected(3000)) {
        // Format the message with the coordinates
        stringstream message;
        message << "Coordinates: " << coordinates[0] << ", " << coordinates[1] << ", " << coordinates[2] << "\n";
        socket.write(message.str().c_str());
    } else {
        cerr << "Failed to connect to master system at " << ip << ":" << port << endl;
    }
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    // Step 1: Perform camera calibration
    cv::Mat cameraMatrix, distCoeffs;
    calibrateCamera(cameraMatrix, distCoeffs);

    // Step 2: Load the point cloud data
    auto cloud = loadPointCloud("pointcloud.pcd"); // Replace with your point cloud file

    // Step 3: Downsample the point cloud for faster processing
    auto filteredCloud = filterPointCloud(cloud);

    // Step 4: Extract the Region of Interest (ROI)
    auto roiCloud = extractROI(filteredCloud);

    // Step 5: Compute normals for the point cloud
    auto normals = calculateNormals(roiCloud);

    // Step 6: Perform clustering on the filtered point cloud
    clusterPointCloud(roiCloud);

    // Step 7: Transform the point cloud to robot coordinates
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 1.0, 2.0, 3.0; // Example translation vector
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*roiCloud, *transformedCloud, transform);

    // Step 8: Send the robot coordinates to the master system
    Eigen::Vector3f robotCoordinates(1.0, 2.0, 3.0); // Example robot coordinates
    sendCoordinates("192.168.0.1", 12345, robotCoordinates);

    return a.exec();
}