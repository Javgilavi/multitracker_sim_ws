/**
 * @file tracker.h
 * @author Javier Gil Aviles (javgilavi)
 * @brief Tracker from ADS-B and LIDAR of different cubes with a low-pass filter
 * @version 1.3
 * @copyright PUBLIC
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
#include "sim_msgs/msg/adsb.hpp"
#include <unordered_map>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

// Structure for dimensions
struct Dimension{
    float width;
    float length;
    float height;
};

// Structure to store sensordata as state of a tracked obstacle
struct SensorData{
    int id;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Vector3 vel;
    Dimension size;

    Eigen::Matrix<double, 13, 1> state;         // State and covariance for Kalman
    Eigen::Matrix<double, 13, 13> covariance;

    rclcpp::Time timestamp;                     // Time of last predict or update
    rclcpp::Time time_update;                   // Time of the last update. If more than X delate obstacle from list

    // Define the equality operator
    bool operator==(const SensorData& other) const {
        return id == other.id;
    }
};

// ROS2 node for the cube tracker
class Tracker : public rclcpp::Node {
    public:
        Tracker();

    private:
        // Callback for the data received from the cube1 ADSB and transform from global to local (drone).
        void data_recieve(const sim_msgs::msg::Adsb::SharedPtr msg);

        // Apply a low-pass filter before entering the data rom the sensor to Kalman
        void lowpass_filter(const sim_msgs::msg::Adsb fix_state);

        // Update the state of the tracked cube with the data received.
        void kalman_update(const SensorData median_state);

        // Predict the state of the tracked.
        void kalman_predict();

        // Callback for the updating of the GPS data of our drone.
        void drone_update(const sim_msgs::msg::Adsb::SharedPtr msg);

        // Callback of the LIDAR and a driver for it
        void driver_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud);

        // Function to publish the tracker data to marker in rviz2 for visualization
        void rviz_pub();

        // Subscribers for the ADSB data of the cube and the drone.
        rclcpp::Subscription<sim_msgs::msg::Adsb>::SharedPtr subscriber_cube_;
        rclcpp::Subscription<sim_msgs::msg::Adsb>::SharedPtr subscriber_drone_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_lidar_;

        // Publisher for rviz2 marker visualization
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_cloud_;              // ELIMINAR
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_lidar_cubes_;

        // Timer for periodic prediction and state update.
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;

        // Variables 
        sim_msgs::msg::Adsb drone_state;                        // To store GPS data from the LIDAR. Start empty
        std::vector<SensorData> obs_list;                       // Vector to list all tracks
        std::vector<std::vector<SensorData>> obs_buffer_list;   // Buffer to make the low-pass filter
        rclcpp::Time start_time_;
        
    };

