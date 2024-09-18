/**
 * @file tracker.h
 * @author Javier Gil Aviles (javgilavi)
 * @brief Tracker from ADS-B of different cubes
 * @version 1.1
 * @copyright PUBLIC
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
#include "sim_msgs/msg/adsb.hpp"

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
};

// ROS2 node for the cube tracker
class Tracker : public rclcpp::Node {
    public:
        Tracker();

    private:
        // Callback for the data received from the cube1 ADSB and transform from global to local (drone).
        void data_recieve(const sim_msgs::msg::Adsb::SharedPtr msg);

        // Predict the state of the tracked.
        void kalman_predict();

        // Update the state of the tracked cube with the data received.
        void kalman_update(const sim_msgs::msg::Adsb fix_state);

        // Callback for the updating of the GPS data of our drone.
        void drone_update(const sim_msgs::msg::Adsb::SharedPtr msg);

        // Function to publish the tracker data to marker in rviz2 for visualization
        void rviz_pub();

        // Subscribers for the ADSB data of the cube and the drone.
        rclcpp::Subscription<sim_msgs::msg::Adsb>::SharedPtr subscriber_cube_;
        rclcpp::Subscription<sim_msgs::msg::Adsb>::SharedPtr subscriber_drone_;

        // Publisher for rviz2 marker visualization
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_publisher_;

        // Timer for periodic prediction and state update.
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;

        // Variables 
        sim_msgs::msg::Adsb drone_state;    // To store GPS data from the LIDAR. Start empty
        // SensorData obs;                     // State of the obstacle detected. Case one track
        std::vector<SensorData> obs_list;   // Vector to list all tracks
        
    };

