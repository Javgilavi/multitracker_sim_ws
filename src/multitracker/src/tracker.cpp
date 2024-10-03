/**
 * @file tracker.cpp
 * @author Javier Gil Aviles (javgilavi)
 * @brief Tracker from ADS-B of one cube
 * @version 1.0
 * @copyright PUBLIC
 */

#include "multitracker/tracker.h"   // Library with all dependencies and declarations

// Map to store position and orientation errors for each ID
std::unordered_map<int, std::vector<double>> pos_error_x_map;
std::unordered_map<int, std::vector<double>> pos_error_y_map;
std::unordered_map<int, std::vector<double>> pos_error_z_map;

std::unordered_map<int, std::vector<double>> ori_error_x_map;
std::unordered_map<int, std::vector<double>> ori_error_y_map;
std::unordered_map<int, std::vector<double>> ori_error_z_map;
std::unordered_map<int, std::vector<double>> ori_error_w_map;

double calculate_rmse(const std::vector<double>& errors) {
    double sum_error = 0.0;
    for (double error : errors) {
        sum_error += error * error;
    }
    return std::sqrt(sum_error / errors.size());
}

Tracker::Tracker() : Node("simple_tracker"){

    // Initial value. When change to 0 the drone GPS is working
    // drone_state.id = 4;

    // Create subscriber for cube1 and drone
    subscriber_cube_ = this->create_subscription<sim_msgs::msg::Adsb>("cube/state", 10, std::bind(&Tracker::data_recieve, this, std::placeholders::_1));
    subscriber_drone_ = this->create_subscription<sim_msgs::msg::Adsb>("drone/state", 10, std::bind(&Tracker::drone_update, this, std::placeholders::_1));

}


void Tracker::data_recieve(const sim_msgs::msg::Adsb::SharedPtr sensor_state){ 

    // 1. NEED TO PASS DATA FROM GLOBAL VALUES TO LOCAL VALUES (DRONE) -----------------------------------------------

    // Only transform from global to local if we have drone state. Now we only use cube1

    // 1. ---------------------------------------------------------------------------------------------------------------
}


void Tracker::drone_update(const sim_msgs::msg::Adsb::SharedPtr msg){
    // CREATE VARIABLE TO STORE THE DRONE POSITION AND UPDATE IT IN EACH CALLBACK
    drone_state = *msg;
}


// Main to spin the ROS2 node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker>());
    rclcpp::shutdown();

    return 0;
}
