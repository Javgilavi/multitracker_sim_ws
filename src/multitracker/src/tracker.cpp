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
    drone_state.id = 4;

    // Create subscriber for cube1 and drone
    subscriber_cube_ = this->create_subscription<sim_msgs::msg::Adsb>("cube/state", 10, std::bind(&Tracker::data_recieve, this, std::placeholders::_1));
    subscriber_drone_ = this->create_subscription<sim_msgs::msg::Adsb>("drone/state", 10, std::bind(&Tracker::drone_update, this, std::placeholders::_1));

    // Create publisher for rviz2 representation
    rviz_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cube/marker", 10);

    // Create a timer to publish periodically predict the tracked state, also other to publish in rviz2
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Tracker::kalman_predict, this));
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Tracker::rviz_pub, this));
}


void Tracker::data_recieve(const sim_msgs::msg::Adsb::SharedPtr sensor_state){ 

    // 1. NEED TO PASS DATA FROM GLOBAL VALUES TO LOCAL VALUES (DRONE) -----------------------------------------------

    // Only transform from global to local if we have drone state. Now we only use cube1
    if(drone_state.id == 0 && sensor_state->id == 1){
        sim_msgs::msg::Adsb fix_state = *sensor_state;

        // Substract drone pose, orientation and velocity to the sensor datas for fixing it to be relative to the drone state
        fix_state.pose.position.x = fix_state.pose.position.x - drone_state.pose.position.x;
        fix_state.pose.position.y = fix_state.pose.position.y - drone_state.pose.position.y;
        fix_state.pose.position.z = fix_state.pose.position.z - drone_state.pose.position.z;

        fix_state.pose.orientation.x = fix_state.pose.orientation.x - drone_state.pose.orientation.x;
        fix_state.pose.orientation.y = fix_state.pose.orientation.y - drone_state.pose.orientation.y;
        fix_state.pose.orientation.z = fix_state.pose.orientation.z - drone_state.pose.orientation.z;
        fix_state.pose.orientation.w = fix_state.pose.orientation.w - drone_state.pose.orientation.w;

        fix_state.twist.linear.x = fix_state.twist.linear.x - drone_state.twist.linear.x;
        fix_state.twist.linear.y = fix_state.twist.linear.y - drone_state.twist.linear.y;
        fix_state.twist.linear.z = fix_state.twist.linear.z - drone_state.twist.linear.z;


        // Print the fixed position to check the results
        sim_msgs::msg::Adsb prefix_state = *sensor_state;
        RCLCPP_INFO(this->get_logger(), "Data before fixing: X: %f, Y: %f, Z: %f", prefix_state.pose.position.x, prefix_state.pose.position.y, prefix_state.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Data after fixing: X: %f, Y: %f, Z: %f", fix_state.pose.position.x, fix_state.pose.position.y, fix_state.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "---------------------");

        // Compute position errors for each axis (X, Y, Z)
        double pos_error_x_val = fix_state.pose.position.x - obs.position.x;
        double pos_error_y_val = fix_state.pose.position.y - obs.position.y;
        double pos_error_z_val = fix_state.pose.position.z - obs.position.z;
        
        // Compute orientation errors for each component (X, Y, Z, W)
        double ori_error_x_val = fix_state.pose.orientation.x - obs.orientation.x;
        double ori_error_y_val = fix_state.pose.orientation.y - obs.orientation.y;
        double ori_error_z_val = fix_state.pose.orientation.z - obs.orientation.z;
        double ori_error_w_val = fix_state.pose.orientation.w - obs.orientation.w;

        // Store errors in maps by ID
        pos_error_x_map[obs.id].push_back(pos_error_x_val);
        pos_error_y_map[obs.id].push_back(pos_error_y_val);
        pos_error_z_map[obs.id].push_back(pos_error_z_val);
        
        ori_error_x_map[obs.id].push_back(ori_error_x_val);
        ori_error_y_map[obs.id].push_back(ori_error_y_val);
        ori_error_z_map[obs.id].push_back(ori_error_z_val);
        ori_error_w_map[obs.id].push_back(ori_error_w_val);

        // Send the fixed sensor to the kalman update ...
    }

    // 1. ---------------------------------------------------------------------------------------------------------------
}


void Tracker::drone_update(const sim_msgs::msg::Adsb::SharedPtr msg){
    // CREATE VARIABLE TO STORE THE DRONE POSITION AND UPDATE IT IN EACH CALLBACK
    drone_state = *msg;
}


void Tracker::rviz_pub(){
    // Add the marker publication for rviz2 test
    visualization_msgs::msg::Marker marker;

    // CUBE marker type
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();

    // Position and orientation of the cube
    marker.pose.position = obs.position;
    marker.pose.orientation = obs.orientation;

    // Dimensions of the cube
    marker.scale.x = obs.size.width;    // Width
    marker.scale.y = obs.size.length;   // Length
    marker.scale.z = obs.size.height;   // Height

    // Cube color (RGBA)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;  // Transparency

    marker.id = obs.id;  // ID unique per marker
    marker.action = visualization_msgs::msg::Marker::ADD;  // Action to add the marker

    // Pulish in the topic
    rviz_publisher_->publish(marker);
}


// Main to spin the ROS2 node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker>());
    rclcpp::shutdown();

    // Loop over all IDs in the position error maps
    std::ofstream error_file("error_data.txt");
    std::cout << "----" << std::endl;
    for (const auto& [id, pos_errors_x] : pos_error_x_map) {
        double rmse_pos_x = calculate_rmse(pos_errors_x);
        double rmse_pos_y = calculate_rmse(pos_error_y_map[id]);
        double rmse_pos_z = calculate_rmse(pos_error_z_map[id]);

        double rmse_ori_x = calculate_rmse(ori_error_x_map[id]);
        double rmse_ori_y = calculate_rmse(ori_error_y_map[id]);
        double rmse_ori_z = calculate_rmse(ori_error_z_map[id]);
        double rmse_ori_w = calculate_rmse(ori_error_w_map[id]);

        // Log the RMSE for this ID
        std::cout << "ID " << id << " - RMSE Position (X: " << rmse_pos_x << ", Y: " << rmse_pos_y << ", Z: " << rmse_pos_z << ")" << std::endl;
        std::cout << "     - RMSE Orientation (X: " << rmse_ori_x << ", Y: " << rmse_ori_y << ", Z: " << rmse_ori_z << ", W: " << rmse_ori_w << ")" << std::endl;
        std::cout << "----" << std::endl;

        // Save the errors
        for (int i = 0; i < static_cast<int>(pos_errors_x.size()); ++i){
            error_file << "ID: " << id << " ";
            error_file << pos_errors_x[i] << " " << pos_error_y_map[id][i] << " " << pos_error_z_map[id][i] << " ";
            error_file << ori_error_x_map[id][i] << " " << ori_error_y_map[id][i] << " " << ori_error_z_map[id][i] << " " << ori_error_w_map[id][i] << std::endl;
        }
    }
    error_file.close();

    return 0;
}
