/**
 * @file tracker.cpp
 * @author Javier Gil Aviles (javgilavi)
 * @brief Tracker from ADS-B of one cube
 * @version 1.0
 * @copyright PUBLIC
 */

#include "multitracker/tracker.h"   // Library with all dependencies and declarations


Tracker::Tracker() : Node("simple_tracker"){

    // Initial value. When change to 0 the drone GPS is working
    drone_state.id = 4;

    // Create subscriber for cube1 and drone
    subscriber_cube_ = this->create_subscription<sim_msgs::msg::Adsb>("cube/state", 10, std::bind(&Tracker::data_recieve, this, std::placeholders::_1));
    subscriber_drone_ = this->create_subscription<sim_msgs::msg::Adsb>("drone/state", 10, std::bind(&Tracker::drone_update, this, std::placeholders::_1));

    // Create publisher for rviz2 representation
    rviz_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cube/marker", 10);

    // Create a timer to publish periodically predict the tracked state, also other to publish in rviz2
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Tracker::kalman_predict, this));
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Tracker::rviz_pub, this));
}


void Tracker::data_recieve(const sim_msgs::msg::Adsb::SharedPtr sensor_state){ 

    // 1. NEED TO PASS DATA FROM GLOBAL VALUES TO LOCAL VALUES (DRONE)

    // Only transform from global to local if we have drone state. Now we only use cube1
    if(drone_state.id == 0 && sensor_state->id == 1){
        sim_msgs::msg::Adsb prefix_state = *sensor_state;
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

        fix_state.twist.angular.x = fix_state.twist.angular.x - drone_state.twist.angular.x;
        fix_state.twist.angular.y = fix_state.twist.angular.y - drone_state.twist.angular.y;
        fix_state.twist.angular.z = fix_state.twist.angular.z - drone_state.twist.angular.z;


        // Print the fixed position to check the results
        // RCLCPP_INFO(this->get_logger(), "Data before fixing: X: %f, Y: %f, Z: %f", prefix_state.pose.position.x, prefix_state.pose.position.y, prefix_state.pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "Data after fixing: X: %f, Y: %f, Z: %f", fix_state.pose.position.x, fix_state.pose.position.y, fix_state.pose.position.z);
        // RCLCPP_INFO(this->get_logger(), "---------------------");

        // Send the fixed sensor for kalman update
        kalman_update(fix_state);
    }
}


void Tracker::kalman_predict(){ 

    // 2. DEVELOPE THE KALMAN PREDICT FOR THE TRACKER

    // Declare of the state and covariance for the prediction of kalman
    VectorXd state(13);     // State is [x, y, z, angx, angy, angz, angw, vx, vy, vz, width, length, height]
    MatrixXd covariance(13,13);
    
    // Give the vector and matrix the previous value store from Kalman
    state = obs.state;
    covariance = obs.covariance;

    // Time variation since last prediction
    double time = this->now().seconds() - obs.timestamp.seconds();

    // A Matrix for state space equation
    MatrixXd A(13,13);
    A << 1, 0, 0, 0, 0, 0, 0, time, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, time, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0, time, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    // State^[i+1] = A*State^[i] || State -> [x,y,z,angx,angy,angz,w,vx,vy,vz,width,len,height]
    state = A*state;

    // R Matrix for noise assume of the prediction
    MatrixXd R(13,13);
    R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5;

    // Covariance calculation
    covariance = A*covariance*A.transpose() + R;

    // Restoring state values and covariance
    obs.position.x = state(0);  
    obs.position.y = state(1);  
    obs.position.z = state(2);  
    obs.orientation.x = state(3);  
    obs.orientation.y = state(4);  
    obs.orientation.z = state(5);  
    obs.orientation.w = state(6);  
    obs.vel.x = state(7);  
    obs.vel.y = state(8);  
    obs.vel.z = state(9);  
    obs.size.width = state(10);
    obs.size.length = state(11); 
    obs.size.height = state(12); 

    obs.state = state;
    obs.covariance = covariance;

    // Update the time of last prediction
    obs.timestamp = this->now();

    // Print to check the results
    // std::cout << " Predict State (only position and velocity [x y z vx vy vz]):" << std::endl;
    // std::cout << "  " << state(0) << ", " << state(1) << ", " << state(2) << ", " << state(7) << ", " << state(8) << ", " << state(9) << std::endl;
    // std::cout << " Predict Covariance: \n" << covariance << std::endl;
}


void Tracker::kalman_update(const sim_msgs::msg::Adsb fix_state){ 
    // 2. DEVELOPE THE KALMAN UPDATE FOR THE TRACKER
    
    // Before updating with kalman, always predict
    kalman_predict();

    // Declare of the state and covariance for the update of kalman
    VectorXd state(13);     // State is [x, y, z, angx, angy, angz, angw, vx, vy, vz, width, length, height]
    MatrixXd covariance(13,13);
    
    // Give the vector and matrix the previous value store from Kalman
    state = obs.state;
    covariance = obs.covariance;

    // Vector of data recieve from the sensor
    VectorXd sensorData(13); 

    // Fill the sensorData with the fixed state of the cube to the drone
    sensorData(0) = fix_state.pose.position.x;
    sensorData(1) = fix_state.pose.position.y;
    sensorData(2) = fix_state.pose.position.z;
    sensorData(3) = fix_state.pose.orientation.x;
    sensorData(4) = fix_state.pose.orientation.y;
    sensorData(5) = fix_state.pose.orientation.z;
    sensorData(6) = fix_state.pose.orientation.w;
    sensorData(7) = fix_state.twist.linear.x;
    sensorData(8) = fix_state.twist.linear.y;
    sensorData(9) = fix_state.twist.linear.z;
    sensorData(10) = fix_state.size.width;
    sensorData(11) = fix_state.size.length;
    sensorData(12) = fix_state.size.height;

    // C Matrix of the medition modelo
    MatrixXd C(13,13);
    C << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;


    // R Matrix for noise acknowledge of the sensor t the filter 
    MatrixXd R(13,13);
    R << 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01;
        
    // Calculate Kalman Gain
    MatrixXd K = covariance*C.transpose()*(C*covariance*C.transpose() + R).inverse();

    // Print to check results
    // std::cout << " C'*Q*C: \n" << C*covariance*C.transpose() << std::endl;
    // std::cout << " (C'*Q*C+R)^-1: \n" << (C*covariance*C.transpose() + R).inverse() << std::endl;
    // std::cout << " Ganancia de Kalman: \n" << K << std::endl;
    // VectorXd Kalman_error = (sensorData - C*state);
    // std::cout << " Kalman Error (only position and velocity [x y z vx vy vz]): " << std::endl;
    // std::cout << "  " << Kalman_error(0) << ", " << Kalman_error(1) << ", " << Kalman_error(2) << ", " << Kalman_error(7) << ", " << Kalman_error(8) << ", " << Kalman_error(9) << std::endl;

    // Update Kalman equation for the state
    state = state + K*(sensorData - C*state);

    // Update of the covariance
    covariance = (MatrixXd::Identity(13,13) - K*C)*covariance;

    // Restoring state values and covariance
    obs.position.x = state(0);  
    obs.position.y = state(1);  
    obs.position.z = state(2);  
    obs.orientation.x = state(3);  
    obs.orientation.y = state(4);  
    obs.orientation.z = state(5);  
    obs.orientation.w = state(6);  
    obs.vel.x = state(7);  
    obs.vel.y = state(8);  
    obs.vel.z = state(9);  
    obs.size.width = state(10);
    obs.size.length = state(11); 
    obs.size.height = state(12); 

    obs.state = state;
    obs.covariance = covariance;

    // Update the time of last prediction
    obs.timestamp = this->now();
    
    // Print to check results
    // std::cout << " Update State (only position and velocity [x y z vx vy vz]):" << std::endl;
    // std::cout << "  " << state(0) << ", " << state(1) << ", " << state(2) << ", " << state(7) << ", " << state(8) << ", " << state(9) << std::endl;
    // std::cout << " Update Covariance: \n" << covariance << std::endl;

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
    return 0;
}
