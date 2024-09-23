/**
 * @file tracker.cpp
 * @author Javier Gil Aviles (javgilavi)
 * @brief Tracker from ADS-B of different cubes with a low-pass filter
 * @version 1.2
 * @copyright PUBLIC
 */

#include "multitracker/tracker.h"   // Library with all dependencies and declarations

// For knowing errors ----------------------------------------------

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
//--------------------------------------------------------------------


// 1.5 FUNCTION TO MAKE MEDIAN VALUE OF SENSORDATA VECTOR -------------------------------------------------

// Subfunction to calculate median from any value vector
template<typename T>
T median(std::vector<T>& values) {
    size_t size = values.size();
    if (size == 0) {
        throw std::domain_error("Cannot compute median of an empty vector");
    }
    std::sort(values.begin(), values.end());
    if (size % 2 == 0) {
        return (values[size / 2 - 1] + values[size / 2]) / 2.0;
    } else {
        return values[size / 2];
    }
}

// Function to calculate median of a vector of SensorData
SensorData median_sensordata(std::vector<SensorData> data_buffer){
    SensorData median_data = data_buffer[0];
    
    // Vectors to store individual components for calculating median
    std::vector<double> x_positions, y_positions, z_positions;
    std::vector<double> x_orientations, y_orientations, z_orientations, w_orientations;
    std::vector<double> x_velocities, y_velocities, z_velocities;
    std::vector<float> widths, lengths, heights;
    
    // Extract each value from each SensorData into separate vectors
    for (const auto& data : data_buffer) {
        x_positions.push_back(data.position.x);
        y_positions.push_back(data.position.y);
        z_positions.push_back(data.position.z);

        x_orientations.push_back(data.orientation.x);
        y_orientations.push_back(data.orientation.y);
        z_orientations.push_back(data.orientation.z);
        w_orientations.push_back(data.orientation.w);

        x_velocities.push_back(data.vel.x);
        y_velocities.push_back(data.vel.y);
        z_velocities.push_back(data.vel.z);

        widths.push_back(data.size.width);
        lengths.push_back(data.size.length);
        heights.push_back(data.size.height);
    }

    // Calculate median for each component and assign it to the new SensorData
    median_data.position.x = median(x_positions);
    median_data.position.y = median(y_positions);
    median_data.position.z = median(z_positions);

    median_data.orientation.x = median(x_orientations);
    median_data.orientation.y = median(y_orientations);
    median_data.orientation.z = median(z_orientations);
    median_data.orientation.w = median(w_orientations);

    median_data.vel.x = median(x_velocities);
    median_data.vel.y = median(y_velocities);
    median_data.vel.z = median(z_velocities);

    median_data.size.width = median(widths);
    median_data.size.length = median(lengths);
    median_data.size.height = median(heights);

    return median_data;
}
// 1.5 ----------------------------------------------------------------------------------------------------


Tracker::Tracker() : Node("simple_tracker"){

    // Store the time the node starts
    start_time_ = this->now();

    // Initial value. When change to 0 the drone GPS is working
    drone_state.id = 4;

    // Create subscriber for cube1 and drone
    subscriber_cube_ = this->create_subscription<sim_msgs::msg::Adsb>("cube/state", 10, std::bind(&Tracker::data_recieve, this, std::placeholders::_1));
    subscriber_drone_ = this->create_subscription<sim_msgs::msg::Adsb>("drone/state", 10, std::bind(&Tracker::drone_update, this, std::placeholders::_1));

    // Create publisher for rviz2 representation
    rviz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cube/marker", 10);

    // Create a timer to publish periodically predict the tracked state, also other to publish in rviz2
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Tracker::kalman_predict, this));
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Tracker::rviz_pub, this));
}


void Tracker::data_recieve(const sim_msgs::msg::Adsb::SharedPtr sensor_state){ 

    // Only transform from global to local if we have drone state. Now we only use cube1
    if(drone_state.id == 0){
        // Set up random number generation
        std::random_device rd;  // Non-deterministic random number generator
        std::mt19937 gen(rd()); // Seed generator
        std::uniform_real_distribution<> dis(0.0, 1.0);  // Generate numbers between 0 and 1

        // Generate a random value
        double random_value = dis(gen);

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

        // For taking the error of the output list with the real state ----------------------------
        for (auto& obs : obs_list) {
            if (obs.id == fix_state.id) {
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
            }
        }
        // -------------------------------------------------------------------------------------------

        // Skip publishing for the cubes based on a threshold (e.g., 50% chance)
        if (random_value > 1) {
            RCLCPP_INFO(this->get_logger(), "Skipping publication for Cube%d", fix_state.id);
            return;  // Don't pass the data
        }

        // From 60s to 80s the cube3 ADS-B won't be publish to test
        if(fix_state.id == 3 && (this->now()-start_time_).seconds() > 60 && (this->now()-start_time_).seconds() < 80){
            RCLCPP_INFO(this->get_logger(), "Cube3 doesnt publish");
            return;  // Don't pass the message
        }

        // Send the fixed sensor to the low-pass filter
        lowpass_filter(fix_state);
    }
}

// 1. MAKE A LOW-PASS FILTER FOR THE INPUT DATA OF THE SENSORS --------------------------------------------
void Tracker::lowpass_filter(const sim_msgs::msg::Adsb fix_state){ 

    // Define quantity of data for the buffer of low-pass filter
    uint32_t n_buffer = 10;
    
    // Change data to SensorData format
    SensorData obs;
    obs.id = fix_state.id;
    obs.position = fix_state.pose.position;
    obs.orientation = fix_state.pose.orientation;
    obs.vel = fix_state.twist.linear;
    obs.size.width = fix_state.size.width;
    obs.size.length = fix_state.size.length;
    obs.size.height = fix_state.size.height;
    obs.timestamp = this->now();

    // Search the buffer on the list with the same ID
    for (auto& obs_buffer : obs_buffer_list){
        if(obs_buffer[0].id == obs.id){

            // Enter the new data at the start
            obs_buffer.insert(obs_buffer.begin(), obs);

            // If the buffer surpass limit value eliminate the oldest (last value)
            if(obs_buffer.size() > n_buffer){
                obs_buffer.pop_back();
            }

            // If buffer is full make the median value and send it to kalman update
            if(obs_buffer.size() == n_buffer){
                SensorData median_obs;
                median_obs = median_sensordata(obs_buffer);

                // Send the low-pass filter data with median for kalman update
                kalman_update(median_obs);     
            }

            // To end the loop and function
            return; 
        }
    }

    // In case this ID data has not appear still insert it
    std::vector<SensorData> new_obs_buffer;
    new_obs_buffer.push_back(obs);
    obs_buffer_list.push_back(new_obs_buffer);

}
// 1 ------------------------------------------------------------------------------------------------------


void Tracker::kalman_update(const SensorData median_state){ 

    // Declare of the state and covariance for the update of kalman
    VectorXd state(13);     // State is [x, y, z, angx, angy, angz, angw, vx, vy, vz, width, length, height]
    MatrixXd covariance(13,13);
    
    // Vector of data recieve from the sensor
    VectorXd sensorData(13); 

    // C Matrix of the reading module
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

    // R Matrix for noise acknowledge of the sensor t the filter. Change to 0.1
    MatrixXd R(13,13);
    R << 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1;
    
    // Get each obstacle in the list
    for (auto& obs : obs_list) {

        // Only update the obstacle in the list with the same ID
        if(obs.id == median_state.id){

            // Before updating with kalman, always predict
            kalman_predict();   // Revisar quitar este predict o hacer que solo prediga el obstaculo en cuestion

            // Give the vector and matrix the previous value store from Kalman
            state = obs.state;
            covariance = obs.covariance;

            // Fill the sensorData with the fixed state of the cube to the drone
            sensorData(0) = median_state.position.x;
            sensorData(1) = median_state.position.y;
            sensorData(2) = median_state.position.z;
            sensorData(3) = median_state.orientation.x;
            sensorData(4) = median_state.orientation.y;
            sensorData(5) = median_state.orientation.z;
            sensorData(6) = median_state.orientation.w;
            sensorData(7) = median_state.vel.x;
            sensorData(8) = median_state.vel.y;
            sensorData(9) = median_state.vel.z;
            sensorData(10) = median_state.size.width;
            sensorData(11) = median_state.size.length;
            sensorData(12) = median_state.size.height;
        
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
            obs.time_update = this->now();           // 2 ADD NEW TIME VALUE

            // Print to check results
            // std::cout << " Update State (only position and velocity [x y z vx vy vz]):" << std::endl;
            // std::cout << "  " << state(0) << ", " << state(1) << ", " << state(2) << ", " << state(7) << ", " << state(8) << ", " << state(9) << std::endl;
            // std::cout << " Update Covariance: \n" << covariance << std::endl;

            return;     // Once the obstacle is updated we stop the loop and function
        }
    }

    // In case the loop ended and no upload was made we add the new data to the list
    SensorData obs;
    obs.id = median_state.id;
    obs.position = median_state.position;
    obs.orientation = median_state.orientation;
    obs.vel = median_state.vel;
    obs.size = median_state.size;
    obs.timestamp = this->now();
    obs.time_update = this->now();      // 2 ADD NEW TIME VALUE

    // Fill the state of the obs and the initial covariance
    obs.state(0) = obs.position.x;  
    obs.state(1) = obs.position.y;  
    obs.state(2) = obs.position.z;  
    obs.state(3) = obs.orientation.x;  
    obs.state(4) = obs.orientation.y;  
    obs.state(5) = obs.orientation.z;  
    obs.state(6) = obs.orientation.w;  
    obs.state(7) = obs.vel.x; 
    obs.state(8) = obs.vel.y; 
    obs.state(9) = obs.vel.z; 
    obs.state(10) = obs.size.width;
    obs.state(11) = obs.size.length; 
    obs.state(12) = obs.size.height;

    obs.covariance.setZero();
    obs.covariance.diagonal().setConstant(0.01);

    // Push back to the vector of the list of obstacle this new obstacle detected
    obs_list.push_back(obs);
}


void Tracker::kalman_predict(){ 

    // Declare of the state and covariance for the prediction of kalman
    VectorXd state(13);     // State is [x, y, z, angx, angy, angz, angw, vx, vy, vz, width, length, height]
    MatrixXd covariance(13,13);

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

    // Vector to store the obs wanting to delate
    std::vector<SensorData> to_remove;              // 2

    // Get each obstacle in the list
    for (auto& obs : obs_list) {
        
        // 2. IF OBSTACLE MORE THAN X TIME SINCE LAST UPDATE DELATE IT (ASSUME DISSAPEAR) --------------------------------------------
        uint32_t t_alive = 5;
        if(this->now().seconds()-obs.time_update.seconds() >= t_alive){
            to_remove.push_back(obs);
        }
        // 2. ------------------------------------------------------------------------------------------------------------------------

        // Print the id of the obstacle to predict from the list
        // RCLCPP_INFO(this->get_logger(), "Obstacle to predict with ID: %d", obs.id);

        // Give the vector and matrix the previous value store from Kalman
        state = obs.state;
        covariance = obs.covariance;

        // Time variation since last prediction
        double var_time = this->now().seconds() - obs.timestamp.seconds();

        // A Matrix for state space equation
        MatrixXd A(13,13);
        A << 1, 0, 0, 0, 0, 0, 0, var_time, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, var_time, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, var_time, 0, 0, 0,
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

        // RCLCPP_INFO(this->get_logger(), "State %d: ", obs.id);
        // RCLCPP_INFO(this->get_logger(), " -X: %f \n -Y: %f \n -Z: %f ", obs.position.x, obs.position.y, obs.position.z);
        // RCLCPP_INFO(this->get_logger(), " -QuatX: %f \n -QuatY: %f \n -QuatZ: %f \n -QuatW: %f ", obs.orientation.x, obs.orientation.y, obs.orientation.z, obs.orientation.w);
        // RCLCPP_INFO(this->get_logger(), " -VX: %f \n -VY: %f \n -VZ: %f ", obs.vel.x, obs.vel.y, obs.vel.z);
        // RCLCPP_INFO(this->get_logger(), " -Width: %f \n -Length: %f \n -Height: %f ", obs.size.width, obs.size.length, obs.size.height);

        // Update the time of last prediction
        obs.timestamp = this->now();

        // Print to check the results
        // std::cout << " Predict State (only position and velocity [x y z vx vy vz]):" << std::endl;
        // std::cout << "  " << state(0) << ", " << state(1) << ", " << state(2) << ", " << state(7) << ", " << state(8) << ", " << state(9) << std::endl;
        // std::cout << " Predict Covariance: \n" << covariance << std::endl;
    }

    // 2. IF OBSTACLE MORE THAN X TIME SINCE LAST UPDATE DELATE IT (ASSUME DISSAPEAR) --------------------------------------------
    for (auto& obs : to_remove) {
        obs_list.erase(std::remove(obs_list.begin(), obs_list.end(), obs), obs_list.end());
        RCLCPP_INFO(this->get_logger(), "Obstacle %d eliminated", obs.id);
    }
    // 2. ------------------------------------------------------------------------------------------------------------------------

}


void Tracker::drone_update(const sim_msgs::msg::Adsb::SharedPtr msg){
    // CREATE VARIABLE TO STORE THE DRONE POSITION AND UPDATE IT IN EACH CALLBACK
    drone_state = *msg;
}


void Tracker::rviz_pub(){

    // Add the marker publication for rviz2 test
    visualization_msgs::msg::MarkerArray marker_array;

    for (auto& obs : obs_list) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();

        marker.id = obs.id;                                     // ID unique per marker
        marker.type = visualization_msgs::msg::Marker::CUBE;    // CUBE marker type
        marker.action = visualization_msgs::msg::Marker::ADD;   // Action to add the marker

        // Time to disappear if not detect again
        marker.lifetime.sec = 0;   // Segundos
        marker.lifetime.nanosec = 500*1e6;

        // Position and orientation of the cube
        marker.pose.position = obs.position;
        marker.pose.orientation = obs.orientation;

        // Dimensions of the cube
        marker.scale.x = obs.size.width;    // Width
        marker.scale.y = obs.size.length;   // Length
        marker.scale.z = obs.size.height;   // Height

        // Cube color (RGBA). Different for each cube
        if (obs.id == 1){
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f; 

        } else if (obs.id == 2){
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

        } else if (obs.id == 3){
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }

        // Push the cube marker to the marker array
        marker_array.markers.push_back(marker);
        
    }

    // Pulish in the topic
    rviz_publisher_->publish(marker_array);
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
