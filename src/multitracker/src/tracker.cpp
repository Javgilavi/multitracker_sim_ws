/**
 * @file tracker.cpp
 * @author Javier Gil Aviles (javgilavi)
 * @brief Tracker from ADS-B and LIDAR of different cubes with a low-pass filter
 * @version 1.3
 * @copyright PUBLIC
 */

#include "multitracker/tracker.h"   // Library with all dependencies and declarations

int new_id = 4;

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


// 1.5 DISTANCE FUNCTION FOR MATCHING -----------------------------------------------------------------------------------
double distance(const SensorData& data1, const SensorData& data2){
    double dist;

    double dist_x = data1.position.x - data2.position.x;
    double dist_y = data1.position.y - data2.position.y;
    double dist_z = data1.position.z - data2.position.z;

    dist = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2) + std::pow(dist_z, 2));
    // Return the Euclidean distance
    return dist;
}
// ----------------------------------------------------------------------------------------------------------------------


Tracker::Tracker() : Node("simple_tracker"){

    // Store the time the node starts
    start_time_ = this->now();

    // Initial value. When change to 0 the drone GPS is working
    drone_state.id = 4;

    // Create subscriber for cube1 and drone
    subscriber_cube_ = this->create_subscription<sim_msgs::msg::Adsb>("cube/state", 10, std::bind(&Tracker::data_recieve, this, std::placeholders::_1));
    subscriber_drone_ = this->create_subscription<sim_msgs::msg::Adsb>("drone/state", 10, std::bind(&Tracker::drone_update, this, std::placeholders::_1));
    subscriber_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("drone/laser/scan", 10, std::bind(&Tracker::driver_lidar, this, std::placeholders::_1));

    // Create publisher for rviz2 representation
    rviz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cube/marker", 10);
    filter_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("drone/laser/filter", 10);
    rviz_lidar_cubes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lidar/marker", 10);

    // Create a timer to publish periodically predict the tracked state, also other to publish in rviz2
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Tracker::kalman_predict, this));
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Tracker::rviz_pub, this));
}


// 1. MATCHING SYSTEM BETWEEN LIDAR TRACES WITH THE OBSTACLE LIST ----------------------------------------------------------
void Tracker::matching(std::vector<SensorData> lidar_traces){ 
    // If for a X lidar trace the minimum distance is with Y obstacle in list, and for Y obstacle the minimum distance is also with X lidar is a MATCH
    // If not, and X lidar trace has more distance than distance threshold to all obstacle in the list, considere a new obstacle and add new ID

    std::vector<SensorData> lidar_matched;

    double match_threshold = 1.25;             // Theshold of distance to considere a matchdouble min_dist;   
    double min_dist;
    SensorData obs_match, trace_match;

    int i = 1;
    for (auto& trace : lidar_traces) {
        min_dist = match_threshold;
        RCLCPP_INFO(this->get_logger(), "Lectura de traza %d", i++);
        for (auto& obs : obs_list) {
            double dist = distance(trace, obs);                 // Calculate distance between the trace and the obstacle
            RCLCPP_INFO(this->get_logger(), "Distancia con obstaculo al ID %d es %f", obs.id, dist);
            if(dist < min_dist){
                obs_match = obs;                                // Update minimum distance and obstacle for possible match
                min_dist = dist;
            }
        }
        
        // Only analice if there was a match, if not creare new data giving it an ID
        if(obs_match.id != 0){   
            RCLCPP_INFO(this->get_logger(), "Posible match con %d", obs_match.id);
            // Once we have possible obstacle match for the trace, check if for that obstacle the trace is also its minimum distance value
            for(auto& trace2 : lidar_traces){
                double dist = distance(trace2, obs_match);    // Calculate distance between the obstacle and all trace
                if(dist <= min_dist){
                    trace_match = trace2;                     // Update minimum distance and trace with minimum distance to this obstacle
                    min_dist = dist;
                }
            }

            // Only if trace with minimum distance to the obstacle possible match is the same as original trace we do a match
            if(trace_match.position.x == trace.position.x){
                trace.id = obs_match.id;
                trace.timestamp = this->now();
                lidar_matched.push_back(trace);
                RCLCPP_INFO(this->get_logger(), "Match con Cube %d", obs_match.id);
                lowpass_filter(trace);
            }
        } else{
            // In case is a new data create it
            trace.id = new_id++;
            trace.timestamp = this->now();
            RCLCPP_INFO(this->get_logger(), "New Cube with ID %d", trace.id);
            // kalman_update(trace);   // Uncomment foe next step
        }
    }
    RCLCPP_INFO(this->get_logger(), "-------");
}
// 1 -------------------------------------------------------------------------------------------------------------------------


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

        // Skip publishing for the cubes based on a threshold (e.g., 50% chance)
        if (random_value > 0.75) {
            // RCLCPP_INFO(this->get_logger(), "Skipping publication for Cube%d", fix_state.id);
            return;  // Don't pass the data
        }

        // From 30s to 50s the cube2 ADS-B won't be publish to test
        if(fix_state.id == 2 && (this->now()-start_time_).seconds() > 30 && (this->now()-start_time_).seconds() < 50){
            RCLCPP_INFO(this->get_logger(), "Cube2 doesnt publish");
            return;  // Don't pass the message
        }

        // Send the fixed sensor to the kalman update
        SensorData obs;
        obs.id = fix_state.id;
        obs.position = fix_state.pose.position;
        obs.orientation = fix_state.pose.orientation;
        obs.vel = fix_state.twist.linear;
        obs.size.width = fix_state.size.width;
        obs.size.length = fix_state.size.length;
        obs.size.height = fix_state.size.height;
        obs.timestamp = this->now();
        RCLCPP_INFO(this->get_logger(), "Cube %d recieve. PASAMOS A SU UPDATE", obs.id);
        kalman_update(obs);
    }
}


void Tracker::lowpass_filter(const SensorData obs){ 

    // Define quantity of data for the buffer of low-pass filter
    uint32_t n_buffer = 10;


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
                // kalman_update(median_obs);   // Uncomment for starting next step  
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


void Tracker::kalman_update(const SensorData median_state){ 

    // Declare of the state and covariance for the update of kalman
    VectorXd state(13);     // State is [x, y, z, angx, angy, angz, angw, vx, vy, vz, width, length, height]
    MatrixXd covariance(13,13);
    
    // Vector of data recieve from the sensor
    VectorXd sensorData(13); 

    // 2 CHANGE KALMAN FILTER UPDATE FOR EACH SENSOR DATA -------------------------------------------------------
    // C Matrix of the reading module || R Matrix for noise acknowledge of the sensor t the filter.

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
            
// 2 -------------------------------------------------------------------------------------------------------

            // Calculate Kalman Gain
            MatrixXd K = covariance*C.transpose()*(C*covariance*C.transpose() + R).inverse();

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
            obs.time_update = this->now();          

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
    obs.time_update = this->now();   

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
    R << 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1.5, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1.5, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5;

    // Vector to store the obs wanting to delate
    std::vector<SensorData> to_remove;           

    // Get each obstacle in the list
    for (auto& obs : obs_list) {
        
        uint32_t t_alive = 5;
        if(this->now().seconds()-obs.time_update.seconds() >= t_alive){
            to_remove.push_back(obs);
        }

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

        // Update the time of last prediction
        obs.timestamp = this->now();
    }

    for (auto& obs : to_remove) {
        obs_list.erase(std::remove(obs_list.begin(), obs_list.end(), obs), obs_list.end());
        RCLCPP_INFO(this->get_logger(), "Obstacle %d eliminated", obs.id);
    }

}


void Tracker::drone_update(const sim_msgs::msg::Adsb::SharedPtr msg){
    drone_state = *msg;
}


// OPTIONAL. LIDAR DRIVER -----------------------------------------------------------------------------
void Tracker::driver_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_cloud){

    std::vector<SensorData> lidar_traces;    // Vector to send LIDAR traces after process

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*lidar_cloud, *cloud);

    // RANSAC plane segmentation to extract the ground
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the points that are not part of the plane (non-ground points)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Remove plane points (ground)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*filtered_cloud);

    // SHOW LIDAR WITHOUT GROUND ON RVIZ 
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    // Set the header information for the message
    output_msg.header = lidar_cloud->header;  // Use the same header from the input message (frame_id, timestamp, etc.)
    // Publish the filtered point cloud
    filter_cloud_->publish(output_msg);

    // Extract data from the cubes
    // Separate the cubes by clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); 
    ec.setMinClusterSize(10);     // Minimum points for a cluster to be considered a cube
    ec.setMaxClusterSize(25000);  // Maximum number of points per cube
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);    // Points divide in the different clusters

    visualization_msgs::msg::MarkerArray marker_array;  // To visualice in RVIZ
    int32_t cluster_id = 0;

    // Loop through each cluster (representing each cube)
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Extract points corresponding to this cluster
        for (const auto& idx : indices.indices) {
            cube_cloud->points.push_back(filtered_cloud->points[idx]);
        }

        // Detect first plane in the cube, assume is a face of the cube
        pcl::SACSegmentation<pcl::PointXYZ> cube_seg;
        pcl::PointIndices::Ptr cube_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr cube_coefficients(new pcl::ModelCoefficients);
        cube_seg.setModelType(pcl::SACMODEL_PLANE);
        cube_seg.setMethodType(pcl::SAC_RANSAC);
        cube_seg.setDistanceThreshold(0.05);  
        cube_seg.setInputCloud(cube_cloud);
        cube_seg.segment(*cube_inliers, *cube_coefficients);    // Segment the first plane (one face of the cube)

        // Normal of the first plane
        Eigen::Vector3f normal1(cube_coefficients->values[0], cube_coefficients->values[1], cube_coefficients->values[2]);

        // Extract the points of the first plane (first face of the cube)
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> cube_extract;
        cube_extract.setInputCloud(cube_cloud);
        cube_extract.setIndices(cube_inliers);
        cube_extract.setNegative(false);  // Extract the points that belong to the plane
        cube_extract.filter(*plane1_cloud);  
        Eigen::Vector4f centroid1;
        pcl::compute3DCentroid(*plane1_cloud, centroid1);   // Compute the centroid of the first plane
        
        // Remove the first plane points and find a second plane (another face of the cube)
        cube_extract.setNegative(true);  // Remove the points of the first plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cube_extract.filter(*remaining_cloud);
        cube_seg.setInputCloud(remaining_cloud);
        cube_seg.segment(*cube_inliers, *cube_coefficients);

        // Normal of the second plane
        Eigen::Vector3f normal2(cube_coefficients->values[0], cube_coefficients->values[1], cube_coefficients->values[2]);

        // Extract the points of the second plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cube_extract.setInputCloud(remaining_cloud);
        cube_extract.setIndices(cube_inliers);
        cube_extract.setNegative(false);  // Extract the second plane
        cube_extract.filter(*plane2_cloud);
        Eigen::Vector4f centroid2;
        pcl::compute3DCentroid(*plane2_cloud, centroid2);   // Compute the centroid of the first plane

        // Convert centroids to 3D vectors (x, y, z) by ignoring the 4th element
        Eigen::Vector3f centroid1_3d(centroid1[0], centroid1[1], centroid1[2]);
        Eigen::Vector3f centroid2_3d(centroid2[0], centroid2[1], centroid2[2]);

        // Estimate cube dimensions 
        pcl::PointXYZ min_point_1, max_point_1;
        pcl::getMinMax3D(*plane1_cloud, min_point_1, max_point_1);

        // Compute the dimensions of the cube based on the extent of the points
        float width = max_point_1.x - min_point_1.x;  
        float length = max_point_1.y - min_point_1.y; 
        float height = width;                          // Suppose height as width

        // Compute the overall cube centroid using the centroids and normal vectors of both planes.
        Eigen::Vector3f cube_centroid = centroid1_3d - height/2 * normal1;

        // Estimate the orientation with the normals vector
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix.col(0) = normal1.normalized();  // Set the first column to normal1
        rotation_matrix.col(1) = normal2;    
        rotation_matrix.col(2) = normal1.cross(normal2).normalized();   // Compute the third column (orthogonal to both)
        Eigen::Quaternionf orientation(rotation_matrix);    // Convert the rotation matrix to quaternion

        // Print the results for each cube
        // RCLCPP_INFO(this->get_logger(), "-----");
        // RCLCPP_INFO(this->get_logger(), "Cube centroid: (%f, %f, %f)", cube_centroid(0), cube_centroid(1), cube_centroid(2));
        // RCLCPP_INFO(this->get_logger(), "Cube orientation (quaternion): (%f, %f, %f, %f)", orientation.x(), orientation.y(), orientation.z(), orientation.w());
        // RCLCPP_INFO(this->get_logger(), "Cube dimensions (WxLxH): (%f, %f, %f)", width, length, height);
        // RCLCPP_INFO(this->get_logger(), "-----");

        // Enter trace of the cube in the vector to send
        SensorData trace;
        trace.position.x = cube_centroid(0);
        trace.position.y = cube_centroid(1);
        trace.position.z = cube_centroid(2);
        trace.orientation.x = orientation.x();
        trace.orientation.y = orientation.y();
        trace.orientation.z = orientation.z();
        trace.orientation.w = orientation.w();
        trace.size.width = width;
        trace.size.length = length;
        trace.size.height = height;
        lidar_traces.push_back(trace);

        //RVIZ
        // Create a marker for each cube
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = lidar_cloud->header.frame_id;
        marker.header.stamp = lidar_cloud->header.stamp;
        marker.ns = "cubes";
        marker.id = cluster_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the cube based on the centroid
        marker.pose.position.x = cube_centroid(0);
        marker.pose.position.y = cube_centroid(1);
        marker.pose.position.z = cube_centroid(2);

        // Set the orientation of the cube
        marker.pose.orientation.x = orientation.x();
        marker.pose.orientation.y = orientation.y();
        marker.pose.orientation.z = orientation.z();
        marker.pose.orientation.w = orientation.w();

        // Estimate cube size by the bounding box of the points (simple approximation)
        marker.scale.x = width;
        marker.scale.y = length;
        marker.scale.z = height;

        // Set marker color (adjust for each cube if needed)
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime.sec = 0;   
        marker.lifetime.nanosec = 500*1e6;

        marker_array.markers.push_back(marker);
    }

    rviz_lidar_cubes_->publish(marker_array);

    // Pass the lidar_traces to the matching system
    matching(lidar_traces);

}
// ----------------------------------------------------------------------------------------------------


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
        } else{
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
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
