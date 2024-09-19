#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "sim_msgs/msg/adsb.hpp"
#include <random>


class PositionPublisher : public rclcpp::Node {
public:
    PositionPublisher() : Node("position_publisher") {
        // Store the time the node starts
        start_time_ = this->now();

        // Create publishers for each cube
        publisher_cube_ = this->create_publisher<sim_msgs::msg::Adsb>("cube/state", 10);
        publisher_drone_ = this->create_publisher<sim_msgs::msg::Adsb>("drone/state", 10);

        // Create a timer to publish periodically
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PositionPublisher::publish_positions, this));

        // Create client for GetEntityState service
        client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state"); 
    }

private:
    void publish_positions() {
        
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }

        // Get and publish information for each cube
        publish_pose("drone", publisher_drone_);
        publish_pose("cube1", publisher_cube_);
        publish_pose("cube2", publisher_cube_);
        publish_pose("cube3", publisher_cube_);
    }

    // Make an asynchronous entity state request
    void publish_pose(const std::string& model_name, rclcpp::Publisher<sim_msgs::msg::Adsb>::SharedPtr publisher) {

        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = model_name;
        
        // Send the request asynchronously and register the callback
        auto future = client_->async_send_request(request, [this, publisher, model_name](rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future) { 
                try {
                    auto response = future.get();
                    if (response->success) {
                        
                        // Set up random number generation
                        std::random_device rd;  // Non-deterministic random number generator
                        std::mt19937 gen(rd()); // Seed generator
                        std::uniform_real_distribution<> dis(0.0, 1.0);  // Generate numbers between 0 and 1

                        // Generate a random value
                        double random_value = dis(gen);
                        
                        // Skip publishing for the cubes based on a threshold (e.g., 50% chance)
                        if (model_name != "drone" && random_value > 0.5) {
                            RCLCPP_INFO(this->get_logger(), "Skipping publication for %s", model_name.c_str());
                            return;  // Don't publish the message
                        }

                        // From 60s to 80s the cube3 ADS-B won't be publish to test
                        if(model_name == "cube3" && (this->now()-start_time_).seconds() > 60 && (this->now()-start_time_).seconds() < 80){
                            RCLCPP_INFO(this->get_logger(), "%s doesnt publish", model_name.c_str());
                            return;  // Don't publish the message
                        }

                        auto state_msg = sim_msgs::msg::Adsb();

                        // Give position, orientation and velocity
                        state_msg.pose = response->state.pose;   
                        state_msg.twist = response->state.twist;   

                        // Add the ID and size        
                        if(model_name == "drone"){
                            state_msg.id = 0;
                            state_msg.size.width = 0.6;
                            state_msg.size.length = 0.1;
                            state_msg.size.height = 0.6;
                        } else{
                            state_msg.size.width = 0.5;
                            state_msg.size.length = 0.5;                    // Random Gaussian noise should be added to the data
                            state_msg.size.height = 0.5;
                            if(model_name == "cube1")   state_msg.id = 1;
                            if(model_name == "cube2")   state_msg.id = 2;
                            if(model_name == "cube3")   state_msg.id = 3;
                        }
                        
                        publisher->publish(state_msg);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get model state for %s", model_name.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }

    rclcpp::Publisher<sim_msgs::msg::Adsb>::SharedPtr publisher_cube_;
    rclcpp::Publisher<sim_msgs::msg::Adsb>::SharedPtr publisher_drone_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
