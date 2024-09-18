#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cstdlib>   

float vel_x = 0;
float vel_y = 0;

class CubeMover : public rclcpp::Node {
public:
    CubeMover() : Node("cube_mover") {
        // Create client for the SetEntityState service
        clientSet_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

        // Client to receive state
        clientGet_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state"); 

        // Wait for the services to become available
        while (!clientSet_->wait_for_service(std::chrono::seconds(5))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Node interrupted while waiting for set_entity_state service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service /gazebo/set_entity_state to become available...");
        }

        while (!clientGet_->wait_for_service(std::chrono::seconds(5))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Node interrupted while waiting for get_entity_state service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service /gazebo/get_entity_state to become available...");
        }

        // Create a timer to periodically publish
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {this->get_state("cube1");});  // Call get_state with the argument "cube1"
    }

private:

    // Function to update the state
    void move_cube(gazebo_msgs::msg::EntityState previous_state) {
        // Create a request for the SetEntityState service
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

        // Set the name of the cube (entity) to be moved
        request->state = previous_state;

        // Set the velocity (twist)
        request->state.twist.linear.x = vel_x;  // Velocity on the X-axis
        request->state.twist.linear.y = vel_y;
        request->state.twist.linear.z = 0.0;

        request->state.twist.angular.x = 0.0;
        request->state.twist.angular.y = 0.0;
        request->state.twist.angular.z = 0.0;

        // Randomly update the velocities
        vel_x = vel_x + (static_cast<double>(rand()) / RAND_MAX) * 0.5 - 0.25;
        vel_y = vel_y + (static_cast<double>(rand()) / RAND_MAX) * 0.5 - 0.25;

        // Set velocity limits
        if (vel_x >= 1) vel_x = 1;
        if (vel_x <= -1) vel_x = -1;
        if (vel_y >= 1) vel_y = 1;
        if (vel_y <= -1) vel_y = -1;

        // Send the request to the service
        auto future = clientSet_->async_send_request(request, [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture future) { 

                // Wait for the service response
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Cube1 is moving at %f m/s in X", vel_x);
                        RCLCPP_INFO(this->get_logger(), "Cube1 is moving at %f m/s in Y", vel_y);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to move cube1");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }

    // Function to get the state
    void get_state(const std::string &model_name) {

        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = model_name;
        
        // Send the request asynchronously and register the callback
        auto future = clientGet_->async_send_request(request, [this, model_name](rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future) { 
                try {
                    auto response = future.get();
                    if (response->success) {
                        response->state.name = model_name;
                        move_cube(response->state);  // Send the previous state to the movement function
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get model state for %s", model_name.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );

    }

    // Service client to change the entity's state
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr clientSet_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr clientGet_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeMover>());
    rclcpp::shutdown();
    return 0;
}
