#include "../include/skidpadNode/my_node.hpp"

using std::placeholders::_1;

Eigen::MatrixXd plan_path(
    const Eigen::MatrixXd& cones,
    const Eigen::Vector2d& car_pos)
{
    // logic here
}

skidpad_node::skidpad_node() : Node("skidpadNode"){
    RCLCPP_INFO(this->get_logger(),"Skidpad node has been started");
    
    this->path_pub = this->create_publisher<nav_msgs::msg::Path>("/slam/pose",10);
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}