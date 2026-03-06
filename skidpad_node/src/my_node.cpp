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
    
    this->path_vis_pub= this->create_publisher<nav_msgs::msg::Path>("/slam/pose",10);
    this->path_control_pub = this->create_publisher<lart_msgs::msg::PathSpline>("/planning/path",10);

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>("/planning/path", 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/pose", 10, std::bind(&skidpad_node::positionCallback, this, _1));
};


void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg){

}


void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}


/*
This node will subscribe to the current position of the car (x,y, theta), the observed cones and will publish the next x meters of the path.
Informations:
  • x Should be configurable with a macro
  • The topic of the observed cones is /mapping/cones
  • The topic of the pose is /slam/pose
  • The topic of the planned path is /planning/path
  • The planned path must have curvature, x, y and distance to the car
*/