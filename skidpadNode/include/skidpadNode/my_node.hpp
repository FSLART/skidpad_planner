#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"


class skidpad_node : public rclcpp::Node
{
    public:
     skidpad_node();

    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        
};