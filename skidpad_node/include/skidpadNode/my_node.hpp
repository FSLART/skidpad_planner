#include <Eigen/Dense>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp> //publisher para o vissualizer
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/cone.hpp"
#include "lart_msgs/msg/cone_array.hpp"

class skidpad_node : public rclcpp::Node
{
    public:
     skidpad_node();

    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub;
        rclcpp::Publisher<lart_msgs::msg::PathSpline>::SharedPtr path_control_pub; 


        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
        rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr cone_array_subscriber;
        
        void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg);



};