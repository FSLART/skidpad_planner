#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp> //publisher para o vissualizer
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/cone.hpp"
#include "lart_msgs/msg/cone_array.hpp"


class skidpad_node : public rclcpp::Node
{
    public:
     skidpad_node();

    private:
        const double distance = 20;
        bool car_localized = false;
        double car_angle;
        size_t path_index = 0;

        std::pair<bool,bool> car_pos;
        std::vector<lart_msgs::msg::Cone> rotated_cones;
    
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub;
        rclcpp::Publisher<lart_msgs::msg::PathSpline>::SharedPtr path_control_pub; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
        rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr cone_array_subscriber;
        
        void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg);
        
        void points_sender();
        void localize_car(const lart_msgs::msg::ConeArray::SharedPtr msg);
};