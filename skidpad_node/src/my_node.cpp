#include "../include/skidpadNode/my_node.hpp"
using std::placeholders::_1;

#define DISTANCE 20

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

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>("/mapping/cones", 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/pose", 10, std::bind(&skidpad_node::positionCallback, this, _1));

};

//rever 
void skidpad_node::localize_car(const lart_msgs::msg::ConeArray::SharedPtr msg){
    auto cones_s = msg->cones;  
    int blue_index, yellow_index;
    //if it returns -1 something went wrong
    double dist_b = 999;
    double dist_y = 999;
    double tmp_distance_blue,tmp_distance_yellow = 10;

    auto distance  = [](double cone_x, double cone_y, double car_x,double car_y){
        return (cone_x - car_x)*(cone_x - car_x)+(cone_y - car_y)*(cone_y - car_y);
    };
    
    //return the index off the nearest cones
    for(int i = 0; i<cones_s.size();i++)
    {
        if(cones_s[i].BLUE == 2){
            tmp_distance_blue = distance(cones_s[i].position.x,cones_s[i].position.y,car_pos.first,car_pos.second);
            if(tmp_distance_blue < dist_b){
                dist_b = tmp_distance_blue;
                blue_index = i;
            }
        
        }else if(cones_s[i].YELLOW == 1){
            tmp_distance_yellow = distance(cones_s[i].position.x,cones_s[i].position.y,car_pos.first,car_pos.second);
            if(tmp_distance_yellow < dist_b){
                dist_y = tmp_distance_yellow;
                yellow_index= i;
            }
        }else{
            //verify if its needed
        }
    }

    std::pair<double,double> cones_mid_point = {(cones_s[blue_index].position.x - cones_s[yellow_index].position.x)/2,
        (cones_s[blue_index].position.y - cones_s[yellow_index].position.y)/2};
        
    double map_angle = std::atan2(
        cones_mid_point.second - car_pos.second,
         cones_mid_point.first - car_pos.first);
    //rotation to be aplied in the map
    double rotation = map_angle - this->car_angle;
    
    double cos = std::cos(rotation);
    double sin = std::sin(rotation);

    for(const auto& cone : cones_s){
        auto new_cone = cone;

        double dx = cone.position.x;
        double dy = cone.position.y;

        double rotated_x = (dx*cos) - (dy * sin);
        double rotated_y = (dx * sin) - (dy * cos);

        double final_x = rotated_x + car_pos.first;
        double final_y = rotated_y + car_pos.second;
    
        new_cone.position.x = final_x;
        new_cone.position.y = final_y;

        rotated_cones.push_back(new_cone);
    }
}

void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg){
    int cone_count = msg->cones.size();
    RCLCPP_INFO(this->get_logger(),"Recive %d cones",cone_count);
    if(!(car_localized)){
        localize_car(msg);
        car_localized = !car_localized;
    }
    
}

//aqui preciso de usar o Z na posicao?
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    //passo isto para variavel global?
    this->car_pos = {msg->pose.position.x, msg->pose.position.y};
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch ,yaw;
    m.getRPY(roll,pitch,yaw);
    this->car_angle = yaw;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}
