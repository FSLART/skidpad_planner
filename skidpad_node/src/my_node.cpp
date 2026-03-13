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

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>("/mapping/cones", 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/pose", 10, std::bind(&skidpad_node::positionCallback, this, _1));
};


auto localize_car(const lart_msgs::msg::ConeArray::SharedPtr msg, std::pair<int,int> car_pos){
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
            //verificar se é necessario ou n.
        }
    }

           double cones_angle = std::atan2(cones_s[yellow_index].position.y-cones_s[blue_index].position.y,cones_s[yellow_index].position.x-cones_s[blue_index].position.x);    
}

void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg){
    int cone_count = msg->cones.size();
    RCLCPP_INFO(this->get_logger(),"Recive %d cones",cone_count);
    
    
}

//aqui preciso de usar o Z na posicao?
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    //passo isto para variavel global?
    std::pair<int,int> car_pos = {msg->pose.position.x, msg->pose.position.y};
    
    auto car_rotation_x  = msg->pose.orientation.x;
    auto car_rotation_y  = msg->pose.orientation.y;
    auto car_rotation_z  = msg->pose.orientation.z;
    auto car_rotation_w  = msg->pose.orientation.w;




}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}
