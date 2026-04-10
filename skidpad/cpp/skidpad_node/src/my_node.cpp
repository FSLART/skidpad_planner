#include "../include/skidpadNode/my_node.hpp"
using std::placeholders::_1;

/*
Nao esquecer de fazer as perguntas que tao nos comments !!!!!
e apagar os mesmos
*/
Eigen::MatrixXd plan_path(
    const Eigen::MatrixXd &cones,
    const Eigen::Vector2d &car_pos)
{
    // logic here
}

skidpad_node::skidpad_node() : Node("skidpadNode")
{
    RCLCPP_INFO(this->get_logger(), "Skidpad node has been started");

    this->path_vis_pub = this->create_publisher<nav_msgs::msg::Path>("/slam/pose", 10);
    this->path_control_pub = this->create_publisher<lart_msgs::msg::PathSpline>("/planning/path", 10);

    this->cone_array_subscriber = this->create_subscription<lart_msgs::msg::ConeArray>("/mapping/cones", 10, std::bind(&skidpad_node::coneArrayCallback, this, _1));
    this->position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/pose", 10, std::bind(&skidpad_node::positionCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Trying to open skidpad_path.csv");
    // perguntar se a problema de abrir o ficheiro aqui
    PATH_POINTS.open("skidpad_path.csv");
};
// pontos equidistantes na path spline
void skidpad_node::localize_car(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    auto cones_s = msg->cones;
    int blue_index, yellow_index;
    // if it returns -1 something went wrong
    double dist_b = 999;
    double dist_y = 999;
    double tmp_distance_blue, tmp_distance_yellow = 10;

    auto distance = [](double cone_x, double cone_y, double car_x, double car_y)
    {
        return (cone_x - car_x) * (cone_x - car_x) + (cone_y - car_y) * (cone_y - car_y);
    };

    // return the index off the nearest cones
    for (int i = 0; i < cones_s.size(); i++)
    {
        if (cones_s[i].BLUE == 2)
        {
            tmp_distance_blue = distance(cones_s[i].position.x, cones_s[i].position.y, car_pos.first, car_pos.second);
            if (tmp_distance_blue < dist_b)
            {
                dist_b = tmp_distance_blue;
                blue_index = i;
            }
        }
        else if (cones_s[i].YELLOW == 1)
        {
            tmp_distance_yellow = distance(cones_s[i].position.x, cones_s[i].position.y, car_pos.first, car_pos.second);
            if (tmp_distance_yellow < dist_b)
            {
                dist_y = tmp_distance_yellow;
                yellow_index = i;
            }
        }
    }

    std::pair<double, double> cones_mid_point = {
        (cones_s[blue_index].position.x + cones_s[yellow_index].position.x) / 2.0,
        (cones_s[blue_index].position.y + cones_s[yellow_index].position.y) / 2.0};

    double map_angle = std::atan2(
        cones_mid_point.second - car_pos.second,
        cones_mid_point.first - car_pos.first);
    // rotation to be aplied in the map
    double rotation = map_angle - this->car_angle;

    double cos = std::cos(rotation);
    double sin = std::sin(rotation);
    // rodar o path inteiro

    std::ofstream tmp_file("skidpad_path_rotated.csv");
    std::string line;
    while (std::getline(PATH_POINTS, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str;
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str))
        {
            double x = std::stod(x_str);
            double y = std::stod(y_str);

            double x_rot = (x * cos) - (y * sin);
            double y_rot = (x * sin) - (y * cos);

            tmp_file << std::scientific << x_rot << "," << y_rot << std::endl;
        }
    }
    PATH_POINTS.close();
    tmp_file.close();
}

void skidpad_node::points_sender()
{
    double target_distance = 0.5;
    double added_distance = 0.0;

    lart_msgs::msg::PathSpline pathSpline_msg;
    pathSpline_msg.header.stamp = this->now();
    pathSpline_msg.header.frame_id = "world";

    nav_msgs::msg::Path path_rviz_msg;
    path_rviz_msg.header.stamp = this->now();
    path_rviz_msg.header.frame_id = "world";

    std::pair<double, double> current_point;
    std::pair<double, double> last_sent_point = {0.0, 0.0};

    auto distance = [](double cone_x, double cone_y, double cone_x1, double cone_y1)
    {
        return std::sqrt((cone_x - cone_x1) * (cone_x - cone_x1) + (cone_y - cone_y1) * (cone_y - cone_y1));
    };

    if (!PATH_POINTS.is_open())
    {
        PATH_POINTS.open("skidpad_path_rotated.csv");
    }
    std::string line;
    size_t current_line_in_file = 0;
    while (std::getline(PATH_POINTS, line))
    {
        // Jump to the last line
        if (current_line_in_file < path_index)
        {
            current_line_in_file++;
            continue;
        }

        std::stringstream ss(line);
        std::string x_str, y_str;

        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str))
        {
            current_point.first = std::stod(x_str);
            current_point.second = std::stod(y_str);
            double dist = distance(current_point.first, current_point.second,
                                   last_sent_point.first, last_sent_point.second);
            if (dist >= target_distance)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = pathSpline_msg.header.frame_id;

                pose.pose.position.x = current_point.first;
                pose.pose.position.y = current_point.second;

                tf2::Quaternion quaternion;
                quaternion.setRPY(0.0, 0.0, car_angle);

                pose.pose.orientation.x = quaternion.x();
                pose.pose.orientation.y = quaternion.y();
                pose.pose.orientation.z = quaternion.z();
                pose.pose.orientation.w = quaternion.w();

                /*
                    path_msg.curvature.append(point[3])
                    path_msg.distance.append(point[0])
                */

                pathSpline_msg.poses.push_back(pose);
                path_rviz_msg.poses.push_back(pose);

                added_distance += target_distance;
                last_sent_point = current_point;
            }

            if (added_distance > 20)
            {
                path_control_pub->publish(pathSpline_msg);
                path_vis_pub->publish(path_rviz_msg);
                break;
            }
            path_index++;
        }
    }
}

void skidpad_node::coneArrayCallback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    int cone_count = msg->cones.size();
    RCLCPP_INFO(this->get_logger(), "Recive %d cones", cone_count);
    if (!(car_localized))
    {
        localize_car(msg);
        car_localized = !car_localized;
    }
}

// aqui preciso de usar o Z na posicao?
void skidpad_node::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // passo isto para variavel global?
    this->car_pos = {msg->pose.position.x, msg->pose.position.y};
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    this->car_angle = yaw;
    // verificar se é aqui que tenho de mandar os pontos todos ou n
    points_sender();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<skidpad_node>());
    rclcpp::shutdown();
    return 0;
}