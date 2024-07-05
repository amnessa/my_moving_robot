#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "my_robot_interfaces/action/LocationSpeed.hpp"

using MovingRobot= my_robot_interfaces::action::LocationSpeed;

class MovingRobotServerNode : public rclcpp::Node 
{

public:
    MovingRobotServerNode() :Node("moving_robot_server")
    {

    } 

private:
    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
};


int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MovingRobotServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
