#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_moving_robot_interfaces/action/location_speed.hpp"
#include "example_interfaces/msg/empty.hpp"

using MovingRobot = my_moving_robot_interfaces::action::LocationSpeed;
using MovingRobotGoalHandle = rclcpp_action::ClientGoalHandle<MovingRobot>;
using namespace std::placeholders;
using Empty = example_interfaces::msg::Empty;

class MovingRobotClientNode : public rclcpp::Node 
{

public:
    MovingRobotClientNode() :Node("moving_robot_client")
    {
        moving_robot_client_=
            rclcpp_action::create_client<MovingRobot>(this,"moving_robot");

        cancel_subscriber_ = this->create_subscription<Empty>(
            "cancel_move",10,std::bind(&MovingRobotClientNode::callback_cancel_move,this,_1));
    }

    void send_goal(int position, int velocity)
    {
        moving_robot_client_->wait_for_action_server();

        auto goal = MovingRobot::Goal();
        goal.position = position;
        goal.velocity=velocity;

        auto options = rclcpp_action::Client<MovingRobot>::SendGoalOptions();
        options.goal_response_callback = std::bind(&MovingRobotClientNode::goal_response_callback,this,_1);
        options.result_callback=std::bind(&MovingRobotClientNode::goal_result_callback,this,_1);
        options.feedback_callback = std::bind(&MovingRobotClientNode::goal_feedback_callback,this,_1,_2);

        RCLCPP_INFO(this->get_logger(),"Goal sent with position %d and velocity %d",position,velocity);
        moving_robot_client_->async_send_goal(goal,options);

    }
private:
    
    // callback to check goal situation
    void goal_response_callback(const MovingRobotGoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }else
        {
            this->goal_handle_ =goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted"); 
        }
    }
    // cancel move
    void callback_cancel_move(const Empty::SharedPtr msg)
    {
        (void)msg;
        cancel_goal();
    }

    void cancel_goal()
    {
        if (this->goal_handle_)
        {
            this->moving_robot_client_->async_cancel_goal(goal_handle_);
            goal_handle_.reset();
        }
    }

    //callback for result
    void goal_result_callback(const MovingRobotGoalHandle::WrappedResult &result)
    {
        auto status =result.code;
        if(status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeeded"); 
        }else if(status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Aborted"); 
        }else if(status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Cancelled");
        }
        int position = result.result->position;
        RCLCPP_INFO(this->get_logger(),"Result: %d",position);

    }
    // callback for feedback during execution
    void goal_feedback_callback(const MovingRobotGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const MovingRobot::Feedback> feedback)
    {
        (void)goal_handle;
        int position =feedback->current_position;
        RCLCPP_INFO(this->get_logger(), "Feedback position: %d", position);
    }
    rclcpp_action::Client<MovingRobot>::SharedPtr moving_robot_client_;
    MovingRobotGoalHandle::SharedPtr goal_handle_;
    rclcpp::Subscription<Empty>::SharedPtr cancel_subscriber_;
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MovingRobotClientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
