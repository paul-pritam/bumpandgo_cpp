#include "fsm_bumpgo_cpp/bumpgonode.hpp"

#include <algorithm>

namespace fsm_bumpgo_cpp
{
    using namespace std::chrono_literals;

BumpGoNode::BumpGoNode() : Node("bumpgo_node"), state_(FORWARD)
{
    scan_sub_= create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&BumpGoNode::scan_callback, this, std::placeholders::_1));
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
    timer_= create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));
    state_ts_=now();
}

void BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
    last_scan_ = std::move(msg);

}

void BumpGoNode::control_cycle()
{
    if (last_scan_ == nullptr){
        RCLCPP_WARN(get_logger(), "No scan received yet");
        return;
    }

    geometry_msgs::msg::Twist vel_msg;

    switch (state_)
    {
        case FORWARD:
            //set output velocity for FORWARD
            vel_msg.linear.x = FORWARD_SPEED;
            vel_msg.angular.z = 0.0;

            //check for state transition
            if (check_forward_2_stop()){
                go_state(STOP);
            }
            else if (check_forward_2_back()){
                go_state(BACK);
            }
            break;

        case BACK:
            //set output velocity for BACK
            vel_msg.linear.x = BACKWARD_SPEED;
            vel_msg.angular.z = 0.0;
            //check for state transition
            if (check_back_2_turn()){
                go_state(TURN); 
            }
            break;
        
        case TURN:
            //set output velocity for TURN
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = TURN_SPEED;
            //check for state transition
            if (check_turn_2_forward()){
                go_state(FORWARD); 
            }
            break;

            case STOP:
            //set output velocity for STOP
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            //check for state transition
            if (check_stop_2_forward()){
                go_state(FORWARD);
            }
            break;
    }
    vel_pub_->publish(vel_msg);
}

void BumpGoNode::go_state(int new_state)
{
    state_ = new_state;
    state_ts_ = now();
    RCLCPP_INFO(get_logger(), "Transitioned to state %d", state_);

}

bool BumpGoNode::check_forward_2_back()
{
    //Get the center index
    size_t center_index = last_scan_->ranges.size()/2;
    //Calculate the angle range for checking obstacles (+- 15 degrees)

    size_t range_width=15;

    size_t start_index = std::max((size_t)0, center_index - range_width);
    size_t end_index = std::min(last_scan_->ranges.size(), center_index + range_width);

    //Check for obstacles in the defined range
    for (size_t i = start_index; i < end_index; ++i){
        if (last_scan_->ranges[i] < OBSTACLE_DISTANCE && last_scan_->ranges[i]>0.0)
        {
            RCLCPP_INFO(get_logger(), "Obstacle detected! Distance: %.2f m", last_scan_->ranges[i]);
            return true;
        }
    }
    return false;
}

bool BumpGoNode::check_forward_2_stop()
{
    //stop if no scan received in 1sec
    auto elapsed =now()- rclcpp::Time(last_scan_->header.stamp);
    return elapsed.seconds() > SCAN_TIMEOUT;
}

bool BumpGoNode::check_back_2_turn()
{
    //after backing for BACKING_TIME seconds, transition to TURN
    auto elapsed = now() - state_ts_;
    return elapsed.seconds() > BACKING_TIME;
}

bool BumpGoNode::check_turn_2_forward()
{
    //after turning for TURNING_TIME seconds, transition to FORWARD
    auto elapsed = now() - state_ts_;
    return elapsed.seconds() > TURNING_TIME;
}

bool BumpGoNode::check_stop_2_forward()
{
    //after stopping for STOP_TIME seconds, transition to FORWARD
    auto elapsed = now() - state_ts_;
    return elapsed.seconds() > STOP_TIME;
}
} // namespace fsm_bumpgo_cpp