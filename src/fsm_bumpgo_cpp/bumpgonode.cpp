#include "fsm_bumpgo_cpp/bumpgonode.hpp"
#include <algorithm>

namespace fsm_bumpgo_cpp
{
    using namespace std::chrono_literals;

BumpGoNode::BumpGoNode() : Node("bumpgo_node"), state_(FORWARD)
{
    scan_sub_= create_subscription<sensor_msgs::msg::LaserScan>("input_scan", 10, std::bind(&BumpGoNode::scan_callback, this, std::placeholders::_1));
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
    timer_= create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));
    state_ts_=now();
    
    // Initialize a multiplier for turning direction
    // 1.0 = Left, -1.0 = Right
    turn_dir_multiplier_ = 1.0; 
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
    size_t n = last_scan_->ranges.size();

    switch (state_)
    {
        case FORWARD:
            vel_msg.linear.x = FORWARD_SPEED;
            vel_msg.angular.z = 0.0;

            if (check_forward_2_stop()) go_state(STOP);
            else if (check_forward_2_back()) go_state(BACK);
            break;

        case BACK:
            vel_msg.linear.x = BACKWARD_SPEED;
            vel_msg.angular.z = 0.0;
                if (check_back_2_turn()){
                // FRONTIER ANALYSIS: Compare the total free space on Left vs Right
                double right_dist_sum = 0.0;
                double left_dist_sum = 0.0;

                for (size_t i = 0; i < n/2; ++i) {
                    // Use finite values to calculate the "openness" of the right side
                    if (std::isfinite(last_scan_->ranges[i])) right_dist_sum += last_scan_->ranges[i];
                }
                for (size_t i = n/2; i < n; ++i) {
                    // Use finite values to calculate the "openness" of the left side
                    if (std::isfinite(last_scan_->ranges[i])) left_dist_sum += last_scan_->ranges[i];
                }

                // Choose direction: Turn towards the side with the larger sum of distances (the Frontier)
                turn_dir_multiplier_ = (left_dist_sum > right_dist_sum) ? 1.0 : -1.0;
                
                go_state(TURN); 
            }
            break;
        
        case TURN:
            vel_msg.linear.x = 0.0;
            // Use the calculated multiplier to turn toward the detected gap
            vel_msg.angular.z = turn_dir_multiplier_ * TURN_SPEED;

            if (check_turn_2_forward()) go_state(FORWARD); 
            break;

        case STOP:
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            if (check_stop_2_forward()) go_state(FORWARD);
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
    size_t n = last_scan_->ranges.size();
    size_t window = n * 0.05; // 5% window on each side of center (~30deg total FOV)
    size_t center = n / 2;

    for (size_t i = center - window; i <= center + window; ++i){
        if (last_scan_->ranges[i] < OBSTACLE_DISTANCE){
            return true;
        }
    }
    return false;   
}

bool BumpGoNode::check_forward_2_stop()
{
    auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
    return elapsed.seconds() > SCAN_TIMEOUT;
}

bool BumpGoNode::check_back_2_turn() { return (now() - state_ts_).seconds() > BACKING_TIME; }
bool BumpGoNode::check_turn_2_forward() { return (now() - state_ts_).seconds() > TURNING_TIME; }
bool BumpGoNode::check_stop_2_forward() { return (now() - state_ts_).seconds() > STOP_TIME; }

} // namespace fsm_bumpgo_cpp