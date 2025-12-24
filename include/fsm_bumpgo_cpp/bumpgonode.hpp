#ifndef FSM_BUMPGO_CPP_BUMPGONODE_HPP_
#define FSM_BUMPGO_CPP_BUMPGONODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace fsm_bumpgo_cpp
{
class BumpGoNode : public rclcpp::Node
{
public:
    BumpGoNode();
    
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::UniquePtr msg);
    void control_cycle();

    void go_state(int new_state);

    bool check_forward_2_back();
    bool check_forward_2_stop();
    bool check_back_2_turn();
    bool check_turn_2_forward();
    bool check_stop_2_forward();

    static const int FORWARD = 0;
    static const int BACK = 1;
    static const int TURN = 2;
    static const int STOP = 3;

    int state_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

    rclcpp::Time state_ts_;

    static constexpr float OBSTACLE_DISTANCE = 0.5;  // meters
    static constexpr float SCAN_TIMEOUT = 0.5;       // seconds
    static constexpr float BACKING_TIME = 0.5;       // seconds
    static constexpr float TURNING_TIME = 0.5;       // seconds
    static constexpr float STOP_TIME = 1.0;          // seconds

    static constexpr float FORWARD_SPEED = 0.3;      // meters/second
    static constexpr float BACKWARD_SPEED = -0.3;    // meters/second
    static constexpr float TURN_SPEED = 0.2;         // radians/second

    int turn_direction_ = 1; 
    double turn_dir_multiplier_; 
};
}  // namespace fsm_bumpgo_cpp

#endif  // FSM_BUMPGO_CPP_BUMPGONODE_HPP_
