#include "rclcpp/rclcpp.hpp"
#include "fsm_bumpgo_cpp/bumpgonode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fsm_bumpgo_cpp::BumpGoNode>());
    rclcpp::shutdown();
    return 0;
}