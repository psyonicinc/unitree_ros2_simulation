#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

class EffortExample: public rclcpp::Node{
    public:
        EffortExample() : Node("effort_control_example") 
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_effort_controller/commands", 10);   
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&EffortExample::timer_callback, this));
            force_flag_ = true;
            rest_ = std::vector<double>(6, 0.0);
            force_ = {0.0, 0.0, -8.5, 0.0, 0.0, 0.0};
            
        }

        void timer_callback(){
            std_msgs::msg::Float64MultiArray msg_ = std_msgs::msg::Float64MultiArray();
            if (force_flag_) msg_.data = force_;
            else msg_.data = rest_;

            pub_->publish(msg_);
            force_flag_ = !force_flag_; 
        }

    private:
        std::vector<double> rest_;
        std::vector<double> force_;
        bool force_flag_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
}; 

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EffortExample>());
    rclcpp::shutdown();
    return 0;
}