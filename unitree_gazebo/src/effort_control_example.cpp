#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

class EffortExample: public rclcpp::Node{
    public:
        EffortExample() : Node("effort_control_example") 
        {
            pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_effort_controller/commands", 10);   
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EffortExample::timer_callback, this));
            force_flag_ = true;
            rest_ = std::vector<double>(6, 0.0);
            force_ = std::vector<double>(6, 10.0);
            msg_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
        }

        void timer_callback(){
            if (force_flag_) msg_->data = force_;
            else msg_->data = rest_;

            pub_->publish(*msg_);
            force_flag_ = !force_flag_; 
        }

    private:
        std::vector<double> rest_;
        std::vector<double> force_;
        std::shared_ptr<std_msgs::msg::Float64MultiArray> msg_;
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