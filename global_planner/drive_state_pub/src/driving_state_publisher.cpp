#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class DrivingStatePublisher : public rclcpp::Node {
public:
    DrivingStatePublisher() : Node("driving_state_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("driving_state", 10);
    merged_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("merge_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&DrivingStatePublisher::joyCallback, this, std::placeholders::_1));
    tele_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/joy_cmd_vel", 10, std::bind(&DrivingStatePublisher::teleop_callback, this, std::placeholders::_1));
    planner_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DrivingStatePublisher::planner_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        500ms, std::bind(&DrivingStatePublisher::publishMessage, this));
    
    }

private:
    void teleop_callback(geometry_msgs::msg::Twist::SharedPtr tele_vel) {
        //std::cout << " teleop call bacl " << count_ << std::endl;
        if (count_ == 0) {
            merged_pub_->publish(*tele_vel);
        }
    }

    void planner_callback(geometry_msgs::msg::Twist::SharedPtr plan_vel) {
           //     std::cout << " planner call bacl " << count_ << std::endl;

        if (count_ == 1) {
            merged_pub_->publish(*plan_vel);
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        size_t button = 0; // map this to the button you want here: https://index.ros.org/p/joy/ 
                //  1 is triangle

        if (msg->buttons.size() > button)  { 
            
            if(!sticky &&  msg->buttons[button] ==1 ){
                sticky = true;
                if(count_ == 1)
                    count_ = 0;
                else if(count_ == 0)
                    count_ = 1;
            }

           if(sticky && msg->buttons[button] ==0 ){
                sticky = false;
            }

       RCLCPP_INFO(this->get_logger(), "Received joy button value: %d", count_);
        }
        else {
        RCLCPP_WARN(this->get_logger(), "Joy message not found");
            count_ = 1;
        }


    }

    void publishMessage() {
        auto message = std_msgs::msg::Int32();
        message.data = count_;
      //  RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
        publisher_->publish(message);
      
    }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planner_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tele_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int count_;
  bool sticky = false; //used as a latch (need in case the operator holds down the button)
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrivingStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
