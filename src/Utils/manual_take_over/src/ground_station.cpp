#include <cassert>
#include <rclcpp/rclcpp.hpp>
static rclcpp::Node::SharedPtr node;
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/msg/joy.hpp>


using namespace std;
bool flag_mandatory_stoped_ = false;

rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

void joy_sub_cb(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
{

  if ( msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3] )
  {
    flag_mandatory_stoped_ = true;
  }

  if ( flag_mandatory_stoped_ )
  {
    joy_pub_->publish(*msg);
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("manual_take_over_ground_station");


  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_sub_cb);
  joy_pub_ = node->create_publisher<sensor_msgs::msg::Joy>("joystick_from_users", 10);

  rclcpp::spin(node);


  joy_pub_.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
