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
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <uav_utils/geometry_utils.h>

using namespace std;

// ros::Subscriber trajs_sub_;
rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goals_pub_;
struct Drone_Info_t
{
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  int goal_id{-1};
  Eigen::Vector3d cur_p{Eigen::Vector3d::Zero()};
  double cur_yaw{0.0};
  Eigen::Vector3d last_p{Eigen::Vector3d::Zero()};
  rclcpp::Time arrived_time{0, 0, RCL_ROS_TIME};
  bool arrived_for_a_while{true};
  bool odom_received{false};
};
struct Goal_t
{
  Eigen::Vector3d p;
  bool occupied;
};
std::vector<Drone_Info_t> drones_;

void set_odom_data(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const int &drone_id)
{
  drones_[drone_id].odom_received = true;
  drones_[drone_id].last_p = drones_[drone_id].cur_p;
  drones_[drone_id].cur_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  drones_[drone_id].cur_yaw = uav_utils::get_yaw_from_quaternion(Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z));
}

void odoms_sim_sub_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const int &drone_id)
{
  set_odom_data(msg, drone_id);
}

void combined_odoms_sim_sub_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  if (msg->child_frame_id.size() <= 6 || msg->child_frame_id.compare(0, 6, "drone_") != 0) return;
  int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
  if ( msg->child_frame_id.substr(0, 6) != string("drone_") || id < 0 || id >= (int)drones_.size())
  {
    RCLCPP_ERROR(node->get_logger(), "[random_goals_node] Wrong child_frame_id: %s, or wrong drone_id: %d", msg->child_frame_id.substr(0, 6).c_str(), id);
    return;
  }
  set_odom_data(msg, id);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("random_goals");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);


  srand(floor(node->now().seconds() * 10));

  int drone_num, goal_num;
  drone_num = node->declare_parameter("drone_num", 1);
  goal_num = node->declare_parameter("goal_num", 1);

  if (drone_num <= 0 || goal_num <= 0) throw std::invalid_argument("drone_num and goal_num must be positive");
  vector<Goal_t> goals(goal_num);
  for (int i = 0; i < goal_num; ++i)
  {
    vector<double> pt;
    pt = node->declare_parameter<std::vector<double>>("goal" + to_string(i), {0.0, 0.0, 1.0});
    if (pt.size() != 3) throw std::invalid_argument("goal must have 3 coordinates");
    goals[i].p << pt[0], pt[1], pt[2];
    goals[i].occupied = false;
  }

  drones_.resize(drone_num);
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odoms_sim_sub(drone_num);
  for (int i = 0; i < drone_num; ++i)
  {
    odoms_sim_sub[i] = node->create_subscription<nav_msgs::msg::Odometry>("drone_" + to_string(i) + "/odometry", 10, [i](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odoms_sim_sub_cb(msg, i); });
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr combined_odoms_sim_sub;
  combined_odoms_sim_sub = node->create_subscription<nav_msgs::msg::Odometry>("others_odom", 1000, combined_odoms_sim_sub_cb);

  goals_pub_ = node->create_publisher<quadrotor_msgs::msg::GoalSet>("goal_user2brig", 10);

  std::vector<int> count(goal_num);
  fill(count.begin(), count.end(), 0);

  while (rclcpp::ok())
  {
    rclcpp::Time t_now = node->now();

    for (int i = 0; i < drone_num; ++i)
    {
      if (!drones_[i].odom_received || drones_[i].goal_id < 0) continue;
      double d_to_goal = (drones_[i].cur_p - drones_[i].goal).norm();
      double last_d_to_goal = (drones_[i].last_p - drones_[i].goal).norm();
      if (d_to_goal > 0.1 || last_d_to_goal > 0.1)
        drones_[i].arrived_time = t_now;
      drones_[i].arrived_for_a_while |= ((t_now - drones_[i].arrived_time).seconds() > 2);
    }

    int drone_trials = 0;
    while (drone_trials < drone_num)
    {
      int d_id = rand() % drone_num;
      if (drones_[d_id].odom_received && drones_[d_id].arrived_for_a_while)
      {
        int goal_trials = 0;
        while (goal_trials < goal_num)
        {
          int g_id = rand() % goal_num;
          double ang = acos(((goals[g_id].p - drones_[d_id].cur_p).normalized()).dot((Eigen::Vector3d(cos(drones_[d_id].cur_yaw), sin(drones_[d_id].cur_yaw), 0)).normalized()));
          if (!goals[g_id].occupied && ang > 0 && ang < M_PI / 6)
          {
            if (drones_[d_id].goal_id >= 0)
              goals[drones_[d_id].goal_id].occupied = false;
            goals[g_id].occupied = true;
            drones_[d_id].arrived_for_a_while = false;
            drones_[d_id].goal = goals[g_id].p;
            drones_[d_id].goal_id = g_id;
            quadrotor_msgs::msg::GoalSet msg;
            msg.drone_id = d_id;
            msg.goal[0] = drones_[d_id].goal(0);
            msg.goal[1] = drones_[d_id].goal(1);
            msg.goal[2] = drones_[d_id].goal(2);
            goals_pub_->publish(msg);
            cout << "drone_id=" << d_id << " goal=" << drones_[d_id].goal.transpose() << endl;
            goto multi_loop;
          }

          goal_trials++;
        }
      }

      drone_trials++;
    }

  multi_loop:;

    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1)));
    if (rclcpp::ok()) executor.spin_some();
  }


  goals_pub_.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
