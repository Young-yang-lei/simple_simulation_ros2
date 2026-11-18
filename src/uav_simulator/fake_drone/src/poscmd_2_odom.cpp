#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <Eigen/Geometry>

// Ideal reference tracking backend. No vehicle dynamics or tracking error.
class FakeDrone : public rclcpp::Node {
public:
  FakeDrone() : Node("poscmd_2_odom") {
    odom_.header.frame_id = declare_parameter("world_frame", "world");
    odom_.child_frame_id = declare_parameter("body_frame", "base_link");
    odom_.pose.pose.position.x = declare_parameter("init_x", 0.0);
    odom_.pose.pose.position.y = declare_parameter("init_y", 0.0);
    odom_.pose.pose.position.z = declare_parameter("init_z", 1.0);
    odom_.pose.pose.orientation.w = 1.0;
    const double rate = declare_parameter("publish_rate", 100.0);
    if (rate <= 0) throw std::invalid_argument("publish_rate must be positive");
    pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>("command", 10,
      [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd) {
        const auto &p = cmd->position;
        Eigen::Vector3d a(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z + 9.81);
        if (!a.allFinite() || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(cmd->yaw)) return;
        odom_.pose.pose.position = p;
        odom_.twist.twist.linear = cmd->velocity;
        Eigen::Vector3d z = a.norm() > 1e-6 ? a.normalized().eval() : Eigen::Vector3d::UnitZ().eval();
        Eigen::Vector3d y = z.cross(Eigen::Vector3d(std::cos(cmd->yaw), std::sin(cmd->yaw), 0));
        if (y.norm() < 1e-6) y = z.unitOrthogonal();
        y.normalize();
        Eigen::Matrix3d rot;
        rot.col(0) = y.cross(z); rot.col(1) = y; rot.col(2) = z;
        Eigen::Quaterniond q(rot);
        auto &o = odom_.pose.pose.orientation;
        o.w = q.w(); o.x = q.x(); o.y = q.y(); o.z = q.z();
        // Reference jerk determines the body-axis rates for this flatness model.
        Eigen::Vector3d jerk(cmd->jerk.x, cmd->jerk.y, cmd->jerk.z);
        auto &omega = odom_.twist.twist.angular;
        omega.x = a.norm() > 1e-6 ? -y.dot(jerk) / a.norm() : 0.0;
        omega.y = a.norm() > 1e-6 ? rot.col(0).dot(jerk) / a.norm() : 0.0;
        omega.z = cmd->yaw_dot * z.z();
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate), [this] {
      odom_.header.stamp = now(); pub_->publish(odom_);
    });
  }
private:
  nav_msgs::msg::Odometry odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<FakeDrone>()); rclcpp::shutdown();
}
