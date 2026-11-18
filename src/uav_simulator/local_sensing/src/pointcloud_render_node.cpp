#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <cstring>
#include <limits>

// CPU local cloud sampling and optional pinhole depth projection.
// This is a point-map sensor, not a textured RGB renderer.
class LocalSensing : public rclcpp::Node {
public:
  LocalSensing() : Node("pcl_render_node") {
    range_ = declare_parameter("sensing_horizon", 8.0);
    voxel_ = declare_parameter("voxel_size", 0.1);
    depth_ = declare_parameter("publish_depth", false);
    width_ = declare_parameter("camera.width", 640); height_ = declare_parameter("camera.height", 480);
    fx_ = declare_parameter("camera.fx", 387.229); fy_ = declare_parameter("camera.fy", 387.229);
    cx_ = declare_parameter("camera.cx", 321.046); cy_ = declare_parameter("camera.cy", 243.450);
    const double rate = declare_parameter("sensing_rate", 30.0);
    auto extr = declare_parameter<std::vector<double>>("camera.translation", {0.0, 0.0, 0.0});
    if (range_ <= 0 || voxel_ <= 0 || rate <= 0 || width_ <= 0 || height_ <= 0 || fx_ <= 0 || fy_ <= 0 || extr.size()!=3)
      throw std::invalid_argument("Invalid sensing/camera parameters");
    translation_ = Eigen::Vector3d(extr[0],extr[1],extr[2]);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth", rclcpp::SensorDataQoS());
    camera_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("global_map", rclcpp::QoS(1).transient_local(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { setMap(*msg, global_); map_frame_ = msg->header.frame_id; });
    local_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("local_map", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { setMap(*msg, dynamic_); dynamic_frame_ = msg->header.frame_id; });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(10).best_effort(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = *msg; have_odom_ = true; });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate), [this] { render(); });
  }
private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  void setMap(const sensor_msgs::msg::PointCloud2 &msg, Cloud::Ptr &out) {
    Cloud::Ptr input(new Cloud), filtered(new Cloud);
    pcl::fromROSMsg(msg, *input);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(voxel_,voxel_,voxel_); voxel.setInputCloud(input); voxel.filter(*filtered);
    out = filtered; dirty_ = true;
  }
  void render() {
    if (!have_odom_ || !global_) return;
    if (odom_.header.frame_id != map_frame_ || (dynamic_ && dynamic_frame_ != map_frame_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Map and odometry must use the same frame"); return;
    }
    if (dirty_) {
      combined_.reset(new Cloud(*global_)); if (dynamic_) *combined_ += *dynamic_;
      if (combined_->empty()) return;
      tree_.setInputCloud(combined_); dirty_ = false;
    }
    const auto &p = odom_.pose.pose.position;
    std::vector<int> indices; std::vector<float> distances;
    tree_.radiusSearch(pcl::PointXYZ(p.x,p.y,p.z),range_,indices,distances);
    Cloud local;
    for (auto i: indices) {
      const auto &point=combined_->points[i];
      if (std::abs(point.z-p.z) <= range_*std::tan(M_PI/6)) local.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(local,cloud); cloud.header.frame_id=map_frame_; cloud.header.stamp=now();
    cloud_pub_->publish(cloud);
    if (depth_) renderDepth(local, cloud.header);
  }
  void renderDepth(const Cloud &cloud, const std_msgs::msg::Header &header) {
    const auto &o=odom_.pose.pose.orientation;
    Eigen::Quaterniond q(o.w,o.x,o.y,o.z);
    if (q.norm() < 1e-6) return;
    q.normalize();
    // Optical x right, y down, z forward relative to FLU body axes.
    Eigen::Matrix3d body_from_optical;
    body_from_optical << 0,0,1, -1,0,0, 0,-1,0;
    Eigen::Matrix3d world_from_optical=q.toRotationMatrix()*body_from_optical;
    const auto &p=odom_.pose.pose.position;
    Eigen::Vector3d camera=Eigen::Vector3d(p.x,p.y,p.z)+q*translation_;
    std::vector<float> pixels(width_*height_,std::numeric_limits<float>::infinity());
    for (const auto &point: cloud) {
      Eigen::Vector3d v=world_from_optical.transpose()*(Eigen::Vector3d(point.x,point.y,point.z)-camera);
      if (v.z()<=0.05) continue;
      int u=std::lround(fx_*v.x()/v.z()+cx_), w=std::lround(fy_*v.y()/v.z()+cy_);
      if (u>=0 && u<width_ && w>=0 && w<height_) pixels[w*width_+u]=std::min(pixels[w*width_+u],static_cast<float>(v.z()));
    }
    sensor_msgs::msg::Image image;
    image.header=header; image.header.frame_id=odom_.child_frame_id+"/camera_optical";
    image.height=height_;image.width=width_;image.encoding="32FC1";image.step=width_*sizeof(float);
    image.data.resize(pixels.size()*sizeof(float));std::memcpy(image.data.data(),pixels.data(),image.data.size());
    depth_pub_->publish(image);
    sensor_msgs::msg::CameraInfo info;
    info.header=image.header;info.height=height_;info.width=width_;info.distortion_model="plumb_bob";
    info.k={fx_,0,cx_,0,fy_,cy_,0,0,1};info.r={1,0,0,0,1,0,0,0,1};info.p={fx_,0,cx_,0,0,fy_,cy_,0,0,0,1,0};
    camera_pub_->publish(info);
    geometry_msgs::msg::PoseStamped pose;
    pose.header=header;pose.pose.position.x=camera.x();pose.pose.position.y=camera.y();pose.pose.position.z=camera.z();
    Eigen::Quaterniond orientation(world_from_optical);
    pose.pose.orientation.w=orientation.w();pose.pose.orientation.x=orientation.x();pose.pose.orientation.y=orientation.y();pose.pose.orientation.z=orientation.z();
    pose_pub_->publish(pose);
  }
  double range_,voxel_,fx_,fy_,cx_,cy_;int width_,height_;bool depth_,have_odom_=false,dirty_=true;
  Eigen::Vector3d translation_;
  std::string map_frame_,dynamic_frame_;
  Cloud::Ptr global_,dynamic_,combined_;
  pcl::search::KdTree<pcl::PointXYZ> tree_;
  nav_msgs::msg::Odometry odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_,local_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LocalSensing>());rclcpp::shutdown();
}
