#include <cassert>
#include <rclcpp/rclcpp.hpp>
static rclcpp::Node::SharedPtr node;
#include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
// #include <pcl/search/impl/flann_search.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include <random>
#include <pcl/search/impl/search.hpp>



using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
// default_random_engine eng(4);
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _local_map_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_static_obs_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_wall_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _obj_cloud_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr click_map_pub_;


vector<double> _state;

int _obs_num, _moving_obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _pub_rate;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_theta_tilt_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::msg::PointCloud2 globalMap_pcd;
sensor_msgs::msg::PointCloud2 globalMap_static_obs_pcd;
sensor_msgs::msg::PointCloud2 globalMap_wall_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::PointCloud<pcl::PointXYZ> cloudMap_static_obs;
pcl::PointCloud<pcl::PointXYZ> cloudMap_wall;

sensor_msgs::msg::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

vector<Eigen::Vector2d> obj_pos;
vector<Eigen::Vector2d> obj_vel;
vector<double> obj_vel_per_x;
vector<double> obj_vel_amp_x;
vector<double> obj_vel_per_y;
vector<double> obj_vel_amp_y;

uniform_real_distribution<double> rand_vel_per_x_;
uniform_real_distribution<double> rand_vel_per_y_;
uniform_real_distribution<double> rand_vel_amp_x_;
uniform_real_distribution<double> rand_vel_amp_y_;

uniform_real_distribution<double> rand_const_vx_;
uniform_real_distribution<double> rand_const_vy_;

vector<pcl::PointCloud<pcl::PointXYZ>> obj_clusters;

void ObjUpdate(double t, double dt)
{
  // cout<<"start"<<endl;
  for (int i = 0; i < _moving_obs_num; i++)
  {
    // pos += vel;

    // cout<<pos<<endl;

    float vx = obj_vel_amp_x[i] * sin(obj_vel_per_x[i] * t) + 1.1 * obj_vel_amp_x[i];
    float vy = obj_vel_amp_y[i] * sin(obj_vel_per_y[i] * t) + 1.1 * obj_vel_amp_y[i];
    // float vx = obj_vel_amp_x[i] * sin(obj_vel_per_x[i] * t) + 1.1 * obj_vel_amp_x[i];
    // float vy = obj_vel_amp_y[i] * sin(obj_vel_per_y[i] * t) + 1.1 * obj_vel_amp_x[i];
    // vy = 0;

    obj_pos[i](0) += vx * dt;
    obj_pos[i](1) += vy * dt;

    for (int j = 0; j < obj_clusters[i].points.size(); ++j)
    {
      obj_clusters[i].points[j].x += vx * dt;
      obj_clusters[i].points[j].y += vy * dt;
    }

    if (obj_pos[i](0) < _x_l || obj_pos[i](0) > _x_h || obj_pos[i](1) < _y_l || obj_pos[i](1) > _y_h)
    {
      obj_vel_amp_x[i] = -obj_vel_amp_x[i];
      obj_vel_amp_y[i] = -obj_vel_amp_y[i];
    }
  }

  // for (int i = 0; i < 5; i++)
  // {
  //   Eigen::Vector2d pos = obj_pos[i];
  //   float vx, vy;
  //   vy = 0;
  //   vx = obj_vel[i][0];

  //   obj_pos[i](0) += vx * dt;
  //   obj_pos[i](1) += vy * dt;

  //   for (int j = 0; j < obj_clusters[i].points.size(); ++j)
  //   {
  //     obj_clusters[i].points[j].x += vx * dt;
  //     obj_clusters[i].points[j].y += vy * dt;
  //   }

  //   if (obj_pos[i](0) < _x_l || obj_pos[i](0) > _x_h || obj_pos[i](1) < _y_l || obj_pos[i](1) > _y_h)
  //   {
  //     obj_vel[i][0] = -obj_vel[i][0];
  //   }
  // }

  pcl::PointCloud<pcl::PointXYZ> obj_points;
  for (int i = 0; i < obj_clusters.size(); i++)
  {
    obj_points += obj_clusters[i];
  }

  // obj_points += cloudMap;

  sensor_msgs::msg::PointCloud2 objCloud_pcd;
  pcl::toROSMsg(obj_points, objCloud_pcd);
  objCloud_pcd.header.frame_id = "world";
  objCloud_pcd.header.stamp = node->now();
  _obj_cloud_pub->publish(objCloud_pcd);
}

void RandomObjGenerate()
{
  pcl::PointXYZ pt_random;

  // generate moving obs
  for (int i = 0; i < _moving_obs_num; i++)
  {
    pcl::PointCloud<pcl::PointXYZ> cluster;

    rand_vel_per_x_ = uniform_real_distribution<double>(0.0, 2.0);
    rand_vel_per_y_ = uniform_real_distribution<double>(0.0, 2.0);
    rand_vel_amp_x_ = uniform_real_distribution<double>(0.2, 2.0);
    rand_vel_amp_y_ = uniform_real_distribution<double>(0.2, 2.0);

    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    h = rand_h(eng);
    inf = rand_inf(eng);

    double vel_per_x, vel_per_y, vel_amp_x, vel_amp_y;
    vel_per_x = rand_vel_per_x_(eng);
    vel_per_y = rand_vel_per_y_(eng);
    vel_amp_x = rand_vel_amp_x_(eng);
    vel_amp_y = rand_vel_amp_y_(eng);
    Eigen::Vector2d pos(x, y);
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    // cout<<x<<endl;
    // cout<<y<<endl;
    // Eigen::Vector2d pos(x, y);
    Eigen::Vector2d vel(0, 0);
    obj_pos.push_back(pos);
    obj_vel.push_back(vel);
    obj_vel_per_x.push_back(vel_per_x);
    obj_vel_amp_x.push_back(vel_amp_x);
    obj_vel_per_y.push_back(vel_per_y);
    obj_vel_amp_y.push_back(vel_amp_y);

    int widNum = ceil((w * inf) / (2.0 * _resolution));
    double radius = (w * inf) / 2;
    cout<<widNum<<endl;
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        int heiNum = ceil(h / (1 * _resolution));
        for (int t = -10; t < heiNum; t++)
        {
          if((r <= 0.5-(widNum / 2.0)) || (r >= -1+(widNum / 2.0)) || (s <= 0.5-(widNum / 2.0)) || (s >= -1+(widNum / 2.0)))
          {
            double temp_x = x + (r + 0.5) * (1.0 * _resolution) + 1e-2;
            double temp_y = y + (s + 0.5) * (1.0 * _resolution) + 1e-2;
            double temp_z = (t + 0.5) * (1.0 * _resolution) + 1e-2;
            if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
            {
              pt_random.x = temp_x;
              pt_random.y = temp_y;
              pt_random.z = temp_z;
              cluster.push_back(pt_random);
            }
          }
        }
      }
      // cout<<cluster.size()<<endl;
    obj_clusters.push_back(cluster);
  }
  // cout<<obj_clusters.size()<<endl;

  // generate const v objects
  // for (int i = 0; i < 5; i++)
  // {
  //   pcl::PointCloud<pcl::PointXYZ> cluster;
  //   rand_const_vx_ = uniform_real_distribution<double>(0.0, 2.0);
  //   rand_const_vy_ = uniform_real_distribution<double>(0.0, 2.0);

  //   double x, y, w, h, inf;
  //   x = 0.0;
  //   y = -1.2 + 0.5 * i;
  //   w = 0.3;
  //   h = 2.5;
  //   inf = 1.0;

  //   Eigen::Vector2d pos(x, y);
  //   x = floor(x / _resolution) * _resolution + _resolution / 1.5;
  //   y = floor(y / _resolution) * _resolution + _resolution / 1.5;

  //   // Eigen::Vector2d pos(x, y);
  //   Eigen::Vector2d vel(-0.6, 0);
  //   obj_pos.push_back(pos);
  //   obj_vel.push_back(vel);
  //   int widNum = ceil((w * inf) / (1.5 * _resolution));
  //   double radius = (w * inf) / 2;

  //   for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
  //     for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
  //     {
  //       int heiNum = ceil(h / (3 * _resolution));
  //       for (int t = -10; t < heiNum; t++)
  //       {
  //         double temp_x = x + (r + 0.5) * (1.5 * _resolution) + 1e-2;
  //         double temp_y = y + (s + 0.5) * (1.5 * _resolution) + 1e-2;
  //         double temp_z = (t + 0.5) * (3 * _resolution) + 1e-2;
  //         if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
  //         {
  //           pt_random.x = temp_x;
  //           pt_random.y = temp_y;
  //           pt_random.z = temp_z;
  //           cluster.push_back(pt_random);
  //         }
  //       }
  //     }

  //   obj_clusters.push_back(cluster);
  // }

}

void CorridorGenerate()
{
  pcl::PointXYZ pt_random;
  double wall_resolution = _resolution * 2;
  for (int i = 0; i < _x_size/wall_resolution; ++i)
  {
    for (int j = 0; j < _z_size/wall_resolution; ++j)
    {
      pt_random.x = -(_x_size/2) + wall_resolution * i;
      pt_random.y = -(_y_size/2);
      pt_random.z = -1.0 + wall_resolution * j;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }

  for (int i = 0; i < _x_size/wall_resolution; ++i)
  {
    for (int j = 0; j < _z_size/wall_resolution; ++j)
    {
      pt_random.x = -(_x_size/2) + wall_resolution * i;
      pt_random.y = _y_size/2;
      pt_random.z = -1.0 + wall_resolution * j;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }
  for (int i = 0; i < _y_size/wall_resolution; ++i)
  {
    for (int j = 0; j < _z_size/wall_resolution; ++j)
    {
      pt_random.y = -(_y_size/2) + wall_resolution * i;
      pt_random.x = _x_size/2;
      pt_random.z = -1.0 + wall_resolution * j;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }
  for (int i = 0; i < _y_size/wall_resolution; ++i)
  {
    for (int j = 0; j < _z_size/wall_resolution; ++j)
    {
      pt_random.y = -(_y_size/2) + wall_resolution * i;
      pt_random.x = -(_x_size/2);
      pt_random.z = -1.0 + wall_resolution * j;
      cloudMap.points.push_back(pt_random);
      cloudMap_wall.push_back(pt_random);
    }
  }

  // for (int i = 0; i < 200; ++i)
  // {
  //   for (int j = 0; j < 40; ++j)
  //   {
  //     pt_random.y = -10.0 + 0.1 * i;
  //     pt_random.x = -10;
  //     pt_random.z = -2.0 + 0.1 * j;
  //     cloudMap.points.push_back(pt_random);
  //   }
  // }

  // for (int i = 0; i < 200; ++i)
  // {
  //   for (int j = 0; j < 30; ++j)
  //   {
  //     pt_random.y = -10.0 + 0.1 * i;
  //     pt_random.x = 10;
  //     pt_random.z = -1.0 + 0.1 * j;
  //     cloudMap.points.push_back(pt_random);
  //   }
  // }
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  cloudMap_static_obs.width = cloudMap_static_obs.points.size();
  cloudMap_static_obs.height = 1;
  cloudMap_static_obs.is_dense = true;

  cloudMap_wall.width = cloudMap_wall.points.size();
  cloudMap_wall.height = 1;
  cloudMap_wall.is_dense = true;
}

void RandomMapGenerateCylinder()
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num && rclcpp::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w * inf) / _resolution);
    double radius = (w * inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -10; t < heiNum; t++)
        {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          float point_rad = (Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm();
          if ( point_rad<= radius && point_rad>=radius-(_resolution*1.2))
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
            cloudMap_static_obs.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // for (double x = -1.5; x <= 1.5; x += 0.1)
  //   for (double y = -1.5; y <= 1.5; y += 0.1)
  //     for (double z = -0.5; z <= 3.0; z += 0.1)
  //     {
  //       pt_random.x = x;
  //       pt_random.y = y;
  //       pt_random.z = z;
  //       cloudMap.push_back(pt_random);
  //     }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  RCLCPP_WARN(node->get_logger(), "Finished generate random map ");
  cout<<"point num:"<<cloudMap.points.size()<<endl;

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomInclinedColumn()
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.2, 0.5);

  rand_theta_ = uniform_real_distribution<double>(-3.14, 3.14);
  rand_theta_tilt_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate inclined column obs
  for (int i = 0; i < _obs_num && rclcpp::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    h = rand_h(eng);
    double rand_theta = rand_theta_(eng);
    double rand_theta_tilt = rand_theta_tilt_(eng);

    int widNum = ceil((w * inf) / _resolution);
    // double radius = (w * inf) / 2;
    Eigen::AngleAxisd R_v(rand_theta_tilt, Eigen::Vector3d(cos(rand_theta),sin(rand_theta),0));
    Eigen::Matrix3d R_m = R_v.matrix();

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        int heiNum = ceil(h / _resolution);
        for (int t = -10; t < heiNum; t++)
        {
          Eigen::Vector3d temp_p(x + (r + 0.5) * _resolution + 1e-2, y + (s + 0.5) * _resolution + 1e-2, (t + 0.5) * _resolution + 1e-2);
          temp_p = R_m * (temp_p - Eigen::Vector3d(x,y,0)) + Eigen::Vector3d(x,y,0);

          pt_random.x = temp_p(0);
          pt_random.y = temp_p(1);
          pt_random.z = temp_p(2);
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  RCLCPP_WARN(node->get_logger(), "Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void clickCallback(const geometry_msgs::msg::PoseStamped &msg)
{
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / (_resolution*2));

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
    {
      h = rand_h(eng);
      int heiNum = ceil(h / (_resolution*2));
      for (int t = -1; t < heiNum; t++)
      {
        pt_random.x = x + (r + 0.5) * (_resolution*2) + 1e-2;
        pt_random.y = y + (s + 0.5) * (_resolution*2) + 1e-2;
        pt_random.z = (t + 0.5) * (_resolution*2) + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_->publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int i = 0;
void pubPoints()
{
  if (!_map_ok || !rclcpp::ok()) return;

  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  globalMap_pcd.header.stamp = node->now();
  _all_map_pub->publish(globalMap_pcd);

  pcl::toROSMsg(cloudMap_static_obs, globalMap_static_obs_pcd);
  globalMap_static_obs_pcd.header.frame_id = "world";
  globalMap_static_obs_pcd.header.stamp = node->now();
  _all_map_static_obs_pub->publish(globalMap_static_obs_pcd);

  pcl::toROSMsg(cloudMap_wall, globalMap_wall_pcd);
  globalMap_wall_pcd.header.frame_id = "world";
  _all_map_wall_pub->publish(globalMap_wall_pcd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("random_map_sensing");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);


  _local_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("map_generator/local_cloud", 1);
  _all_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("map_generator/global_cloud", rclcpp::QoS(1).transient_local());
  _all_map_static_obs_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("map_generator/static_obs_cloud", rclcpp::QoS(1).transient_local());
  _all_map_wall_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("map_generator/wall_cloud", 1);

  click_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/local_map", 1);

  _obj_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("map_generator/obj_cloud", 1);

  _x_size = node->declare_parameter("map.x_size", 50.0);
  _y_size = node->declare_parameter("map.y_size", 50.0);
  _z_size = node->declare_parameter("map.z_size", 5.0);
  _obs_num = node->declare_parameter("map.obs_num", 20);
  _resolution = node->declare_parameter("map.resolution", 0.2);
  circle_num_ = node->declare_parameter("map.circle_num", 10);
  _moving_obs_num = node->declare_parameter("map.moving_obs_num", 40);

  _w_l = node->declare_parameter("ObstacleShape.lower_rad", 0.3);
  _w_h = node->declare_parameter("ObstacleShape.upper_rad", 0.8);
  _h_l = node->declare_parameter("ObstacleShape.lower_hei", 3.0);
  _h_h = node->declare_parameter("ObstacleShape.upper_hei", 7.0);

  radius_l_ = node->declare_parameter("ObstacleShape.radius_l", 7.0);
  radius_h_ = node->declare_parameter("ObstacleShape.radius_h", 7.0);
  z_l_ = node->declare_parameter("ObstacleShape.z_l", 7.0);
  z_h_ = node->declare_parameter("ObstacleShape.z_h", 7.0);
  theta_ = node->declare_parameter("ObstacleShape.theta", 7.0);

  _pub_rate = node->declare_parameter("pub_rate", 50.0);
  _min_dist = node->declare_parameter("min_distance", 1.0);

  if (_pub_rate <= 0 || _resolution <= 0 || _x_size <= 0 || _y_size <= 0 || _z_size <= 0 || _obs_num < 0 || _moving_obs_num < 0)
    throw std::invalid_argument("Invalid map size, rate, resolution or obstacle count");
  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));

  // unsigned int seed = rd();
  unsigned int seed = static_cast<unsigned int>(node->declare_parameter<int64_t>("seed", 3728542744LL));
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  // RandomMapGenerate();
  RandomMapGenerateCylinder();
  CorridorGenerate();
  RandomObjGenerate();
  // RandomInclinedColumn();

  rclcpp::WallRate loop_rate(_pub_rate);
  rclcpp::Time start_time = node->now();
  auto previous_time = start_time;

  while (rclcpp::ok())
  {
    rclcpp::Time cur_time = node->now();

    const double dt = std::max(0.0, (cur_time - previous_time).seconds());
    previous_time = cur_time;
    ObjUpdate((cur_time - start_time).seconds(), dt);
    pubPoints();
    if (rclcpp::ok()) executor.spin_some();
    loop_rate.sleep();
  }

  _local_map_pub.reset();
  _all_map_pub.reset();
  _all_map_static_obs_pub.reset();
  _all_map_wall_pub.reset();
  _obj_cloud_pub.reset();
  click_map_pub_.reset();
  node.reset();
  rclcpp::shutdown();
}
