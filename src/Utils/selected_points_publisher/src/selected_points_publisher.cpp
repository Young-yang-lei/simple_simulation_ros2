#include <QKeyEvent>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v0.1.0: drwnz (david.wong@tier4.jp) *
 *
 * selected_points_publisher.cpp
 *
 *  Created on: December 5th 2019
 */

#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreViewport.h>
static bool getPointOnPlaneFromWindowXY(Ogre::Viewport *viewport, const Ogre::Plane &plane,
                                       int x, int y, Ogre::Vector3 &point) {
  auto ray = viewport->getCamera()->getCameraToViewportRay(
    static_cast<float>(x)/viewport->getActualWidth(), static_cast<float>(y)/viewport->getActualHeight());
  auto hit = ray.intersects(plane);
  if (!hit.first) return false;
  point = ray.getPoint(hit.second); return true;
}

#include "OgreCamera.h"

#include "selected_points_publisher/selected_points_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QVariant>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rviz_plugin_selected_points_publisher
{
  SelectedPointsPublisher::SelectedPointsPublisher()
  {
    selecting_ = false;
    num_selected_points_ = 0;
  }

  SelectedPointsPublisher::~SelectedPointsPublisher()
  {
  }

  void SelectedPointsPublisher::onInitialize() {
    rviz_default_plugins::tools::SelectionTool::onInitialize();
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    updateTopic();
  }

  void SelectedPointsPublisher::updateTopic()
  {
    tf_frame_ = context_->getFixedFrame().toStdString();
    selected_drones_topic_ = std::string("rviz_selected_drones");
    goal_topic_ = std::string("goal");

    rviz_selected_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(selected_drones_topic_.c_str(), 100);
    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_.c_str(), 100);
    num_selected_points_ = 0;
  }

  int SelectedPointsPublisher::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel)
  {
    int flags = rviz_default_plugins::tools::SelectionTool::processKeyEvent(event, panel);
    if (event->type() == QKeyEvent::KeyPress)
    {
      if (event->key() == 'c' || event->key() == 'C')
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning previous selection (selected area "
                                                                          "and points).");
        auto selection_manager = context_->getSelectionManager();
        rviz_common::interaction::M_Picked selection = selection_manager->getSelection();
        selection_manager->select(panel->getRenderWindow(), 0, 0, 0, 0, rviz_common::interaction::SelectionManagerIface::Replace);
        selected_drones_.clear();
        visualization_msgs::msg::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
        marker.header.stamp = node_->now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        num_selected_points_ = 0;
      }
      else if (event->key() == 'p' || event->key() == 'P')
      {
        RCLCPP_INFO_STREAM(node_->get_logger(),
                              "Publishing " << num_selected_points_ << " selected points to topic "
                                            << selected_drones_topic_);
        //
        for (int i = 0; i < selected_drones_.size(); ++i)
        {
          rviz_selected_publisher_->publish(selected_drones_[i]);
        }
      }
    }

    return flags;
  }

  int SelectedPointsPublisher::processMouseEvent(rviz_common::ViewportMouseEvent &event)
  {
    int flags = rviz_default_plugins::tools::SelectionTool::processMouseEvent(event);
    if (event.alt())
    {
      selecting_ = false;
    }
    else
    {
      if (event.leftDown())
      {
        selecting_ = true;
      }
      if (event.rightUp()) // Publish immediately!
      {
        Ogre::Vector3 intersection;
        Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        if (getPointOnPlaneFromWindowXY(rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow()), ground_plane, event.x,
                                              event.y, intersection))
        {
          this->processSelectedArea();
          for (int i = 0; i < selected_drones_.size(); ++i)
          {
            rviz_selected_publisher_->publish(selected_drones_[i]);
          }



          geometry_msgs::msg::PoseStamped goal_msg;
          goal_msg.header.frame_id = context_->getFixedFrame().toStdString();
          goal_msg.header.stamp = node_->now();
          goal_msg.pose.position.x = intersection.x;
          goal_msg.pose.position.y = intersection.y;
          goal_msg.pose.position.z = 1.0;
          goal_msg.pose.orientation.w = 1.0;
          goal_publisher_->publish(goal_msg);
        }
      }
    }

    if (selecting_)
    {
      if (event.leftUp())
      {
        this->processSelectedArea();
      }
    }
    return flags;
  }

  int SelectedPointsPublisher::processSelectedArea()
  {
    auto selection_manager = context_->getSelectionManager();
    rviz_common::interaction::M_Picked selection = selection_manager->getSelection();
    rviz_common::properties::PropertyTreeModel *model = selection_manager->getPropertyModel();

    selected_drones_.clear();
    int i = 0;
    while (model->hasIndex(i, 0))
    {

      QModelIndex child_index = model->index(i, 0);

      rviz_common::properties::Property *child = model->getProp(child_index);
      auto *subchild = dynamic_cast<rviz_common::properties::VectorProperty *>(child->childAt(0));
      if (!subchild) { ++i; continue; }
      Ogre::Vector3 point_data = subchild->getVector();
      // ROS_ERROR("%f %f %f", point_data.x, point_data.y, point_data.z);
      std::string name = child->getNameStd();
      // ROS_ERROR("%s", name.c_str());

      if (name.substr(0, 12) == std::string("Marker drone"))
      {
        geometry_msgs::msg::PoseStamped droneX_msg;
        droneX_msg.header.frame_id = std::string("drone_") + name.substr(13, 20);
        droneX_msg.header.stamp = node_->now();
        droneX_msg.pose.position.x = point_data.x;
        droneX_msg.pose.position.y = point_data.y;
        droneX_msg.pose.position.z = point_data.z;

        selected_drones_.push_back(droneX_msg);
      }

      i++;
    }
    num_selected_points_ = i;
    RCLCPP_INFO_STREAM(node_->get_logger(),
                          "Number of points in the selected area: " << num_selected_points_);

    return 0;
  }
} // namespace rviz_plugin_selected_points_publisher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz_common::Tool)
