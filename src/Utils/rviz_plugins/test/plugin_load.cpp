#include <QApplication>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/tool.hpp>
#include <iostream>

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  pluginlib::ClassLoader<rviz_common::Tool> loader("rviz_common", "rviz_common::Tool");
  for (const auto &name : {"rviz_plugins/Goal3DTool", "rviz_plugin_selected_points_publisher/SelectedPointsPublisher"}) {
    auto tool = loader.createSharedInstance(name);
    if (!tool) return 1;
    std::cout << "Loaded " << name << std::endl;
  }
  return 0;
}
