# RViz2 selected drones tool

Build this package with colcon on ROS2 Humble. In RViz2, open Panels → Tools,
then add `rviz_plugin_selected_points_publisher/SelectedPointsPublisher`.

- Drag with the left button to select visible objects; Alt moves the viewpoint.
- Shift adds to the selection; Ctrl removes from it; C clears it.
- P publishes the selected drone poses on `rviz_selected_drones`.
- Right-click release publishes the selection and a `goal` pose at the clicked
  ground-plane x/y position, with target altitude 1 m.

Both topics use `geometry_msgs/msg/PoseStamped` and are relative to the RViz
namespace. Selection is specific to drone markers named `Marker drone...`;
this is not a generic point-cloud extraction tool. Selection messages retain
the upstream convention of carrying `drone_<id>` in `header.frame_id` for
`assign_goals`; this field is an identifier in that interface, not a TF frame.

The plugin library has a loading/construction test. Full mouse interaction
still requires verification in a desktop RViz2 session.
