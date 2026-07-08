# glog_component

This package provides the glog (google logging library) feature as a ROS 2 component library.
Autoware launch files load it into component containers that need glog initialized.

See the [glog GitHub repository](https://github.com/google/glog) for details of its features.

## Example

```py
glog_component = ComposableNode(
    package="autoware_glog_component",
    plugin="autoware::glog_component::GlogComponent",
    name="glog_component",
)

container = ComposableNodeContainer(
    name="my_container",
    namespace="",
    package="rclcpp_components",
    executable=LaunchConfiguration("container_executable"),
    composable_node_descriptions=[
        component1,
        component2,
        glog_component,
    ],
)
```
