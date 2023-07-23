# Launch file to start two sick safety_scanner nodes
- `output="screen"`. This attribute determines where the node's output (e.g., log messages) gets sent. When output is set to "screen", the node's output will be printed to the console. If output is set to "log", the output will be written to a log file.

- `ns="sick_safetyscanners"`: This attribute sets the namespace for the node. In ROS, a namespace is a prefix that is added to the names of resources (like topics, services, and parameters) that a node uses. This can be used to avoid name conflicts when multiple instances of a node are running, or to group related nodes together. In this case, the ns attribute is setting the namespace to "sick_safetyscanners", which means that all of the resources for this node will have "sick_safetyscanners" added to the beginning of their names. For example, if this node publishes a topic called "scan", the actual topic name will be "/sick_safetyscanners/scan"

# How to start the configured launch file
`roslaunch sick_sub start_two_laser_scanners.launch`

# [Diagnostics](http://wiki.ros.org/diagnostics)
For robots without a `diagnostic_aggregator`, the [`rqt_runtime_monitor`](http://wiki.ros.org/rqt_runtime_monitor) package contains a simple monitor that displays data from the `/diagnostics` topic.
- `rosrun rqt_runtime_monitor rqt_runtime_monitor`
- The `/diagnostics` topic in ROS uses the `DiagnosticArray` message type from the [`diagnostic_msgs/DiagnosticArray`](http://docs.ros.org/en/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html) package. This is a complex message type that can carry a wide range of diagnostic information.