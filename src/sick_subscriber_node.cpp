#include <sstream>      // std::ostringstream

// Include the necessary header files
#include <ros/ros.h>
// ROS_INFO_STREAM is defined in this file
// http://wiki.ros.org/roscpp/Overview/Logging
// https://wiki.ros.org/rosconsole
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>

// Function to convert LaserScan message to a string
std::string LaserScanToString(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::ostringstream os;
    os << "---\n";
    os << "header:\n";
    os << "  seq: " << msg->header.seq << "\n";
    os << "  stamp:\n";
    os << "    sec: " << msg->header.stamp.sec << "\n";
    os << "    nsec: " << msg->header.stamp.nsec << "\n";
    os << "    frame_id: " << msg->header.frame_id << "\n";
    os << "angle_min: " << msg->angle_min << "\n";
    os << "angle_max: " << msg->angle_max << "\n";
    os << "angle_increment: " << msg->angle_increment << "\n";
    os << "time_increment: " << msg->time_increment << "\n";
    os << "scan_time: " << msg->scan_time << "\n";
    os << "range_min: " << msg->range_min << "\n";
    os << "range_max: " << msg->range_max << "\n";
    os << "ranges: [";
    for (const auto& range : msg->ranges)
    {
        os << range << ", ";
    }
    os << "]\n";
    os << "intensities: [";
    for (const auto& intensity : msg->intensities)
    {
        os << intensity << ", ";
    }
    os << "]\n";

    return os.str();
}

/*
1. Efficiency: When you pass a const sensor_msgs::LaserScan object to a function
the entire object is copied, which can be expensive in terms of time and memory for large objects.
When you pass a reference to a shared pointer (ConstPtr&), only the pointer is copied, not the object it points to. 
This is much more efficient, especially when dealing with large point clouds or other sensor data.

2. Safety: The ConstPtr type is a pointer to a const object, which means you can't modify the object through this pointer.
This is good practice when writing callback functions, because it ensures that you can't accidentally modify the sensor data.

3. Memory Management: The ConstPtr type is a type of smart pointer that automatically manages the memory 
of the object it points to. When there are no more ConstPtrs pointing to an object, the object is automatically deleted.
This helps prevent memory leaks.
*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::string scan_data = LaserScanToString(msg);
    /* Returns a pointer to an array that contains a null-terminated sequence of characters
    const char* c_str() const;
    ROS_INFO provides a macro-based interface which allows printf-style output. */
    ROS_INFO("Received scan data: %s", scan_data.c_str());
    
    /* ROS_INFO_STREAM provides a macro-based interface which allows stream-style output.
    ROS_INFO_STREAM implements ostream operator (operator<<) for each message type defined by ros.*/
    ROS_INFO_STREAM("Received scan data" << *msg);

    /* LaserScan message can be visualised in Rviz directly. Or it can be transformed into sensor_msgs/PointCloud
    or sensor_msgs/PointCloud2 using this ros package https://github.com/ros-perception/laser_geometry */
}

int main(int argc, char **argv)
{
  // Initialize the ROS node
  // The ros::init() function needs to see argc and argv so that it can perform
  // any ROS arguments and name remapping that were provided at the command line.
  // You must call one of the versions of ros::init() before using any other
  // part of the ROS system
  ros::init(argc, argv, "sick_subscriber");

  // Create a ros nodehandle
  // NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle n;

  // Create a subscriber object
  // 1. The first parameter is the topic name will be subscribed
  // 2. The second parameter to the subscribe() function is the size of the message queue. If messages
  // are arriving faster than they are being processed, this is the number of messages that will be
  // buffered up before beginning to throw away the oldest ones.
  // 3. The third parameter is the callback function, Received messages are passed to a callback function,
  // here called scanCallback.
  ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);

  // Enter a loop, pumping callbacks.
  // callback function will be called from within this thread (the main one).
  ros::spin();

  return 0;
}
