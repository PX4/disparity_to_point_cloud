#ifndef GLOBAL_PLANNER_ROS2_H_
#define GLOBAL_PLANNER_ROS2_H_

#include <chrono>
#include <math.h>   // sqrt
#include <string>
#include <stdarg.h> // va_list va_start vprintf va_end
#include <stdexcept>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/buffer.h> 
#include <tf2_ros/buffer_interface.h>

#include <tf2/transform_datatypes.h>  // getYaw
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <disparity_to_point_cloud/ros2_additional_fix.h>



/*
Ros2 functionality
*/

// Ros2 added an msg namespace
namespace geometry_msgs { using namespace msg; }
namespace nav_msgs { using namespace msg; }
namespace sensor_msgs { using namespace msg; }
namespace std_msgs { using namespace msg; }
namespace visualization_msgs { using namespace msg; }
namespace ros = rclcpp;

inline void ROS_INFO(const char * format, ... ) {
  va_list args;
  std::string with_new_line = format;
  with_new_line += "\n";
  va_start(args, with_new_line);
  vprintf(with_new_line.c_str(), args);
  va_end(args);
}

#define ROS_ERROR ROS_INFO
#define ROS_DEBUG ROS_INFO


namespace RosAPI {

using namespace builtin_interfaces::msg;
using namespace rclcpp::node;
using namespace rclcpp::rate;
using namespace rclcpp;

typedef tf2::Vector3 TfVector3;
typedef tf2_ros::TransformListener TfTransformListener;


inline void init(int argc, char** argv, const std::string & node_name) 
{
  rclcpp::init(argc, argv);
}


inline builtin_interfaces::msg::Time timeNow() 
{
  // return std::chrono::steady_clock::now();
  return builtin_interfaces::msg::Time();
}


template <typename NodeHandle>
inline void spinOnce(NodeHandle & node) 
{
  rclcpp::spin_some(node);
}


template <typename NodeHandle> 
inline void spin(NodeHandle & node) 
{
  rclcpp::spin(node);
}


template <typename T>
inline T transform(T & msg, const std::string & frame) 
{
  T transformed_msg = msg;
  tf2_ros::Buffer buffer;
  // tf2_ros::TransformListener tf2(buffer);

  // auto now = std::chrono::system_clock::now();
  // auto trans = buffer.lookupTransform(frame, msg.header.frame_id, now);
  // transformed_msg.pose = trans.transform * msg.pose;
  // // // buffer.transform(msg, transformed_msg, frame);
  // // // tf2_ros::BufferInterface::transform(msg, transformed_msg, frame);
  // // // tf2_ros::Buffer::transform(msg, transformed_msg, frame);
  return transformed_msg;
}


template <typename T>
inline void getParamWithDefault(RosAPI::Node * node, const std::string & name, 
                                T & var, const T & default_val) 
{
  // TODO: Implement
  auto node2 = rclcpp::Node::make_shared("get_parameters");
  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node2);
  if (false && parameters_client->has_parameter(name)){
    printf("before get\n");
    var = parameters_client->get_parameter(name, default_val);
    printf("after get\n");
  }
  else {
    var = default_val;
  }
}


inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) 
{
  tf2::Quaternion q;
  // TODO: This works but the documentation says setEuler(yaw, pitch, roll)
  q.setEuler(0.0, 0.0, yaw);
  geometry_msgs::Quaternion q_msg;
  q_msg.x = q.x(); q_msg.y = q.y(); q_msg.z = q.z(); q_msg.w = q.w();
  return q_msg;
}



/*
PUBLISHERS AND SUBSCRIBERS
*/

// Names for middleware-profiles
const static std::map<std::string, rmw_qos_profile_t> names_to_profiles = 
{
  {"default", rmw_qos_profile_default},
  {"sensor", rmw_qos_profile_sensor_data},
  {"camera", rmw_qos_profile_services_default}
};


inline rmw_qos_profile_t getProfile(const std::string & profile_name) 
{
  if (names_to_profiles.count(profile_name) == 0) {
    // Profile name NOT recognized, throw error
    ROS_INFO("Invalid QoS profile name");
    throw std::invalid_argument("Invalid QoS profile name");
  }

  return names_to_profiles.at(profile_name);
}



template <class MsgType>
class Subscriber {
 public:
  // Create a subscriber without initializing
  Subscriber() {}

  // Create and initialize a subscriber with an object and a method for callback
  template <typename Method, typename T>
  Subscriber(const std::string & topic, RosAPI::Node * node_handle, 
             Method method, T obj, const std::string & profile_name = "sensor") 
  {
    init(topic, node_handle, method, obj, profile_name);
  }

  // Create and initialize a subscriber with a callback function
  template <typename Callback>
  Subscriber(const std::string & topic, RosAPI::Node * node_handle, 
             Callback callback, const std::string & profile_name = "sensor") 
  {
    init(topic, node_handle, callback, profile_name);
  }

  // Initialize a subscriber with an object and a method for callback 
  template <typename Method, typename T>
  void init(const std::string & topic, RosAPI::Node * node_handle, 
            Method method, T obj, const std::string & profile_name = "sensor") 
  {
    auto callback = std::bind(method, obj, std::placeholders::_1);
    init(topic, node_handle, callback, profile_name);
  }

  // Initialize a subscriber with a callback function 
  template <typename Callback>
  void init(const std::string & topic, RosAPI::Node * node_handle, 
            Callback callback, const std::string & profile_name = "sensor")
  {
    rmw_qos_profile_t profile = getProfile(profile_name);
    subscription_ = node_handle->create_subscription<MsgType>(topic, callback, profile);
  }

  typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;
};



template <class MsgType>
class Publisher {
 public:
  // Create a publisher without initializing
  Publisher() {}

  // Create and initialize a publisher
  Publisher(const std::string & topic, RosAPI::Node * node, std::string profile_name = "sensor") 
  {
    advertise(topic, node, profile_name);
  }

  // Initialize a publisher
  void advertise(const std::string & topic, RosAPI::Node * node, 
                 std::string profile_name = "sensor") 
  {
    rmw_qos_profile_t profile = getProfile(profile_name);
    publisher_ = node->create_publisher<MsgType>(topic, profile);
  }

  // Publish msg, must have already advertised a topic
  void publish(const MsgType & msg) 
  {
    auto shared_msg = std::make_shared<MsgType>(msg);
    publishPointer(shared_msg);
  }

  // Publish as a pointer
  void publishPointer(std::shared_ptr<MsgType> & msg) 
  {
    printf("Publish\n");
    publisher_->publish(msg);
  }

  // void publish(const MsgType & msg) {
  //   publisher_->publish(msg);
  // }

  typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
};

} // RosAPI


/*
TODO: These functions don't really work and are just so that it compiles until they are 
      implemented in ros2
*/

inline RosAPI::Duration operator+(const RosAPI::Duration & lhs, const RosAPI::Duration & rhs) {
  RosAPI::Duration ans;
  ans.sec = lhs.sec + rhs.sec;
  return ans;
}

inline RosAPI::Duration operator-(const RosAPI::Duration & lhs, const RosAPI::Duration & rhs) {
  RosAPI::Duration ans;
  ans.sec = lhs.sec - rhs.sec;
  return ans;
}

inline RosAPI::Time operator+(const RosAPI::Time & lhs, const RosAPI::Time & rhs) {
  RosAPI::Time ans;
  ans.sec = lhs.sec + rhs.sec;
  return ans;
}

inline RosAPI::Duration operator-(const RosAPI::Time & lhs, const RosAPI::Time & rhs) {
  RosAPI::Duration ans;
  ans.sec = lhs.sec - rhs.sec;
  return ans;
}

inline RosAPI::Time operator+(const RosAPI::Time & lhs, const RosAPI::Duration & rhs) {
  RosAPI::Time ans;
  ans.sec = lhs.sec + rhs.sec;
  return ans;
}

inline RosAPI::Time operator-(const RosAPI::Time & lhs, const RosAPI::Duration & rhs) {
  RosAPI::Time ans;
  ans.sec = lhs.sec - rhs.sec;
  return ans;
}

// inline RosAPI::Duration durationFromTime(const RosAPI::Time & time) {
//   RosAPI::Duration dur;
//   dur.sec = time.sec;
//   return dur;
// }


#endif /* GLOBAL_PLANNER_ROS2_H_ */
