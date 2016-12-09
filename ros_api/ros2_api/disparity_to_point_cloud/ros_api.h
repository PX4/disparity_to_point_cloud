#ifndef GLOBAL_PLANNER_ROS2_H_
#define GLOBAL_PLANNER_ROS2_H_

#include <chrono>
#include <math.h>   // sqrt
#include <string>
#include <stdarg.h> // va_list va_start vprintf va_end
#include <stdexcept>

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

#include <ros2_additional_fix.h>

// #include "avoidance/msg/three_point_msg.hpp"


// Ros2 functionality

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

inline void init(int argc, char** argv, const std::string & node_name) {
  rclcpp::init(argc, argv);
}

inline builtin_interfaces::msg::Time timeNow() {
  // return std::chrono::steady_clock::now();
  return builtin_interfaces::msg::Time();
}

template <typename NodeHandle>
inline void spinOnce(NodeHandle & node) {
  rclcpp::spin_some(node);
}

template <typename NodeHandle> 
inline void spin(NodeHandle & node) {
  rclcpp::spin(node);
}

template <typename T>
inline T transform(T & msg, const std::string & frame) {
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
inline void getParamWithDefault(RosAPI::Node * node, const std::string & name, T & var, const T & default_val) {
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

inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  // TODO: This works but the documentation says setEuler(yaw, pitch, roll)
  q.setEuler(0.0, 0.0, yaw);
  geometry_msgs::Quaternion q_msg;
  q_msg.x = q.x(); q_msg.y = q.y(); q_msg.z = q.z(); q_msg.w = q.w();
  return q_msg;
}

template <class MsgType>
class Subscriber {
 public:
  Subscriber() {}

  template <typename Method, typename T>
  Subscriber(const std::string & topic, RosAPI::Node * node_handle, Method method, T obj) {
    init(topic, node_handle, method, obj);
  }

  template <typename Callback>
  Subscriber(const std::string & topic, RosAPI::Node * node_handle, Callback callback) {
    init(topic, node_handle, callback);
  }

  template <typename Method, typename T>
  void init(const std::string & topic, RosAPI::Node * node_handle, Method method, T obj){
    auto callback = std::bind(method, obj, std::placeholders::_1);
    init(topic, node_handle, callback);
  }

  template <typename Callback>
  void init(const std::string & topic, RosAPI::Node * node_handle, Callback callback){
    subscription_ = node_handle->create_subscription<MsgType>(topic, callback, rmw_qos_profile_sensor_data);
  }

  typename rclcpp::Subscription<MsgType>::SharedPtr subscription_;
};

// template <typename Method>
// rclcpp::Subscription<nav_msgs::Path>::SharedPtr
// createPathSubscription(const std::string & topic, Method method, RosAPI::Node * node) {
//   return node->create_subscription<nav_msgs::Path>(topic, method, rmw_qos_profile_sensor_data);
//   // return node->create_subscription<nav_msgs::Path>(topic, std::bind(method, node, std::placeholders::_1), rmw_qos_profile_sensor_data);
// }


template <class MsgType>
class Publisher {
 public:
  Publisher() {}

  Publisher(const std::string & topic, RosAPI::Node * node, std::string profile_name = "sensor") {
    advertise(topic, node, profile_name);
  }

  void advertise(const std::string & topic, RosAPI::Node * node, std::string profile_name = "sensor") {
    if (profile_name == "sensor") {
      publisher_ = node->create_publisher<MsgType>(topic, rmw_qos_profile_sensor_data);
    }
    else if (profile_name == "default") {
      publisher_ = node->create_publisher<MsgType>(topic, rmw_qos_profile_default);
    }
    else {
      ROS_INFO("Unknown QoS profile");
      throw std::invalid_argument("Unknown QoS profile");
    }
  }

  void publish(const MsgType & msg) {
    publisher_->publish(msg);
  }

  typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
};

}


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
