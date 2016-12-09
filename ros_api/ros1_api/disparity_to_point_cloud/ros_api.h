#ifndef GLOBAL_PLANNER_ROS2_H_
#define GLOBAL_PLANNER_ROS2_H_

#include <chrono>
#include <math.h>   // sqrt
#include <string>
#include <stdarg.h> // va_list va_start vprintf va_end
#include <stdexcept>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h> 
#include <tf/transform_datatypes.h>  // getYaw

#include <disparity_to_point_cloud/ros1_additional_fix.h>



// Ros1 functionality
namespace RosAPI {

using namespace ros;

typedef ros::NodeHandle Node;
typedef tf::Vector3 TfVector3;
// typedef tf2_ros::TransformListener TfTransformListener;

inline void init(int argc, char** argv, const std::string & node_name) {
  ros::init(argc, argv, node_name);
}

inline ros::Time timeNow() {
  return ros::Time::now();
}

template <typename NodeHandle>
inline void spinOnce(NodeHandle & node) {
  ros::spinOnce();
}

template <typename NodeHandle> 
inline void spin(NodeHandle & node) {
  ros::spin();
}

template <typename T>
inline T transform(T & msg, const std::string & frame) {
  T transformed_msg = msg;
  // tf2_ros::Buffer buffer;
  // tf2_ros::TransformListener tf2(buffer);

  // auto now = std::chrono::system_clock::now();
  // auto trans = buffer.lookupTransform(frame, msg.header.frame_id, now);
  // transformed_msg.pose = trans.transform * msg.pose;
  // // // buffer.transform(msg, transformed_msg, frame);
  // // // tf2_ros::BufferInterface::transform(msg, transformed_msg, frame);
  // // // tf2_ros::Buffer::transform(msg, transformed_msg, frame);
  return transformed_msg;
}

inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw) {
  // TODO: implement
  return createQuaternionMsgFromYaw(yaw);
}

template <typename T>
inline void getParamWithDefault(RosAPI::Node * node, const std::string & name, T & var, const T & default_val) {
  node->param<T>(name, var, default_val);
}

template <class MsgType>
class Subscriber {
 public:
  Subscriber() {}

  template <typename Method, typename T>
  Subscriber(const std::string & topic, RosAPI::Node * node_handle, Method method, T obj) {
    init(topic, node_handle, method, obj);
  }

  template <typename Method, typename T>
  void init(const std::string & topic, RosAPI::Node * node_handle, Method method, T obj){
    subscription_ = node_handle->subscribe("/" + topic, 1, method, obj);
  }

  ros::Subscriber subscription_;
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
      publisher_ = node->advertise<MsgType>("/" + topic, 10);
    }
    else if (profile_name == "default") {
      publisher_ = node->advertise<MsgType>("/" + topic, 10);
    }
    else {
      ROS_INFO("Unknown QoS profile");
      throw std::invalid_argument("Unknown QoS profile");
    }
  }

  void publish(const MsgType & msg) {
    publisher_.publish(msg);
  }

  ros::Publisher publisher_;
};
  
}

#endif /* GLOBAL_PLANNER_ROS2_H_ */
