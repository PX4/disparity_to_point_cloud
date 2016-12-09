#ifndef GLOBAL_PLANNER_ROS2_ADDITIONAL_FIX
#define GLOBAL_PLANNER_ROS2_ADDITIONAL_FIX

// #include <sensor_msgs/distortion_models.h>
// #include <ros/console.h>

// #include <ait_ros_messages/msg/vio_sensor_msg.hpp>
#include <ait_ros_messages/msg/vio_sensor_msg.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/fill_image.h>

namespace sensor_msgs
{
  namespace msg 
  {
	static inline bool fillImage(Image& image,
	             const std::string& encoding_arg,
	             uint32_t rows_arg,
	             uint32_t cols_arg,
	             uint32_t step_arg,
	             const void* data_arg)
	{
	image.encoding = encoding_arg;
	image.height   = rows_arg;
	image.width    = cols_arg;
	image.step     = step_arg;
	size_t st0 = (step_arg * rows_arg);
	image.data.resize(st0);
	memcpy(&image.data[0], data_arg, st0);

	image.is_bigendian = 0;
	return true;
	}

	static inline void clearImage(Image& image)
	{
	image.data.resize(0);
	}
  }
}

#endif /* GLOBAL_PLANNER_ROS2_ADDITIONAL_FIX */
