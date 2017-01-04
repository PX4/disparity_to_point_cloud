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


namespace RosAPI {
	
inline int addPointField(sensor_msgs::PointCloud2 & cloud_msg, const std::string & name, 
						 int count, int datatype, int offset)
{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  // Update the offset
  return offset + point_field.count * 4;
}

void toROSMsg(const pcl::PointCloud<pcl::PointXYZ> & cloud_in, 
			   sensor_msgs::PointCloud2 & cloud_out) 
{	

	int n = cloud_in.points.size();
	cloud_out.height = 1;
	cloud_out.width = n;

	int offset = 0;
	offset = addPointField(cloud_out, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(cloud_out, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
	offset = addPointField(cloud_out, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
	offset = 16;

	cloud_out.is_bigendian = false;
	cloud_out.point_step = offset;
	cloud_out.row_step = n * offset;

	int data_size = cloud_out.row_step;
	std::vector< pcl::uint8_t > data;
	data.resize (data_size);
	memcpy (&data[0], &cloud_in.points[0], data_size);
	cloud_out.data.swap(data);
	cloud_out.is_dense = false;
}

} // RosAPI

#endif /* GLOBAL_PLANNER_ROS2_ADDITIONAL_FIX */
