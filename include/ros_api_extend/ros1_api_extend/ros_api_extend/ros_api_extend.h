#ifndef GLOBAL_PLANNER_ROS1_ADDITIONAL_FIX
#define GLOBAL_PLANNER_ROS1_ADDITIONAL_FIX

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>

// #include <ait_ros_messages/VioSensorMsg.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>

#include <pcl_conversions/pcl_conversions.h>



namespace RosAPI {

void toROSMsg(const pcl::PointCloud<pcl::PointXYZ> & cloud_in, 
			  sensor_msgs::PointCloud2 & cloud_out) 
{
  pcl::toROSMsg(cloud_in, cloud_out);
}

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

// Mostly copied from pcl::toROSMsg
void toROSMsg2(const pcl::PointCloud<pcl::PointXYZ> & cloud_in, 
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

	// data = cloud_in.points;
	int data_size = cloud_out.row_step;
	std::vector< pcl::uint8_t > data;
	data.resize(data_size);
	memcpy (&data[0], &cloud_in.points[0], data_size);
	cloud_out.data.swap(data);
	cloud_out.is_dense = false;
}

} // RosAPI

#endif /* GLOBAL_PLANNER_ROS1_ADDITIONAL_FIX */
