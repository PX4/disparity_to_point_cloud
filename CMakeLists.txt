cmake_minimum_required(VERSION 3.5)
project(disparity_to_point_cloud)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  include/ros_api/ros2_api
  include/ros_api_extend/ros2_api_extend
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")

macro(targets)
	if(NOT target_suffix STREQUAL "")
		get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
	endif()

	# ## Declare a C++ executable
	add_executable(disparity_to_point_cloud_node${target_suffix} 
				   src/disparity_to_point_cloud_node.cpp
				   src/disparity_to_point_cloud.cpp)

	ament_target_dependencies(disparity_to_point_cloud_node${target_suffix}
	    "rclcpp${target_suffix}"
	    "sensor_msgs"
	    "std_msgs"
	)

	## Specify libraries to link a library or executable target against
	target_link_libraries(disparity_to_point_cloud_node${target_suffix}
	  ${PCL_LIBRARIES}
	  ${OpenCV_LIBS}
	)

	install(TARGETS disparity_to_point_cloud_node${target_suffix}
    	DESTINATION bin
    )
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()
