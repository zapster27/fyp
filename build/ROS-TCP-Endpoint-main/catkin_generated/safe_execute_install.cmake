execute_process(COMMAND "/home/ros1/Documents/fyp2/build/ROS-TCP-Endpoint-main/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ros1/Documents/fyp2/build/ROS-TCP-Endpoint-main/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
