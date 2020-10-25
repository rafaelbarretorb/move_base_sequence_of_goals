cmake_minimum_required(VERSION 2.8.3)
project(move_base_sequence_of_goals)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
	rospy
	geometry_msgs
    actionlib
	actionlib_msgs
	move_base_msgs
	tf
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
	rospy
	geometry_msgs
    actionlib
	actionlib_msgs
	move_base_msgs
	tf
)

install(PROGRAMS
	nodes/sequence_of_goals
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
endif()
