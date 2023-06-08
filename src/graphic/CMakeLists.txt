cmake_minimum_required(VERSION 3.0.2)
project(graphic)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  bin/hello
  bin/action_graphic
  bin/result_graphic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


