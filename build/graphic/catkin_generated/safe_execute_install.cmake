execute_process(COMMAND "/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/build/graphic/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/clown911026/cs/homework/機器人專題/專題/catkin_ws/build/graphic/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
