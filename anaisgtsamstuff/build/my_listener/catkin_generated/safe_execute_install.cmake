execute_process(COMMAND "/home/anaisjeger/catkin_ws/build/my_listener/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/anaisjeger/catkin_ws/build/my_listener/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
