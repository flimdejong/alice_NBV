cmake_minimum_required(VERSION 3.1.3)
project(alice_moveit)

find_package(catkin REQUIRED)

catkin_package()


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp

)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
