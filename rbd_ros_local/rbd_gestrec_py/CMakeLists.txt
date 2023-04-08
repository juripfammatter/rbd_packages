cmake_minimum_required(VERSION 2.8.3)
project(rbd_gestrec_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

catkin_python_setup()

add_service_files(  # add files to generate-list
  FILES
  getLastGesture.srv
)

generate_messages(  # generate files
  DEPENDENCIES
  std_msgs          # for string (response)
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES python
   CATKIN_DEPENDS 
     rospy
     std_msgs
     message_runtime 

#  DEPENDS system_lib

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/gestrec_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

