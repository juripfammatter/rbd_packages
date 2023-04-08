cmake_minimum_required(VERSION 2.8.3)
project(rbd_gestrec_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES python
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/gestrec.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
