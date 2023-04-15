cmake_minimum_required(VERSION 3.10)
project(rbd_controller_template)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
#  standard C library includes are not required here
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_srvs
    roslint

    pcl_ros
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

## This is required to specify catkin-specific information to the build system which in turn is used to generate pkg-config and CMake files.
catkin_package(
  INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_coreS
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_srvs
    roslint

    pcl_ros

  DEPENDS
  ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
  ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
  # Eigen3

  ## Boost is not part of the DEPENDS since it is only used in source files,
  ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
)

##################################### Additional Libraries: uncomment everything with _core #######
# ## Declare a cpp library
# add_library(${PROJECT_NAME}_core
#    src/Algorithm.cpp
# )
# target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

## A executable file called "${PROJECT_NAME}" will be produced
## This file is build from the sources from the list (src/${PROJECT_NAME}_node.cpp and src RbdControllerTemplate.cpp)
## These two source files usually have 1 "main" function that gets executed when executing the "${PROJECT_NAME}" file
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/RbdControllerTemplate.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Add dependencies to exported targets, like ROS msgs or srvs
# add_dependencies(${PROJECT_NAME}_core
#   ${catkin_EXPORTED_TARGETS}
# )

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

# für die Laufzeit (find_packages() und catkin_package() für den Build)
# die erforderlichen Bibliotheken für das Build-Target mit dem Namen "${PROJECT_NAME}" hinzufügen.
# "${catkin_LIBRARIES}" wird normalerweise durch den Aufruf von "find_package(catkin REQUIRED COMPONENTS ...)" in der "CMakeLists.txt"-Datei definiert.
target_link_libraries(${PROJECT_NAME}
  # ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# if(${CATKIN_ENABLE_TESTING})
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# ## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test
#   test/test_ros_package_template.cpp
#   test/AlgorithmTest.cpp)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
# endif()

##########################
## Static code analysis ##
##########################

roslint_cpp()
