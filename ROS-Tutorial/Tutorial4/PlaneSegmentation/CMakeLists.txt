cmake_minimum_required(VERSION 2.8.3)
project(PlaneSegmentation)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  image_geometry
  pcl_ros
  pcl_conversions

#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES PlaneSegmentation
CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf image_geometry
)
## Specify additional locations of header files
## Your package locations should be listed before other locations

#>>>>TODO: Add the other include locations.
include_directories(
  ${catkin_INCLUDE_DIRS}  
  include
  ${PCL_INCLUDE_DIRS}
)


#>>>>TODO: Create a library for the class declared in the files.
add_library(PlaneSegmentation_function src/PlaneSegmentation_function.cpp)
target_link_libraries(PlaneSegmentation_function ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(PlaneSegmentation_function ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#>>>>TODO: Link the executable with the library.


## Declare a cpp executable
add_executable(PlaneSegmentation_main src/PlaneSegmentation_main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(PlaneSegmentation_main ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} PlaneSegmentation_function)


