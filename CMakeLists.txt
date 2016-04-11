cmake_minimum_required(VERSION 2.8.3)
project(rotors_exercise)


#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## System dependencies are found with CMake's conventions
find_package(OpenCV 2 REQUIRED)

##Qt5 dependences dont know why.....IF NOT ERROR COMPILE
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

#source files

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

#find_package(Eigen REQUIRED)

catkin_package(
  #INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  #LIBRARIES
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs
  #DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
## Specify additional locations of header files
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(estimator_node src/estimator.cpp)

target_link_libraries(estimator_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(estimator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(estimator_node ${catkin_LIBRARIES})

add_executable(path_node src/getPath.cpp)

target_link_libraries(path_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_node ${catkin_LIBRARIES})


