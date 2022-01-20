cmake_minimum_required(VERSION 3.0.2)
project(hce_dumpdetector)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pg -O3 -m64 -Ofast -flto -msse -msse2 -msse3 -mavx2 -march=native -mfpmath=sse")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pg -O3 -m64 -Ofast -flto -msse -msse2 -msse3 -mavx2 -march=native -mfpmath=sse")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  cv_bridge

  apriltag_ros
  hce_msgs

  std_msgs
  sensor_msgs
  geometry_msgs
  rosgraph_msgs

)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
	DEPENDS EIGEN3
  # CATKIN_DEPENDS message_runtime roscpp message_filters
  INCLUDE_DIRS include
)

set(SRC ${SRC} 
  src/hce_detector.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

add_executable(example_client ${SRC} src/example_client.cpp)
target_link_libraries(example_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (${PROJECT_NAME}_server_node ${SRC} src/${PROJECT_NAME}_server_node.cpp )
target_link_libraries (${PROJECT_NAME}_server_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

# install
install(TARGETS ${PROJECT_NAME}_server_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(TARGETS ros_clock_pub RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)