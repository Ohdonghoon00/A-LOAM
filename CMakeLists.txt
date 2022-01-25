cmake_minimum_required(VERSION 2.8.3)
project(aloam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  roscpp
  rospy
  rosbag
  image_transport
  cv_bridge
  tf
  rviz_visual_tools
  message_generation
)
find_package( Boost REQUIRED COMPONENTS 
timer )
 

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  feature_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs visualization_msgs
  DEPENDS EIGEN3  
  INCLUDE_DIRS include
)

add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(ascanRegistration src/scanRegistration2.cpp)
# add_dependencies(ascanRegistration ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(PublishLidarData2 src/PublishLidarData2.cpp)
add_dependencies(PublishLidarData2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(PublishLidarData2 glog ${catkin_LIBRARIES} ${OpenCV_LIBS})




