cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_rgbdslam)

find_package(catkin REQUIRED roscpp image_transport tf2 pcl_ros octomap_ros octomap_msgs cv_bridge)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS}
                 ${PCL_LIBRARY_DIRS}
                 ${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OpenCV_LIBRARIES} 
               ${PCL_LIBRARIES}
               ${OCTOMAP_LIBRARIES})

catkin_package()

add_library(keyframe_livvemapper
  src/apps/keyfame_livemapper.cpp
  src/apps/rgbdtools_graph_detector.cpp
  src/apps/rgbdtools_graph_solver.cpp
  src/apps/octomap_handler.cpp
)

target_link_libraries(keyframe_livemapper
  rgbdtools
  ${OpenCV_LIBRARIES}
  tbb
)

add_executable(keyframe_livemapper_node src/node/keyframe_livemapper_node.cpp)

target_link_libraries(keyframe_livemapper_node keyframe_livemapper)
