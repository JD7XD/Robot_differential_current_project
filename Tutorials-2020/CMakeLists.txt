cmake_minimum_required(VERSION 3.0.2)
project(tutorial)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs genmsg tf
geometry_msgs
nav_msgs )
find_package(OpenCV REQUIRED)
# cv_bridge pcl_ros pcl_conversions

# find_package(PCL 1.7 REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#   pcl_ros
#   pcl_conversions
#   # ... other dependencies ...
# )

catkin_package(CATKIN_DEPENDS roscpp tf geometry_msgs nav_msgs)
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

 catkin_package(
   CATKIN_DEPENDS pcl_conversions pcl_ros
 )
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    # ${PCL_INCLUDE_DIRS}
                    # ${image_transport_LIBRARIES}
                    ${OpenCV_LIBS})

add_executable(my_publisher src/main.cpp)
#add_executable(lidar src/lidar.cpp)
#add_executable(lidar_ex src/lidar_ex.cpp)
add_executable(hokuyo src/hokuyo_mindis.cpp)
add_executable(control src/robot_control.cpp)
add_executable(object src/main.cpp)
add_executable(odom src/odom.cpp)
# add_executable(mapp src/mapp.cpp)
add_executable(tm src/turtle_move.cpp)
add_executable(ocv src/opencv_copy.cpp)
add_executable(ocv_vid src/opencv_readvid.cpp)
add_executable(goback src/min_dis_detect.cpp)
add_executable(auto src/auto_drive.cpp)




target_link_libraries(my_publisher ${catkin_LIBRARIES})
#target_link_libraries(lidar ${catkin_LIBRARIES})
#target_link_libraries(lidar_ex ${catkin_LIBRARIES})
target_link_libraries(hokuyo ${catkin_LIBRARIES})
target_link_libraries(control ${catkin_LIBRARIES})
target_link_libraries(object ${catkin_LIBRARIES})
target_link_libraries(odom ${catkin_LIBRARIES})
# target_link_libraries(mapp ${catkin_LIBRARIES})
target_link_libraries(tm ${catkin_LIBRARIES})
target_link_libraries(ocv ${OpenCV_LIBRARIES})
target_link_libraries(ocv_vid ${OpenCV_LIBRARIES})
target_link_libraries(goback ${catkin_LIBRARIES})
target_link_libraries(auto ${catkin_LIBRARIES})
