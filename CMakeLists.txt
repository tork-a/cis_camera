cmake_minimum_required(VERSION 2.8.3)
project(cis_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  dynamic_reconfigure
  image_transport
  pluginlib
  nodelet
  sensor_msgs
  cv_bridge
  pcl_ros
  tf
)
find_package(OpenCV)

generate_dynamic_reconfigure_options(
  cfg/CISCamera.cfg
)

find_package(libuvc REQUIRED)
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
    cv_bridge
    pcl_ros
    tf
  LIBRARIES
    cis_camera_nodelet
)

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})
include_directories(include ${libuvc_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(camera_node src/main.cpp src/camera_driver.cpp)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_library(cis_camera_nodelet src/nodelet.cpp src/camera_driver.cpp)
add_dependencies(cis_camera_nodelet ${cis_camera_EXPORTED_TARGETS})
target_link_libraries(cis_camera_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(cis_camera_nodelet ${PROJECT_NAME}_gencfg)

add_executable(pcl_example src/pcl_example.cpp)
target_link_libraries(pcl_example ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pcl_example ${PROJECT_NAME}_gencfg)

install(TARGETS camera_node cis_camera_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES cis_camera_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

## Mark executables and/or libraries for installation
install(DIRECTORY config launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(status "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
  
  # file(GLOB TEST_FILES test/*.test)
  # foreach(TEST_FILE ${TEST_FILES})
  #   message(status "Testing ${TEST_FILE}")
  #   add_rostest(${TEST_FILE})
  # endforeach()

endif()


