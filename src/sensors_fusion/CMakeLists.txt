cmake_minimum_required(VERSION 3.0.2)
project(sensors_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  util
  common_msgs
  sensor_msgs
  can_msgs
  nav_msgs
)
#find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
#find_package(OpenCV REQUIRED)
# find_package(lglog REQUIRED)

find_package(Protobuf 3.3 REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  # ./include/base
  ./proto
  # ${CMAKE_CURRENT_SOURCE_DIR}
  # ${CMAKE_CURRENT_SOURCE_DIR}/include/util
  # src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensors_fusion
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

############### base_lib ######################
add_library(base_lib STATIC ${PROJECT_SOURCE_DIR}/src/base/base_init_options.cc
                            ${PROJECT_SOURCE_DIR}/src/base/scene.cc
                            ${PROJECT_SOURCE_DIR}/src/base/sensor.cc
                            ${PROJECT_SOURCE_DIR}/src/base/sensor_data_manager.cc
                            ${PROJECT_SOURCE_DIR}/src/base/sensor_frame.cc
                            ${PROJECT_SOURCE_DIR}/src/base/sensor_object.cc
                            ${PROJECT_SOURCE_DIR}/src/base/track.cc
                            ${PROJECT_SOURCE_DIR}/src/base/track_pool_types.cc
                            )

add_dependencies(base_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_lib ${Boost_LIBRARIES}
                               ${PROTOBUF_LIBRARY}
                               ${catkin_INCLUDE_DIRS}
                              #  ${GLOG_LIBRARY}
                                )

############### util_lib ######################
add_library( util_lib STATIC ${PROJECT_SOURCE_DIR}/src/util/binary.cc
                             ${PROJECT_SOURCE_DIR}/src/util/file.cc
                             ${PROJECT_SOURCE_DIR}/src/util/blob.cc
                            #  ${PROJECT_SOURCE_DIR}/src/util/camera.cc
                            #  ${PROJECT_SOURCE_DIR}/src/util/distortion_model.cc
                             ${PROJECT_SOURCE_DIR}/src/util/object.cc
                             ${PROJECT_SOURCE_DIR}/src/util/object_pool_types.cc
                            #  ${PROJECT_SOURCE_DIR}/src/util/omnidirectional_model.cc
                             ${PROJECT_SOURCE_DIR}/src/util/point_cloud_util.cc
                             ${PROJECT_SOURCE_DIR}/src/util/polynomial.cc
                             ${PROJECT_SOURCE_DIR}/src/util/syncedmem.cc
                            #  ${PROJECT_SOURCE_DIR}/src/util/timer.cc
                            #  ${PROJECT_SOURCE_DIR}/src/util/util.cc
                            #  ${PROJECT_SOURCE_DIR}/src/util/vec2d.cc
                            )
target_link_libraries( util_lib ${Boost_LIBRARIES}
                                ${PROTOBUF_LIBRARY}
                              #  ${GLOG_LIBRARY}
                                )

############### common_lib ######################
add_library( common_lib STATIC ${PROJECT_SOURCE_DIR}/src/common/dst_evidence.cc
                               ${PROJECT_SOURCE_DIR}/src/common/information_filter.cc
                               ${PROJECT_SOURCE_DIR}/src/common/kalman_filter.cc
                               ${PROJECT_SOURCE_DIR}/src/common/sensor_manager/sensor_manager.cc
                              #  ${PROJECT_SOURCE_DIR}/src/common/camera.cc
                              #  ${PROJECT_SOURCE_DIR}/src/common/camera_util.cc
                               ${PROJECT_SOURCE_DIR}/src/common/connected_component_analysis.cc
                               ${PROJECT_SOURCE_DIR}/src/common/duration.cc
                               ${PROJECT_SOURCE_DIR}/src/common/information_filter.cc
                               ${PROJECT_SOURCE_DIR}/src/common/perception_gflags.cc
                               ${PROJECT_SOURCE_DIR}/src/common/time.cc
                              )
target_link_libraries(common_lib
                      util_lib
                      fusion_proto_lib
                      # ${GFLAGS_LIBRARY}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${PROTOBUF_LIBRARIES}
                      )



############### fusion_proto_lib ######################
add_library(fusion_proto_lib STATIC ${PROJECT_SOURCE_DIR}/proto/dst_existence_fusion_config.pb.cc
                                    ${PROJECT_SOURCE_DIR}/proto/dst_type_fusion_config.pb.cc
                                    ${PROJECT_SOURCE_DIR}/proto/pbf_tracker_config.pb.cc
                                    ${PROJECT_SOURCE_DIR}/proto/perception_config_schema.pb.cc
                                    ${PROJECT_SOURCE_DIR}/proto/probabilistic_fusion_config.pb.cc
                                    ${PROJECT_SOURCE_DIR}/proto/sensor_meta_schema.pb.cc
                                    # ${PROJECT_SOURCE_DIR}/proto/async_fusion_config.pb.cc
                                    # ${PROJECT_SOURCE_DIR}/proto/async_fusion_config.pb.cc
                                    # ${PROJECT_SOURCE_DIR}/proto/async_fusion_config.pb.cc
                                    )

############### data_fusion_lib ######################
add_library( data_fusion_lib STATIC ${PROJECT_SOURCE_DIR}/src/lib/data_association/hm_data_association/hm_tracks_objects_match.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_association/hm_data_association/probabilities.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_association/hm_data_association/track_object_distance.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_association/hm_data_association/track_object_similarity.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_fusion/dst_existence_fusion.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_fusion/dst_type_fusion.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_fusion/kalman_motion_fusion.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_fusion/pbf_shape_fusion.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/data_fusion/pbf_tracker.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/fusion_system/probabilistic_fusion/probabilistic_fusion.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/registerer/registerer.cc
                                    ${PROJECT_SOURCE_DIR}/src/lib/config_manager/config_manager.cc
                                    )

target_link_libraries(data_fusion_lib
                      util_lib
                      common_lib
                      ${OpenCV_LIBS}
                      )



############### obstacle_fusion_lib ######################
add_library( obstacle_fusion_lib STATIC ${PROJECT_SOURCE_DIR}/src/obstacle_multi_sensor_fusion.cc)

target_link_libraries(obstacle_fusion_lib
                      data_fusion_lib
                      util_lib
                      common_lib
                      )
link_directories(${PROTOBUF_LIBRARIES})

############### executable ######################
add_executable(${PROJECT_NAME}_node
               src/main.cpp
               src/sensors_fusion.cc
              #  src/obstacle_multi_sensor_fusion.cc
              #  src/utm_lib.cpp
              #  src/hungarian.cpp
               )
target_link_libraries(${PROJECT_NAME}_node
                      ${PCL_LIBRARIES}
                      ${PROTOBUF_LIBRARY_DEBUG}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      obstacle_fusion_lib
                      data_fusion_lib
                      util_lib
                      common_lib
                      # yaml-cpp
                      )
# target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES}  ${PROTOBUF_LIBRARY_DEBUG} ${catkin_LIBRARIES} ${OpenCV_LIBS} obstacle_fusion_lib glog gflags)

# install(DIRECTORY launch DESTINATION DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch PATTERN ".svn" EXCLUDE)

# install(DIRECTORY include/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )