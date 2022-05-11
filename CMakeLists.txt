cmake_minimum_required(VERSION 2.8.3)
project(drix_helm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  marine_msgs
  drix_msgs
  mdt_msgs
  roscpp
  rosbag
  project11
  project11_msgs
  geographic_visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_helm
  CATKIN_DEPENDS geometry_msgs roscpp project11 project11_msgs mdt_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/drix_helm_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp drix_msgs_gencpp mdt_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

add_executable(ais_to_udp src/ais_to_udp.cpp)
add_dependencies(ais_to_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ais_to_udp marine_msgs_gencpp drix_msgs_gencpp mdt_msgs_gencpp)
target_link_libraries(ais_to_udp ${catkin_LIBRARIES})

add_executable(range_to_mothership src/range_to_mothership.cpp)
add_dependencies(range_to_mothership ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(range_to_mothership mdt_msgs_gencpp)
target_link_libraries(range_to_mothership ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

