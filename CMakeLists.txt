cmake_minimum_required(VERSION 2.8.3)
project(jetbot_od)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure image_transport nodelet roscpp std_msgs)
find_package(OpenCV REQUIRED)

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/Threshold.cfg
  cfg/YOLO.cfg
)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure nodelet roscpp std_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

macro(jetbot_od_add_nodelet node_name nodelet_cppfile)
  set(NODE_NAME ${node_name})
  set(NODELET_NAME jetbot_od/${node_name})
  configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
  add_executable(${node_name}_exe_ ${node_name}.cpp)
  SET_TARGET_PROPERTIES(${node_name}_exe_ PROPERTIES OUTPUT_NAME ${node_name})
  target_link_libraries(${node_name}_exe_ ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  list(APPEND _jetbot_od_nodelet_cppfiles ${nodelet_cppfile})
  list(APPEND _jetbot_od_nodelet_targets ${node_name}_exe)
endmacro()

jetbot_od_add_nodelet(threshold src/nodelet/threshold_nodelet.cpp)
jetbot_od_add_nodelet(yolo      src/nodelet/yolo_nodelet.cpp)

add_library(${PROJECT_NAME} SHARED
  src/nodelet/nodelet.cpp
  ${_jetbot_od_nodelet_cppfiles}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

