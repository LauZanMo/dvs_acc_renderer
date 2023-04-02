cmake_minimum_required(VERSION 3.0.2)
project(dvs_acc_renderer)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# build dvs_acc_renderer library
include_directories(
    ${PROJECT_SOURCE_DIR}/src/lib
)
file(GLOB_RECURSE DVS_ACC_RENDERER_SOURCE src/lib/*.cpp)
add_library(${PROJECT_NAME} SHARED
    ${DVS_ACC_RENDERER_SOURCE}
)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# OpenCV
set(OpenCV_DIR "/usr/local/opencv4.5.5/lib/cmake/opencv4/")
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(${PROJECT_NAME} TBB::tbb)

# ROS
set(cv_bridge_DIR "/usr/local/cv_bridge4.5.5/share/cv_bridge/cmake/")
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    dvs_msgs
    image_transport
    roscpp
    sensor_msgs
)
catkin_package(
    INCLUDE_DIRS src/lib
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS cv_bridge dvs_msgs image_transport roscpp sensor_msgs
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# build ros node
add_executable(${PROJECT_NAME}_node src/dvs_acc_renderer_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})