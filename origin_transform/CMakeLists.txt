cmake_minimum_required(VERSION 2.8.3)
project(origin_transform)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp 
    tf2
    tf2_ros
    dynamic_reconfigure
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

generate_dynamic_reconfigure_options(
    cfg/origin_transform.cfg
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/origin_transform.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(origin_transform_node node/origin_transform_node.cpp)
target_link_libraries(origin_transform_node ${PROJECT_NAME} ${catkin_LIBRARIES})