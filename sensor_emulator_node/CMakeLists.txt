cmake_minimum_required(VERSION 3.0.2)
project(sensor_emulator)

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    pcl_ros
    actionlib
    roslib
    sensor_emulator_msgs
    roscpp
    sensor_msgs
    pcl_conversions
)

add_compile_options(-std=c++14)

find_package(OpenMP)


if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

find_package(Boost REQUIRED)
find_package(PCL 1.5 REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})


catkin_package(
    INCLUDE_DIRS
       include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
)


include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/sensor_emulator.cpp 
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)





