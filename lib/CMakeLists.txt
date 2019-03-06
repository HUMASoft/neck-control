#project(mainlib)
cmake_minimum_required(VERSION 2.8)

#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Dependencies
#1-eigen
find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "EIGEN: " ${EIGEN3_VERSION} " (internal)")
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(mainlib/ SRC_LIST)

add_subdirectory(CiA402Device/)
add_subdirectory(fcontrol/)
add_subdirectory(sensor-integration/)


set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/mainlib
    PARENT_SCOPE)

ADD_LIBRARY(mainlib SHARED ${SRC_LIST} )

target_link_libraries( mainlib CiA402Device fcontrol SensorIntegration)



