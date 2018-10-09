#project(mainlib)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(mainlib/ SRC_LIST)

add_subdirectory(CiA402Device/)
add_subdirectory(fcontrol/)
add_subdirectory(sensor-integration/)


set(SUBDIR_INCLUDE_DIRECTORIES ${SUBDIR_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/mainlib
    PARENT_SCOPE)

ADD_LIBRARY(mainlib SHARED ${SRC_LIST} )

target_link_libraries( mainlib CiA402Device fcontrol )



