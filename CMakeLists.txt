project(neck-control)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread" )

#main library with local functions and interfaces to external libraries.
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})

add_executable(main main/main.cpp)
target_link_libraries( main CiA402Device fcontrol )

add_executable(kinematics main/kinematics.cpp)
target_link_libraries( kinematics mainlib )

add_executable(position-control main/position-control.cpp)
target_link_libraries( position-control mainlib )

add_executable(openloop main/openloop.cpp)
target_link_libraries( openloop mainlib )




