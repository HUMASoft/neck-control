project(neck-control)
cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread" )

if(UNIX AND NOT APPLE AND "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    # FIX: Qt was built with -reduce-relocations
    add_definitions(-fPIC)
endif()
## FIX: Qt was built with -reduce-relocations
#if (Qt5_POSITION_INDEPENDENT_CODE)
#    SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
#endif()

#main library with local functions and interfaces to external libraries.
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})

add_executable(main main/main.cpp)
target_link_libraries( main CiA402Device fcontrol mainlib)

#add_executable(closeloop main/closeloop.cpp)
#target_link_libraries( closeloop CiA402Device fcontrol mainlib)

add_executable(kinematics main/kinematics.cpp)
target_link_libraries( kinematics mainlib )

add_executable(position-control main/position-control.cpp)
target_link_libraries( position-control mainlib )

add_executable(openloop main/openloop.cpp)
target_link_libraries( openloop mainlib )

add_executable(feedback main/feedback.cpp)
target_link_libraries( feedback mainlib )

#add_executable(test main/test.cpp)
#target_link_libraries( test mainlib )

add_executable(closedloop main/closedloop.cpp)
target_link_libraries( closedloop mainlib )

add_executable(start main/start.cpp)
target_link_libraries( start mainlib )


add_executable(stop main/stop.cpp)
target_link_libraries( stop mainlib )

add_executable(demo1 main/demo1.cpp)
target_link_libraries( demo1 mainlib )

add_executable(sensortest main/sensortest.cpp)
target_link_libraries( sensortest mainlib )
