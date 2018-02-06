project(neck-control)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(. SRC_LIST)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/CiA402Device/)
INCLUDE_DIRECTORIES(lib/CiA402Device/)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/fcontrol/)
INCLUDE_DIRECTORIES(lib/fcontrol/)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( neck-control CiA402Device )
target_link_libraries( neck-control fcontrol )



