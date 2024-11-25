cmake_minimum_required(VERSION 3.1)
project(IndustrialVision)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Open3D REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Open3D_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# # 添加第一个可执行文件
# set(EXECUTABLE_NAME_1 "1")
# add_executable(${EXECUTABLE_NAME_1} 1.cpp)
# target_link_libraries(${EXECUTABLE_NAME_1} ${PCL_LIBRARIES} ${Boost_LIBRARIES} Open3D::Open3D)

# 添加第二个可执行文件
set(EXECUTABLE_NAME_2 "2")
add_executable(${EXECUTABLE_NAME_2} 2.cpp)
target_link_libraries(${EXECUTABLE_NAME_2} ${PCL_LIBRARIES} ${Boost_LIBRARIES} Open3D::Open3D)

# # 添加第三个可执行文件
# set(EXECUTABLE_NAME_3 "3")
# add_executable(${EXECUTABLE_NAME_3} 3.cpp)
# target_link_libraries(${EXECUTABLE_NAME_3} Open3D::Open3D)