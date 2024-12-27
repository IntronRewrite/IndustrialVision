cmake_minimum_required(VERSION 3.10)
project(IndustrialVision)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 Open3D 库
find_package(Open3D REQUIRED)

# 查找 Eigen 库
find_package(Eigen3 REQUIRED)

# 添加可执行文件
add_executable(main main.cpp)
add_executable(RegistrationRANSAC RegistrationRANSAC.cpp)
add_executable(RegistrationICP RegistrationICP.cpp)


# 链接 Open3D 和 Eigen 库
target_link_libraries(main PRIVATE Open3D::Open3D Eigen3::Eigen)
target_link_libraries(RegistrationRANSAC PRIVATE Open3D::Open3D Eigen3::Eigen)
target_link_libraries(RegistrationICP PRIVATE Open3D::Open3D Eigen3::Eigen)