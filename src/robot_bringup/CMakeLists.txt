# 模块 robot_bringup：框架 demo 启动与配置（产品请用各自 bringup）。
cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
