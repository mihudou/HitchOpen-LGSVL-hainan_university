cmake_minimum_required(VERSION 3.8)
project(simple_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 包含头文件
include_directories(
  include
)

# 源文件
set(SOURCES
  src/controller_manager_node.cpp
)

# 如果有插件代码，可单独列出
set(PLUGIN_SOURCES
  src/plugins/longitudinal/pid_controller.cpp
  src/plugins/lateral/pure_pursuit.cpp
)

# 生成插件库 (可选，如果有插件)
ament_auto_add_library(${PROJECT_NAME}
  SHARED
  ${PLUGIN_SOURCES}
)

# 导出插件库给pluginlib
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# 构建主节点
ament_auto_add_executable(controller_manager_node
  ${SOURCES}
)

# 链接主节点和插件库
target_link_libraries(controller_manager_node
  ${PROJECT_NAME}
)

# 安装目录
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

# 生成package
ament_auto_package()

