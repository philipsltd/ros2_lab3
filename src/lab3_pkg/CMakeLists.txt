cmake_minimum_required(VERSION 3.8)
project(lab3_pkg CXX)
# HELP ME PERES
# NOT ON GITHUB
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# as in https://github.com/ros-planning/navigation2/blob/main/nav2_behavior_tree/CMakeLists.txt
set(library_name ${PROJECT_NAME})

# Add your custom node source files NOT IN GITHUB
# set(SOURCES
#   src/DetectCone.cpp
# )

# set(dependencies
#   rclcpp
#   behaviortree_cpp_v3
#   std_msgs
# )
# message("Library Name: ${library_name}")
# message("Dependencies ${dependencies}")

add_library(nav2_my_detection_plugin SHARED src/DetectCone.cpp)
# message("Library Name: ${library_name}")

ament_target_dependencies(nav2_my_detection_plugin # ${library_name}
  rclcpp behaviortree_cpp_v3 std_msgs
)



# target_link_libraries(nav2_my_detection_plugin
#   ${dependencies}
# )

list(APPEND plugin_libs nav2_my_detection_plugin)



foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# Add an executable target NOT IN GITHUB
# add_executable(DetectCone ${SOURCES})

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY test/utils/
#         DESTINATION include/${PROJECT_NAME}/utils/
# )

# install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

# Link the libraries !!!!!!!!!!!!!!!!!!!!!
# ament_target_dependencies(DetectCone
#   rclcpp
#   behaviortree_cpp_v3
# )

# Install the executables/libraries !!!!!!!!!!!!!!!!!!!!!!!!!!
# install(TARGETS
#   DetectCone
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
