cmake_minimum_required(VERSION 3.8)
project(astro_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(astro_action_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_python_install_package(${PROJECT_NAME})


add_library(astro_action_servers SHARED
  src/go_to_pos_action.cpp)
rclcpp_components_register_node(astro_action_servers PLUGIN "astro_action_servers::GoToPositionActionServer" EXECUTABLE go_to_action_server)

target_compile_features(astro_action_servers PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_compile_definitions(astro_action_servers
  PRIVATE "ASTRO_ACTIONS_CPP_BUILDING_DLL")

  target_include_directories(astro_action_servers PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")



  
  target_link_libraries(astro_action_servers
  ${astro_action_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  
)
ament_target_dependencies(astro_action_servers
  moveit_ros_planning_interface
)

install(TARGETS
astro_action_servers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# python install

install(PROGRAMS
  astro_actions/wait_for_cmd_action_server.py
  astro_actions/init_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()
