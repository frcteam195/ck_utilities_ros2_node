cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "ck_utilities_ros2_node")
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(boost_libraries_list
  date_time
  filesystem
)

set(third_party_find_package_list
  ament_cmake
  Threads
)

set(ros_find_package_list
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  ck_ros2_base_msgs_node
  tf2
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs

)

set(link_libraries
  Threads::Threads
)


file(GLOB_RECURSE executable_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

foreach(tdependency ${third_party_find_package_list})
  find_package(${tdependency} REQUIRED)
endforeach()


foreach(rdependency ${ros_find_package_list})
  find_package(${rdependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS ${boost_libraries_list}) 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} ${executable_sources})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
)

foreach(rtdependency ${ros_find_package_list})
  ament_target_dependencies(${PROJECT_NAME} ${rtdependency})
endforeach()

target_link_libraries(${PROJECT_NAME} ${link_libraries} ${Boost_LIBRARIES} Eigen3::Eigen)


target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# this line to exports the library
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
 
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
