cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)
find_package(pcl_conversions)
find_package(pcl_ros)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(rtabmap_util REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS},)

# add_executable(create_pointcloud src/create_pointcloud.cpp)
# ament_target_dependencies(create_pointcloud rclcpp std_msgs sensor_msgs pcl_ros pcl_conversions cv_bridge rtabmap_util)
# target_link_libraries(create_pointcloud ${PCL_LIBRARIES})

# add_executable(binary_thresholding src/binary_thresholding.cpp)
# ament_target_dependencies(binary_thresholding rclcpp std_msgs sensor_msgs pcl_ros pcl_conversions cv_bridge rtabmap_util)
# target_link_libraries(binary_thresholding ${PCL_LIBRARIES})

add_executable(lane_pointcloud src/lane_pointcloud.cpp)
ament_target_dependencies(lane_pointcloud rclcpp rclcpp_components std_msgs sensor_msgs pcl_ros pcl_conversions tf2_ros tf2_geometry_msgs)
target_link_libraries(lane_pointcloud ${PCL_LIBRARIES} ${tf2_LIBRARIES})

add_executable(transform_pointcloud src/transform_pointcloud.cpp)
ament_target_dependencies(transform_pointcloud rclcpp rclcpp_components std_msgs sensor_msgs pcl_ros pcl_conversions tf2_ros tf2_geometry_msgs)
target_link_libraries(transform_pointcloud ${PCL_LIBRARIES} ${tf2_LIBRARIES})

install(TARGETS
  # create_pointcloud
  # binary_thresholding
  lane_pointcloud
  transform_pointcloud
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  # src/transform_pointcloud.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
