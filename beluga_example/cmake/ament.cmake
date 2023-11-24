# Copyright 2023 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(beluga REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(beluga_april_tag_adapter_msgs REQUIRED)
find_package(beluga_feature_map_server_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(lmcl_example_component SHARED)
target_sources(lmcl_example_component PRIVATE src/lmcl_example.cpp)
target_compile_features(lmcl_example_component PUBLIC cxx_std_17)
target_link_libraries(lmcl_example_component PUBLIC beluga::beluga)
ament_target_dependencies(
  lmcl_example_component
  PUBLIC beluga_april_tag_adapter_msgs
         beluga_feature_map_server_msgs
         message_filters
         nav2_msgs
         rclcpp
         rclcpp_components
         tf2
         tf2_eigen
         tf2_geometry_msgs
         tf2_ros)
rclcpp_components_register_node(
  lmcl_example_component
  PLUGIN "beluga_example::LandmarkBasedMonteCarloLocalizationNode"
  EXECUTABLE lmcl_example_node)

add_library(lbmcl_example_component SHARED)
target_sources(lbmcl_example_component PRIVATE src/lbmcl_example.cpp)
target_compile_features(lbmcl_example_component PUBLIC cxx_std_17)
target_link_libraries(lbmcl_example_component PUBLIC beluga::beluga)
ament_target_dependencies(
  lbmcl_example_component
  PUBLIC beluga_april_tag_adapter_msgs
         beluga_feature_map_server_msgs
         message_filters
         nav2_msgs
         rclcpp
         rclcpp_components
         tf2
         tf2_eigen
         tf2_geometry_msgs
         tf2_ros)
rclcpp_components_register_node(
  lbmcl_example_component
  PLUGIN "beluga_example::LandmarkBearingBasedMonteCarloLocalizationNode"
  EXECUTABLE lbmcl_example_node)

install(
  TARGETS lmcl_example_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS lbmcl_example_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS lmcl_example_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS lbmcl_example_component DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR
                             ${PROJECT_SOURCE_DIR}/${PROJECT_NAME})

install(
  DIRECTORY bags
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.bag" EXCLUDE)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.ros2.rviz")

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.ros2.yaml")

install(DIRECTORY maps models worlds DESTINATION share/${PROJECT_NAME})

ament_package()
