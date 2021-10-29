cmake_minimum_required(VERSION 3.8)
project(pseudo_ndt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# build component.
function(add_library_dependencies library)
  target_compile_definitions(${library} PRIVATE "SAMPLE_NODE_BUILDING_DLL")
  ament_target_dependencies(${library}
    "rclcpp"
    "rclcpp_components"
    "std_msgs")
endfunction()

# build node component as library.
# ekf
add_library(pseudo_ekf_node SHARED src/nodes/pseudo_ekf.cpp)
add_library_dependencies(pseudo_ekf_node)
rclcpp_components_register_nodes(pseudo_ekf_node  "pseudo_ndt::PseudoEKF")
# ekf
add_library(pseudo_pointcloud_node SHARED src/nodes/pseudo_pointcloud.cpp)
add_library_dependencies(pseudo_pointcloud_node)
rclcpp_components_register_nodes(pseudo_pointcloud_node  "pseudo_ndt::PseudoPointCloud")
# ndt.
add_library(pseudo_ndt_node SHARED src/nodes/pseudo_ndt.cpp)
add_library_dependencies(pseudo_ndt_node)
rclcpp_components_register_nodes(pseudo_ndt_node  "pseudo_ndt::PseudoNDT")
# ndt.
add_library(pseudo_ndt_multi_thread_node SHARED src/nodes/pseudo_ndt_multi_thread.cpp)
add_library_dependencies(pseudo_ndt_multi_thread_node)
rclcpp_components_register_nodes(pseudo_ndt_multi_thread_node  "pseudo_ndt::PseudoNDTMultiThread")

# build executable.
# ekf.
add_executable(pseudo_ekf_exec src/pseudo_ekf_exec.cpp)
target_link_libraries(pseudo_ekf_exec pseudo_ekf_node)
ament_target_dependencies(pseudo_ekf_exec rclcpp std_msgs)
# pointcloud.
add_executable(pseudo_pointcloud_exec src/pseudo_pointcloud_exec.cpp)
target_link_libraries(pseudo_pointcloud_exec pseudo_pointcloud_node)
ament_target_dependencies(pseudo_pointcloud_exec rclcpp std_msgs)
# ndt.
add_executable(pseudo_ndt_exec src/pseudo_ndt_exec.cpp)
target_link_libraries(pseudo_ndt_exec pseudo_ndt_node)
ament_target_dependencies(pseudo_ndt_exec rclcpp std_msgs)
# ndt.
add_executable(pseudo_ndt_multi_thread_exec src/pseudo_ndt_multi_thread_exec.cpp)
target_link_libraries(pseudo_ndt_multi_thread_exec pseudo_ndt_multi_thread_node)
ament_target_dependencies(pseudo_ndt_multi_thread_exec rclcpp std_msgs)


ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_msgs)

install(DIRECTORY
  include/pseudo_ndt
  DESTINATION include
)

install(TARGETS
  pseudo_ekf_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  pseudo_pointcloud_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  pseudo_ndt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS
  pseudo_ndt_multi_thread_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install (TARGETS
  pseudo_ekf_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  pseudo_pointcloud_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  pseudo_ndt_exec
  DESTINATION lib/${PROJECT_NAME}
)
install (TARGETS
  pseudo_ndt_multi_thread_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
