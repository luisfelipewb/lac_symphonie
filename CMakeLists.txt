cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(lac_symphonie)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Hooks: worlds/ and models/ on GZ_SIM_RESOURCE_PATH so that
# `gz sim lac_symphonie.sdf` and model://lac_symphonie_terrain resolve
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# Python module (geo helpers + the geo_tf_publisher node)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/geo_tf_publisher
  scripts/latlon_to_local.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geo test/test_geo.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
