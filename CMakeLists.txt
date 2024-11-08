cmake_minimum_required(VERSION 3.22)
project(rsl_drive_sdk)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_definitions(-DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)
add_definitions(-DMELO_USE_COUT)

find_package(ethercat_sdk_master REQUIRED)




file(GLOB_RECURSE  SRC_FILES CONFIGURE_DEPENDS ./src/*.c*)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PUBLIC -fPIC)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror=return-type)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  ethercat_sdk_master
  yaml_cpp_vendor
)





install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  ethercat_sdk_master
  yaml_cpp_vendor

)


ament_package()
