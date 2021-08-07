cmake_minimum_required(VERSION 3.5)
project(ugv_sdk VERSION 0.1.5)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BUILD_WITHOUT_ROS ON)

find_package(ament_cmake REQUIRED)
find_package(async_port REQUIRED)

message("port: " ${async_port_FOUND})
message("port: " ${async_port_INCLUDE_DIRS})
message("port: " ${async_port_LIBRARIES})

### Generate symbols for IDE indexer
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
#list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")
#list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

### Chosse build type
#set(default_build_type "Release")
#if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
#  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
#      STRING "Choose the type of build." FORCE)
#  # Set the possible values of build type for cmake-gui
#  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
#    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
#endif()


include_directories(
    include
    include/${PROJECT_NAME}
    ${async_port_INCLUDE_DIRS}/async_port/asio/include/
)

# Build libraries
find_package(Threads REQUIRED)

file(GLOB SOURCE_FILE_NAMES
    src/*.cpp
    src/*.c
)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILE_NAMES})
ament_target_dependencies(${PROJECT_NAME} async_port)
    
install(
  DIRECTORY include/ugv_sdk
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
