cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME mocap_plugin)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")
set(PROJECT_VERSION 0.0.1)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CMAKE_CXX_STANDARD 11)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})
option(AUTOLOAD_${PROJECT_NAME}_PLUGIN "Automatically load ${PROJECT_NAME} plugin" OFF)


include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()




# Configure and install the plugin configuration
configure_file(etc/${PROJECT_NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")



set(plugin_SRC
  src/plugin.cpp
  src/MoCap.cpp
  src/ClientSocket.cpp
  src/Socket.cpp
  )

set(plugin_HDR
  include/mc_neuron_mocap_plugin/MoCap.h
  src/plugin.h
  src/Socket.h
  src/ClientSocket.h
  src/SocketException.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(${PROJECT_NAME} "${plugin_SRC}" "${plugin_HDR}")

target_include_directories(${PROJECT_NAME} PUBLIC ${roscpp_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${roscpp_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${std_msgs_stamped_INCLUDE_DIRS})

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
