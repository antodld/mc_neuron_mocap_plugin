cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME mc_neuron_mocap_plugin)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")
set(PROJECT_VERSION 0.0.1)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CMAKE_CXX_STANDARD 17)
set(CXX_DISABLE_WERROR 1) # FIXME fix errors and re-enable

include(cmake/base.cmake)

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

set(mc_neuron_mocap_properties_HDR_DIR
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/mc_neuron_mocap_plugin>$<INSTALL_INTERFACE:include/mc_neuron_mocap_plugin>)

set(mc_neuron_mocap_properties_HDR
  ${mc_neuron_mocap_properties_HDR_DIR}/MoCapProperties.h
)

# header-only
add_library(mc_neuron_mocap_properties INTERFACE)
if(COMMAND target_sources)
  target_sources(mc_neuron_mocap_properties INTERFACE ${mc_neuron_mocap_properties_HDR})
endif()
add_library(properties ALIAS mc_neuron_mocap_properties)

install(TARGETS mc_neuron_mocap_properties
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

add_subdirectory(src)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
