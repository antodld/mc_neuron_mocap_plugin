set(plugin_SRC
  plugin.cpp
  MoCap.cpp
  ClientSocket.cpp
  Socket.cpp
)

set(plugin_HDR
  ${PROJECT_SOURCE_DIR}/include/mc_neuron_mocap_plugin/MoCap.h
  plugin.h
  Socket.h
  ClientSocket.h
  SocketException.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(${PROJECT_NAME} "${plugin_SRC}" "${plugin_HDR}")
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_neuron_mocap_properties)
target_include_directories(${PROJECT_NAME} PUBLIC ${roscpp_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${roscpp_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${std_msgs_stamped_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/plugin/$<CONFIG>
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/plugin/$<CONFIG>
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/plugin/$<CONFIG>
)
