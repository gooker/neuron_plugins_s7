set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/plugins")

set(S7_SRC s7.c s7_point.c s7_req.c s7_stack.c)

set(CMAKE_BUILD_RPATH ./)
file(COPY ${CMAKE_SOURCE_DIR}/plugins/s7/s7-tcp.json DESTINATION ${CMAKE_BINARY_DIR}/plugins/schema/)

# s7 tcp plugin
set(S7_TCP_PLUS_PLUGIN plugin-s7-tcp)
set(S7_TCP_PLUS_PLUGIN_SOURCES  s7_tcp.c
                                    ${S7_SRC})
add_library(${S7_TCP_PLUS_PLUGIN} SHARED)
target_include_directories(${S7_TCP_PLUS_PLUGIN} PRIVATE ${CMAKE_SOURCE_DIR}/include/neuron)
target_sources(${S7_TCP_PLUS_PLUGIN} PRIVATE ${S7_TCP_PLUS_PLUGIN_SOURCES})
target_link_libraries(${S7_TCP_PLUS_PLUGIN} neuron-base)

