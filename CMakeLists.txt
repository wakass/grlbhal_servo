 add_library(servo INTERFACE)

target_sources(servo INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/servo.c
 ${CMAKE_CURRENT_LIST_DIR}/bltouch.c
)

target_include_directories(servo INTERFACE ${CMAKE_CURRENT_LIST_DIR})
