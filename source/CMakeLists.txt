find_package(Boost)

find_package( Threads REQUIRED )

add_executable(serial_adapter serial_adapter.cc)
target_link_libraries(serial_adapter Boost::boost ${CMAKE_THREAD_LIBS_INIT})
