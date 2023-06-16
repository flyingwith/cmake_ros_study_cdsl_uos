cmake_minimum_required(VERSION 3.0.2)
project(@APP_NAME@)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # message_generation
)

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ==============================================================================
# C++ Nodes

set(CPP_NODES
    # Topics
    chatter_talker
    chatter_listener
    # Services
    # add_two_ints_server
    # add_two_ints_client
    # Actions
    # fibonacci_server
    # fibonacci_client
    # fibonacci_client_callback
    # fibonacci_client_callback_class
    # averaging_server
    # averaging_client
)
foreach(NODE ${CPP_NODES})
    add_executable(${NODE}
        src/${NODE}.cpp
    )
    target_link_libraries(${NODE}
        ${catkin_LIBRARIES}
    )
    # add_dependencies(${NODE}
    #     ${PROJECT_NAME}_generate_messages_cpp
    # )
    install(
        TARGETS ${NODE}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()

# ==============================================================================
# Extra Files

install(FILES
    # launch/chatter.launch
    launch/chatter_cpp.launch
    # launch/chatter_py.launch
    # launch/add_two_ints.launch
    # launch/add_two_ints_cpp.launch
    # launch/add_two_ints_py.launch
    # launch/fibonacci_cpp.launch
    # launch/fibonacci_py.launch
    # launch/fibonacci.launch
    # launch/fibonacci_callback.launch
    # launch/fibonacci_callback_class.launch
    # launch/averaging.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
    test/chatter
    # test/add_two_ints
    # test/fibonacci
    # test/fibonacci_callback
    # test/fibonacci_callback_class
    DESTINATION ${CMAKE_INSTALL_PREFIX}/test
)