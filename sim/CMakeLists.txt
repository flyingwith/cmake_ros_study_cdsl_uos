add_library(simulation simulation.cpp)
target_include_directories(simulation
    PUBLIC "${APP_INCLUDE_DIRS}"
)
target_link_directories(simulation
    PUBLIC "${APP_LIBRARY_DIRS}"
)
target_link_libraries(simulation
    PUBLIC "${APP_LIBRARIES}"
)
set_target_properties(simulation PROPERTIES
    COMPILE_OPTIONS "${APP_COMPILE_OPTIONS}"
    LINK_OPTIONS "${APP_LINK_OPTIONS}"
)