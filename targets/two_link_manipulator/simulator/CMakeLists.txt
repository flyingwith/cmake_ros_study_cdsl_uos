# ==============================================================================
# simulation1

add_library(${TARGET_NAME}_simulation1 simulation1.cpp)
target_include_directories(${TARGET_NAME}_simulation1
    PUBLIC "${TARGET_INCLUDE_DIRS}"
)
target_link_directories(${TARGET_NAME}_simulation1
    PUBLIC "${TARGET_LIBRARY_DIRS}"
)
target_link_libraries(${TARGET_NAME}_simulation1
    PUBLIC "${TARGET_LIBRARIES}"
)
set_target_properties(${TARGET_NAME}_simulation1 PROPERTIES
    COMPILE_OPTIONS "${TARGET_COMPILE_OPTIONS}"
    LINK_OPTIONS "${TARGET_LINK_OPTIONS}"
)

install(
    FILES "${TARGET_SOURCE_DIR}/simulator/simulation1.h"
    DESTINATION "${TARGET_INSTALL_DIR}/simulator"
)
install(
    TARGETS ${TARGET_NAME}_simulation1
    DESTINATION "${TARGET_INSTALL_DIR}/simulator"
)

add_subdirectory(test_simulator)