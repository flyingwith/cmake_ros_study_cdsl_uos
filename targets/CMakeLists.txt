foreach(DNAME ${APP_TARGET_DIR_NAMES})
    add_subdirectory(${DNAME})
endforeach()

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/targets.py"
    "${CMAKE_CURRENT_BINARY_DIR}/targets.py"
)
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/targets.py"
    DESTINATION "${APP_INSTALL_DIR}/targets"
)