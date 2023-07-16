set(FILE_LIST
    xmlreader.pxd
    py_xmlreader.pyx
    mathtools.pxd
    py_mathtools.pyx
    setup.py
)
foreach(FILE ${FILE_LIST})
    configure_file(
        "${APP_TOOLS_SOURCE_DIR}/${FILE}"
        "${APP_TOOLS_BINARY_DIR}/${FILE}"
    )
endforeach()

add_custom_target(${APP_NAME}_tools ALL
    COMMAND ${APP_PYTHON_EXE} setup.py build_ext --build-lib=${APP_TOOLS_BINARY_DIR}
    WORKING_DIRECTORY "${APP_TOOLS_BINARY_DIR}"
)

install(
    DIRECTORY "${APP_TOOLS_BINARY_DIR}/"
    DESTINATION "${APP_TOOLS_INSTALL_DIR}"
    FILES_MATCHING 
        PATTERN "*.${APP_PYTHON_SOABI}.${APP_PYTHON_MODULE_EXTENSION}"
        PATTERN "build" EXCLUDE
        PATTERN "CMakeFiles" EXCLUDE
)

add_subdirectory(test_xmlreader)
add_subdirectory(test_mathtools)