set(APP_INTERFACE_NAME ${APP_NAME}_interface)
set(APP_INTERFACE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(APP_INTERFACE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(APP_INTERFACE_INSTALL_DIR ${APP_INSTALL_DIR}/interface)

set(FILES
    interface.py
    interface_template.ipynb
)
foreach(FILE ${FILES})
    configure_file(
        ${APP_INTERFACE_SOURCE_DIR}/${FILE}
        ${APP_INTERFACE_BINARY_DIR}/${FILE}
    )
    install(
        FILES ${APP_INTERFACE_BINARY_DIR}/${FILE}
        DESTINATION ${APP_INTERFACE_INSTALL_DIR}
    )
endforeach()

add_subdirectory(test_interface)

# ==============================================================================
# Launch Script

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    file(WRITE "${APP_INTERFACE_BINARY_DIR}/interface"
"\
#!${APP_SHELL_EXE}
source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
cd $(dirname \"$0\") &&
cp interface_template.ipynb interface.ipynb &&
jupyter notebook interface.ipynb
"
    )
    execute_process(
        COMMAND chmod +x "interface" 
        WORKING_DIRECTORY "${APP_INTERFACE_BINARY_DIR}"
    )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
else()
endif()

install(
    PROGRAMS "${APP_INTERFACE_BINARY_DIR}/interface"
    DESTINATION "${APP_INTERFACE_INSTALL_DIR}"
)