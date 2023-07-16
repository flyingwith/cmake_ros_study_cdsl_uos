macro(generate_shell_scripts)

set(APP_SHELL_SCRIPTS "")

# ------------------------------------------------------------------------------
# run roscore

set(SCRIPT "run_roscore")
set(APP_SHELL_SCRIPTS ${APP_SHELL_SCRIPTS} "${SCRIPT}")
file(WRITE "${APP_BINARY_DIR}/scripts/${SCRIPT}" "\
#!${APP_SHELL_EXE}
source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
roscore
")

# ------------------------------------------------------------------------------
# install

foreach(SCRIPT ${APP_SHELL_SCRIPTS})
    execute_process(
        COMMAND chmod +x ${SCRIPT}
        WORKING_DIRECTORY "${APP_BINARY_DIR}/scripts"
    )
    install(
        PROGRAMS "${APP_BINARY_DIR}/scripts/${SCRIPT}"
        DESTINATION "${APP_INSTALL_DIR}/scripts"
    )
endforeach()

endmacro()