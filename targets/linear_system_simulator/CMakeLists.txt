get_filename_component(SOURCE_DIR_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
string(REPLACE " " "_" SOURCE_DIR_NAME "${SOURCE_DIR_NAME}")

set(TARGET_NAME "${SOURCE_DIR_NAME}")

list(FIND APP_TARGET_DIR_NAMES ${TARGET_NAME} TARGET_INDEX)
if(${TARGET_INDEX} MATCHES "-1")
    message(FATAL_ERROR "Failed to find the target index!")
endif()
list(GET APP_TARGET_TEXT_NAMES ${TARGET_INDEX} TARGET_TEXT_NAME)
# message(STATUS "TARGET_NAME = ${TARGET_NAME}")
# message(STATUS "TARGET_TEXT_NAME = ${TARGET_TEXT_NAME}")

set(TARGET_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TARGET_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(TARGET_INSTALL_DIR ${APP_INSTALL_DIR}/targets/${TARGET_NAME})

# ==============================================================================
# Manager Setup

# ------------------------------------------------------------------------------
# create catkin_workspace and ros package

if(NOT EXISTS ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager)
    file(
        WRITE ${TARGET_BINARY_DIR}/ros_package_create_manager.sh
        "#!${APP_SHELL_EXE}\n"
        "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
        "conda activate ${APP_CONDA_ENV_DIR} &&\n"
        "mkdir -p ${TARGET_BINARY_DIR}/manager/src &&\n"
        "cd ${TARGET_BINARY_DIR}/manager &&\n"
        "catkin_make &&\n"
        "source devel/setup.${APP_SHELL_TYPE} &&\n"
        "cd src &&\n"
        "catkin_create_pkg ${TARGET_NAME}_manager std_msgs rospy roscpp\n"
    )
    execute_process(
        COMMAND chmod +x ros_package_create_manager.sh
        WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
    )
    execute_process(
        COMMAND ./ros_package_create_manager.sh
        WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
    )
    file(REMOVE_RECURSE "${TARGET_BINARY_DIR}/manager/build")
    file(MAKE_DIRECTORY "${TARGET_BINARY_DIR}/manager/build")
endif()

# ------------------------------------------------------------------------------
# copy files

file(GLOB_RECURSE FILE_LIST CONFIGURE_DEPENDS "manager/*")
foreach(FILE ${FILE_LIST})
    get_filename_component(FILE_DIR ${FILE} DIRECTORY)
    get_filename_component(FILE_NAME ${FILE} NAME)
    get_filename_component(FILE_NAME_WLE ${FILE} NAME_WLE)
    get_filename_component(FILE_LAST_EXT ${FILE} LAST_EXT)
    # message("FILE = ${FILE}")
    # message("     = ${FILE_DIR}/${FILE_NAME_WLE}${FILE_LAST_EXT}")
    if(FILE_NAME MATCHES "CMakeLists.txt" OR FILE_NAME MATCHES "package.xml")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/${FILE_NAME}
            @ONLY
        )
    elseif(FILE_NAME MATCHES ".msg$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/msg/${FILE_NAME}
        )
    elseif(FILE_NAME MATCHES ".srv$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/srv/${FILE_NAME}
        )
    elseif(FILE_NAME MATCHES ".action$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/action/${FILE_NAME}
        )
    elseif(FILE_NAME MATCHES ".cpp$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/src/${FILE_NAME}
        )
    elseif(FILE_NAME MATCHES ".py$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/scripts/${FILE_NAME}
        )
        execute_process(
            COMMAND chmod +x ${FILE_NAME}
            WORKING_DIRECTORY ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/scripts
        )
    elseif(FILE_NAME MATCHES ".launch$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/launch/${FILE_NAME}
        )
        execute_process(
            COMMAND chmod +x ${FILE_NAME}
            WORKING_DIRECTORY ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/launch
        )
    elseif(FILE_NAME MATCHES ".sh$")
        configure_file(
            ${FILE}
            ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/test/${FILE_NAME_WLE}
        )
        execute_process(
            COMMAND chmod +x ${FILE_NAME_WLE}
            WORKING_DIRECTORY ${TARGET_BINARY_DIR}/manager/src/${TARGET_NAME}_manager/test
        )
    else()
        message("Unregistered file type: ${FILE_NAME}")
    endif()
endforeach()

# ------------------------------------------------------------------------------
# configure the ROS package

file(
    WRITE ${TARGET_BINARY_DIR}/ros_package_configure_manager.sh
    "#!${APP_SHELL_EXE}\n"
    "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
    "conda activate ${APP_CONDA_ENV_DIR} &&\n"
    "cd ${TARGET_BINARY_DIR}/manager/build &&\n"
    "cmake ../src -DCMAKE_INSTALL_PREFIX=${TARGET_INSTALL_DIR}/manager -DCATKIN_DEVEL_PREFIX=../devel"
)
execute_process(
    COMMAND chmod +x ros_package_configure_manager.sh
    WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
)
# execute_process(
#     COMMAND ./ros_package_configure_manager.sh
#     WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
# )
# Running the shell script with execute_process generates build errors.
add_custom_target(${TARGET_NAME}_ros_config_manager ALL
    COMMAND ./ros_package_configure_manager.sh
    WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
)

# ------------------------------------------------------------------------------
# build the ROS package

file(
    WRITE ${TARGET_BINARY_DIR}/ros_package_build_manager.sh
    "#!${APP_SHELL_EXE}\n"
    "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
    "conda activate ${APP_CONDA_ENV_DIR} &&\n"
    "cd ${TARGET_BINARY_DIR}/manager/build &&\n"
    "make"
)
execute_process(
    COMMAND chmod +x ros_package_build_manager.sh
    WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
)
add_custom_target(${TARGET_NAME}_ros_build_manager ALL
    COMMAND ./ros_package_build_manager.sh
    WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
)
add_dependencies(
    ${TARGET_NAME}_ros_build_manager
    ${TARGET_NAME}_ros_config_manager
)

# ------------------------------------------------------------------------------
# install the ROS package

file(
    WRITE ${TARGET_BINARY_DIR}/ros_package_install_manager.sh
    "#!${APP_SHELL_EXE}\n"
    "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
    "conda activate ${APP_CONDA_ENV_DIR} &&\n"
    "cd ${TARGET_BINARY_DIR}/manager/build &&\n"
    "make install"
)
execute_process(
    COMMAND chmod +x ros_package_install_manager.sh
    WORKING_DIRECTORY "${TARGET_BINARY_DIR}"
)
install(
    CODE "execute_process(COMMAND ./ros_package_install_manager.sh WORKING_DIRECTORY ${TARGET_BINARY_DIR})"
)