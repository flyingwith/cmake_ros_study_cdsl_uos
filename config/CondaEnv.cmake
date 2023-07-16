macro(install_conda_env)

# given: APP_CONDA_ENV_VER

# ------------------------------------------------------------------------------
# set CMake variables

string(REPLACE . _ APP_CONDA_ENV_NAME ${APP_CONDA_ENV_VER})
set(APP_CONDA_ENV_DIR "${APP_LIB_DIR}/${APP_CONDA_ENV_NAME}")
if(APP_CONDA_ENV_VER MATCHES "^python")
    string(REGEX REPLACE "^python[\\.]*" "" APP_PYTHON_VER "${APP_CONDA_ENV_VER}")
    set(APP_ROS_VER "none")
elseif(APP_CONDA_ENV_VER MATCHES "^ros")
    string(REGEX REPLACE "^ros[\\.]*" "" APP_ROS_VER "${APP_CONDA_ENV_VER}")
    if(APP_ROS_VER STREQUAL "noetic")
        set(APP_PYTHON_VER 3.9)
    elseif(APP_ROS_VER STREQUAL "humble")
        set(APP_PYTHON_VER 3.10)
    else()
        message(FATAL_ERROR "Invalid ROS version: ${APP_ROS_VER}")
    endif()
else()
    message(FATAL_ERROR "Invalid Conda environment version: ${APP_CONDA_ENV_VER}")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(APP_PYTHON_MODULE_EXTENSION "so")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(APP_PYTHON_MODULE_EXTENSION "pyd")
endif()

# ------------------------------------------------------------------------------
# install Python and ROS if not exist

if(NOT EXISTS "${APP_CONDA_ENV_DIR}")
    message(STATUS "NOT found ${APP_CONDA_ENV_VER}: Start to install...")
    configure_file(
        ${APP_CONFIG_DIR}/conda_env.yml
        ${APP_BINARY_DIR}/conda_env.yml
    )
    # install ONLY Python
    if(APP_ROS_VER STREQUAL "none")
        file(
            WRITE ${APP_BINARY_DIR}/conda_env_create.sh
            "#!${APP_SHELL_EXE}\n"
            "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
            "cd ${APP_LIB_DIR} &&\n"
        )
        if(CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
            file(
                APPEND ${APP_BINARY_DIR}/conda_env_create.sh
                "CONDA_SUBDIR=osx-arm64 conda env create -f ${APP_BINARY_DIR}/conda_env.yml -p ${APP_CONDA_ENV_NAME} &&\n"
            )
        else()
            file(
                APPEND ${APP_BINARY_DIR}/conda_env_create.sh
                "conda env create -f ${APP_BINARY_DIR}/conda_env.yml -p ${APP_CONDA_ENV_NAME} &&\n"
            )
        endif()
        file(
            APPEND ${APP_BINARY_DIR}/conda_env_create.sh
            "conda activate ./${APP_CONDA_ENV_NAME} &&\n"
            "conda install -y pyyaml"
        )
        execute_process(
            COMMAND chmod +x ./conda_env_create.sh
            WORKING_DIRECTORY "${APP_BINARY_DIR}"
        )
        execute_process(
            COMMAND ./conda_env_create.sh
            WORKING_DIRECTORY "${APP_BINARY_DIR}"
        )
    # install ROS and Python
    else()
        # install ROS
        file(
            WRITE ${APP_BINARY_DIR}/conda_env_create.sh
            "#!${APP_SHELL_EXE}\n"
            "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
            "cd ${APP_LIB_DIR} &&\n"
            "mamba create -y -p ${APP_CONDA_ENV_NAME} python=${APP_PYTHON_VER} -c conda-forge &&\n"
            "conda activate ./${APP_CONDA_ENV_NAME} &&\n"
            "conda config --env --add channels conda-forge &&\n"
            "conda config --env --add channels robostack-staging &&\n"
            "mamba install -y ros-${APP_ROS_VER}-desktop-full &&\n"
            "mamba install -y compilers cmake pkg-config make ninja colcon-common-extensions &&\n"
        )
        if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
            if(APP_ROS_VER STREQUAL "noetic")
                file(
                    APPEND ${APP_BINARY_DIR}/conda_env_create.sh
                    "mamba install -y catkin_tools &&\n"
                )
            endif()
        elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            file(
                APPEND ${APP_BINARY_DIR}/conda_env_create.sh
                "mamba install -y vs2019_win-64 &&\n"
            )
        else()
        endif()
        file(
            APPEND ${APP_BINARY_DIR}/conda_env_create.sh
            "conda deactivate &&\n"
            "conda activate ./${APP_CONDA_ENV_NAME} &&\n"
            "mamba install -y rosdep &&\n"
            "rosdep init &&\n"
            "rosdep update &&\n"
        )
        # update Python packages
        if(CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
            file(
                APPEND ${APP_BINARY_DIR}/conda_env_create.sh
                "CONDA_SUBDIR=osx-arm64 conda env update -f ${APP_BINARY_DIR}/conda_env.yml -p ${APP_CONDA_ENV_NAME} --prune &&\n"
            )
        else()
            file(
                APPEND ${APP_BINARY_DIR}/conda_env_create.sh
                "conda env update -f ${APP_BINARY_DIR}/conda_env.yml -p ${APP_CONDA_ENV_NAME} --prune &&\n"
            )
        endif()
        file(
            APPEND ${APP_BINARY_DIR}/conda_env_create.sh
            "conda install -y pyyaml"
        )
        execute_process(
            COMMAND chmod +x conda_env_create.sh 
            WORKING_DIRECTORY "${APP_BINARY_DIR}"
        )
        execute_process(
            COMMAND ./conda_env_create.sh
            WORKING_DIRECTORY "${APP_BINARY_DIR}"
        )
    endif()
endif()

# ------------------------------------------------------------------------------
# find Python and ROS

set(Python_ROOT_DIR "${APP_CONDA_ENV_DIR}") # Hint
set(Python_EXECUTABLE "${APP_CONDA_ENV_DIR}/bin/python${APP_PYTHON_VER}") # Artifacts Specification
find_package(Python ${APP_PYTHON_VER} REQUIRED COMPONENTS Interpreter Development)
get_filename_component(TEMP_PYTHON_BIN_DIR "${Python_EXECUTABLE}" DIRECTORY)
if(Python_FOUND AND TEMP_PYTHON_BIN_DIR STREQUAL "${APP_CONDA_ENV_DIR}/bin")
    set(APP_PYTHON_EXE "${Python_EXECUTABLE}")
    set(APP_CYTHON_EXE "${APP_CONDA_ENV_DIR}/bin/cython")
    set(APP_PYTHON_SOABI "${Python_SOABI}")
    set(APP_PYTHON_INCLUDE_DIRS "${Python_INCLUDE_DIRS}")
    set(APP_PYTHON_LIBRARY_DIRS "${Python_LIBRARY_DIRS}")
    set(APP_PYTHON_LIBRARIES "${Python_LIBRARIES}")
    set(APP_PYTHON_LINK_OPTIONS "${Python_LINK_OPTIONS}")
    if(NOT APP_ROS_VER STREQUAL "none")
        file(
            WRITE ${APP_BINARY_DIR}/check_ros_distro.sh
            "#!${APP_SHElLL_EXE}\n"
            "source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&\n"
            "cd ${APP_LIB_DIR} &&\n"
            "conda activate ./${APP_CONDA_ENV_NAME} &&\n"
            "echo $ROS_DISTRO"
        )
        execute_process(
            COMMAND chmod +x check_ros_distro.sh 
            WORKING_DIRECTORY "${APP_BINARY_DIR}"
        )
        execute_process(
            COMMAND ./check_ros_distro.sh
            WORKING_DIRECTORY "${APP_BINARY_DIR}"
            RESULT_VARIABLE ROS_VER_RESULT
            OUTPUT_VARIABLE ROS_VER_OUTPUT
        )
        if(NOT ROS_VER_OUTPUT MATCHES "${APP_ROS_VER}")
            message(FATAL_ERROR "Failed to find ROS ${APP_ROS_VER}")
        endif()
    endif()
    message(STATUS ${STRING_REPEAT_MINUS_80})
    message(STATUS "Found Python")
    message(STATUS "    APP_PYTHON_VER = ${APP_PYTHON_VER}")
    message(STATUS "    APP_ROS_VER = ${APP_ROS_VER}")
    message(STATUS "    APP_CONDA_ENV_NAME = ${APP_CONDA_ENV_NAME}")
    message(STATUS "    APP_CONDA_ENV_DIR = ${APP_CONDA_ENV_DIR}")
    message(STATUS "    APP_PYTHON_EXE = ${APP_PYTHON_EXE}")
    message(STATUS "    APP_CYTHON_EXE = ${APP_CYTHON_EXE}")
    message(STATUS "    APP_PYTHON_SOABI = ${APP_PYTHON_SOABI}")
    message(STATUS "    APP_PYTHON_MODULE_EXTENSION = ${APP_PYTHON_MODULE_EXTENSION}")
    message(STATUS "    APP_PYTHON_INCLUDE_DIRS = ${APP_PYTHON_INCLUDE_DIRS}")
    message(STATUS "    APP_PYTHON_LIBRARY_DIRS = ${APP_PYTHON_LIBRARY_DIRS}")
    message(STATUS "    APP_PYTHON_LIBRARIES = ${APP_PYTHON_LIBRARIES}")
    message(STATUS "    APP_PYTHON_LINK_OPTIONS = ${APP_PYTHON_LINK_OPTIONS}")
    message(STATUS ${STRING_REPEAT_MINUS_80})
else()
    message(FATAL_ERROR "Failed to find Python ${APP_PYTHON_VER}")
endif()

endmacro()