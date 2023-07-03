#!${APP_SHELL_EXE}
source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
source ${APP_INTERFACE_INSTALL_DIR}/ros/setup.${APP_SHELL_TYPE} &&
roslaunch ${APP_INTERFACE_NAME} fibonacci_callback_class.launch