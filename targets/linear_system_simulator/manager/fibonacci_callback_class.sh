#!${APP_SHELL_EXE}
source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
source ${TARGET_INSTALL_DIR}/manager/setup.${APP_SHELL_TYPE} &&
roslaunch ${TARGET_NAME}_manager fibonacci_callback_class.launch