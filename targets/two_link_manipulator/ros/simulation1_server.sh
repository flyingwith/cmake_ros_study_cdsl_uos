#!${APP_SHELL_EXE}
source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
source ${TARGET_INSTALL_DIR}/ros/setup.${APP_SHELL_TYPE} &&
if [ -z "$(rosnode list | grep simulation1_server)" ]
then
    roslaunch --wait ${TARGET_ROS_NAME} simulation1_server.launch
fi