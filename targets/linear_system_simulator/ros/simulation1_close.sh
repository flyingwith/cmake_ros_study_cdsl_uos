#!${APP_SHELL_EXE}
source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
source ${TARGET_INSTALL_DIR}/ros/setup.${APP_SHELL_TYPE} &&
if [ -n "$(rosnode list | grep simulation1_server)" ]
then
    rosnode kill simulation1_server &&
fi
if [ -n "$(rosnode list | grep simulation1_client)" ]
then
    rosnode kill simulation1_client &&
fi
killall roscore &&
killall rosmaster &&
killall rosout