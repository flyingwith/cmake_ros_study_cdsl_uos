#!${APP_SHELL_EXE}
timestamp() {
    date +"%Y%m%d_%H%M%S"
}

if [ $# -eq 0 ]
then
    data_dir_name=$(timestamp)_${TARGET_NAME} &&
    mkdir -p ${APP_DATA_DIR}/$data_dir_name &&
    cp ${TARGET_INSTALL_DIR}/parameter.xml ${APP_DATA_DIR}/$data_dir_name/parameter.xml &&
else
    data_dir_name=$1
fi

source ${APP_CONDA_DIR}/etc/profile.d/conda.sh &&
conda activate ${APP_CONDA_ENV_DIR} &&
source ${TARGET_INSTALL_DIR}/ros/setup.${APP_SHELL_TYPE} &&
roslaunch ${TARGET_ROS_NAME} simulation1_test.launch data_dir_name:=$data_dir_name