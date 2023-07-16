# -*- coding: utf-8 -*-

import pathlib, sys, os, time

# install
# |- interface
# |  |- interface.py
# |  |- test_interface
# |     |- test_interface.py

sys.path.append(str(pathlib.Path(os.getcwd()).parent))
from interface import ControlInterface as CI

def main():
    # =========================================================================
    print('\n'+'='*80+'\nwait for ROS to start\n'+'='*80+'\n')

    while(not CI.is_ros_running()):
        print('ROS is not running')
        time.sleep(1)
    print('ROS is running')

    # =========================================================================
    print('\n'+'='*80+'\ninitialize interface\n'+'='*80+'\n')

    print('\n<-- list all targets -->\n')

    CI.list_all_targets()

    print('\n<-- get target info -->\n')

    id, text_name, dir_name = CI.get_target_info(0)
    print(f'with id argument:\n    id = {id}, text_name = {text_name}, dir_name = {dir_name}')

    id, text_name, dir_name = CI.get_target_info('Linear System Simulator')
    print(f'with text name argument:\n    id = {id}, text_name = {text_name}, dir_name = {dir_name}')

    id, text_name, dir_name = CI.get_target_info('linear_system_simulator')
    print(f'with dir name argument:\n    id = {id}, text_name = {text_name}, dir_name = {dir_name}')

    print('\n<-- print parameter.xml -->\n')

    CI.print_xml(0)

    print('\n<-- instantiate and delete interface -->\n')

    time_for_shutdown = 0.4
    try:
        print('with id')
        ci = CI(0)
        del ci
        time.sleep(time_for_shutdown)
        print('with text name')
        ci = CI('Linear System Simulator')
        del ci
        time.sleep(time_for_shutdown)
        print('with directory name')
        ci = CI('linear_system_simulator')
        del ci
        time.sleep(time_for_shutdown)
        print('with invalid input')
        ci = CI(-1)
        del ci
        time.sleep(time_for_shutdown)
    except RuntimeError as e:
        print('Failed to instantiate the interface: ', e)

    print('\n<-- test the key verbose -->\n')

    try:
        ci = CI(0, verbose=True)
        del ci
        time.sleep(time_for_shutdown)
    except RuntimeError as e:
        print(e)

if __name__ == '__main__':
    main()