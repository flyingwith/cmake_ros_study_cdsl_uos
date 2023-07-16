# -*- coding: utf-8 -*-

import os, sys, pathlib, importlib
from pathlib import Path
from types import MappingProxyType

import rospy, rostopic

cfd = os.path.dirname(os.path.realpath(__file__)) # current file directory
cwd = os.getcwd() # current working directory

sys.path.append(str(pathlib.Path(cfd).parent.parent / 'library' / 'tools'))
from py_xmlreader import PyXmlReader

sys.path.append(str(Path(cfd).parent / 'targets'))
from targets import Targets

class ControlInterface:
    # -------------------------------------------------------------------------
    
    TARGETS = Targets(str(Path(cfd).parent/'targets'))

    # -------------------------------------------------------------------------

    def is_ros_running():
        try:
            rostopic.get_topic_class('/rosout')
            return True
        except rostopic.ROSTopicIOException as e:
            return False

    # -------------------------------------------------------------------------

    def list_all_targets():
        print('List of all targets:')
        print("    id / 'text name' / 'directory name'")
        for i in range(ControlInterface.TARGETS.MAX_ID):
            print(f"    {i} / '{ControlInterface.TARGETS.TEXT_NAME[i]}' / '{ControlInterface.TARGETS.DIR_NAME[i]}'")

    # -------------------------------------------------------------------------

    def get_target_info(target):
        if isinstance(target, int) and target >= 0 and target < ControlInterface.TARGETS.MAX_ID:
            id = target
        elif isinstance(target, str) and target in ControlInterface.TARGETS.TEXT_NAME:
            id = ControlInterface.TARGETS.TEXT_NAME.index(target)
        elif isinstance(target, str) and target in ControlInterface.TARGETS.DIR_NAME:
            id = ControlInterface.TARGETS.DIR_NAME.index(target)
        else:
            id = None
        if id != None:
            text_name = ControlInterface.TARGETS.TEXT_NAME[id]
            dir_name = ControlInterface.TARGETS.DIR_NAME[id]
        else:
            text_name = None
            dir_name = None
        return id, text_name, dir_name
    
    # -------------------------------------------------------------------------

    def print_xml(target):
        id, text_name, dir_name = ControlInterface.get_target_info(target)
        if dir_name != None:
            print(PyXmlReader(str(Path(cfd).parent / 'targets' / dir_name)+'/parameter.xml').readXml())
        else:
            print('invalid target')

    # -------------------------------------------------------------------------
    
    def __init__(self, target, *arguments, **keywords):
        if not ControlInterface.is_ros_running():
            raise RuntimeError('ROS is not running')
        self.target_id, self.target_text_name, self.target_dir_name = ControlInterface.get_target_info(target)
        if self.target_id == None:
            raise RuntimeError(f'Failed to find the target for {target}. Find the target id or the text name or the directory name by running list_all_targets().')
        self.target_dir = str(Path(cfd).parent / 'targets' / self.target_dir_name)
        sys.path.append(self.target_dir)
        self.target = importlib.import_module(self.target_dir_name).Target(arguments, keywords)

    # -------------------------------------------------------------------------

    def __del__(self):
        if hasattr(self, 'target'):
            del self.target

    # -------------------------------------------------------------------------

    def run(self, *arguments, **keywords):
        if hasattr(self, 'target'):
            self.target.run(arguments, keywords)
        else:
            raise RuntimeError('self.target does not exists')