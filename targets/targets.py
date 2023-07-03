# -*- coding: utf-8 -*-

import subprocess, psutil, os
from pathlib import Path

class Targets:
    def __init__(self, targets_root_dir=None):
        if targets_root_dir == None:
            self.targets_root_dir = str(Path('${APP_BINARY_DIR}')/'targets')
        else:
            self.targets_root_dir = targets_root_dir
        ##@brief total number of targets
        self.MAX_ID = ${APP_TARGET_TOTAL_NUMBER}
        ##@brief text name of targets
        self.TEXT_NAME = '${APP_TARGET_TEXT_NAMES}'.split(';')
        ##@brief directory name of targets
        self.DIR_NAME = '${APP_TARGET_DIR_NAMES}'.split(';')
        ##@brief xml file name with its relative path
        self.PARAMETER_XML_PATH_NAME = []
        for i in range(self.MAX_ID):
            self.PARAMETER_XML_PATH_NAME.append(
                str(Path(self.targets_root_dir)/
                    self.DIR_NAME[i]/
                    'parameter.xml'
                )
            )
    def openProgram(self, programId):
        if programId < 0 or programId >= self.MAX_ID:
            return False
        cmd = str(Path(self.targets_root_dir)/
                  self.DIR_NAME[programId]/
                  'main'/
                  (self.DIR_NAME[programId]+'_main')
              )
        if os.name == 'nt':
            cmd += '.exe'
        try:
            subprocess.Popen(cmd)
        except:
            return False
        return True
    def closeProgram(self, programId):
        if programId < 0 or programId >= self.MAX_ID:
            return False
        ret = False
        proc_name_main = self.DIR_NAME[programId]+'_main'
        proc_name_task = self.DIR_NAME[programId]+'_task'
        if os.name == 'nt':
            proc_name_main += '.exe'
            proc_name_task += '.exe'
        proc_name = [proc_name_main, proc_name_task]
        for proc in psutil.process_iter():
            if proc.status() != psutil.STATUS_ZOMBIE and proc.name() in proc_name:
                try:
                    proc.kill()
                    ret = True
                except:
                    return False
        return ret