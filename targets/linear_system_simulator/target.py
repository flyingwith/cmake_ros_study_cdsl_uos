# -*- coding: utf-8 -*-

import os, sys, pathlib, datetime, shutil, subprocess, shlex, time, pandas as pd, numpy as np

cfd = os.path.dirname(os.path.realpath(__file__))
sys.path.append(str(pathlib.Path(cfd).parent.parent / 'library' / 'tools'))
from py_xmlreader import *
from py_mathtools import *

class Target:
    def __init__(self, arguments, keywords):
        self.xml = PyXmlReader(f'{cfd}/parameter.xml')
        if len(arguments) == 0:
            self.launch = self.xml.getData('param.info.<xmlattr>.launch')
        else:
            self.launch = arguments[0]

        self.verbose = False
        if 'verbose' in keywords:
            self.verbose = keywords['verbose']
            del keywords['verbose']
        
        if self.verbose == True:
            print(f'{self.launch}: initializing...')

        if self.launch == 'simulation1':
            self.simulation1_init()
        else:
            raise RuntimeError('invalid argument for launch')
        
        if self.verbose == True:
            print(f'{self.launch}: initialization done')

        if 'run' in keywords and keywords['run'] == True:
            del keywords['run']
            self.run(arguments, keywords)
        
    def __del__(self):
        if self.verbose == True:
            print(f'{self.launch}: finalizing...')

        if self.launch == 'simulation1':
            self.simulation1_del()
        
        if self.verbose == True:
            print(f'{self.launch}: finalization done')

    def run(self, arguments, keywords):
        if self.verbose == True:
            print(f'{self.launch}: running...')

        if self.launch == 'simulation1':
            self.simulation1_run(arguments, keywords)
        
        if self.verbose == True:
            print(f'{self.launch}: run done')

    # =========================================================================
    # simulation1
    # =========================================================================

    def simulation1_init(self):
        # run simulation1_server
        if self.verbose == True:
            subprocess.Popen(shlex.split(f'{cfd}/ros/test/simulation1_server'))
        else:
            subprocess.Popen(
                shlex.split(f'{cfd}/ros/test/simulation1_server'),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT
            )

    # -------------------------------------------------------------------------

    def simulation1_del(self):
        # close simulation1_server and simulation1_client
        if self.verbose == True:
            subprocess.Popen(shlex.split(f'{cfd}/ros/test/simulation1_close'))
        else:
            subprocess.Popen(
                shlex.split(f'{cfd}/ros/test/simulation1_close'),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT
            )
    
    # -------------------------------------------------------------------------

    def simulation1_run(self, arguments, keywords):
        # create data directory
        if 'data_dir_name' in keywords:
            self.data_dir_name = keywords['data_dir_name']
            del keywords['data_dir_name']
        else:
            self.data_dir_name = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_${TARGET_NAME}')
        self.data_dir = pathlib.Path('${APP_DATA_DIR}') / self.data_dir_name
        if os.path.exists(self.data_dir) == True:
            try:
                shutil.rmtree(self.data_dir)
            except OSError as e:
                print("Error: %s - %s." % (e.filename, e.strerror))
        pathlib.Path(self.data_dir).mkdir(parents=True, exist_ok=True)
        
        # update parameter.xml
        for key in keywords:
            value = keywords[key]
            if isinstance(value, np.ndarray):
                self.xml.putData(f'param.simulation1.{key}', atos(value))
            elif isinstance(value, tuple) or isinstance(value, list) or isinstance(dict, list):
                s = ''
                for i in range(len(value)):
                    s += f'{value[i]}'
                    if i < len(value)-1:
                        s += ','
                self.xml.putData(f'param.simulation1.{key}', s)
            else:
                self.xml.putData(f'param.simulation1.{key}', f'{value}')
        self.xml.saveFile(f'{self.data_dir}/parameter.xml')
        
        # run simulation1_client
        if self.verbose == True:
            subprocess.Popen(shlex.split(f'{cfd}/ros/test/simulation1_client {self.data_dir_name}'))
        else:
            subprocess.Popen(
                shlex.split(f'{cfd}/ros/test/simulation1_client {self.data_dir_name}'),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT
            )
        
        # read data file as the pandas dataframe
        data_file_path_name = f'{self.data_dir}/data.csv'
        while not os.path.exists(f'{data_file_path_name}.ready'):
            time.sleep(0.1)
        if os.path.exists(data_file_path_name) and os.path.isfile(data_file_path_name):
            self.df = pd.read_csv(data_file_path_name)
        else:
            raise ValueError(f'{data_file_path_name} is not ready nor a file!')
        
        # extract data from the dataframe
        self.n = int(self.xml.getData('param.simulation1.n'))
        self.m = int(self.xml.getData('param.simulation1.m'))
        self.p = int(self.xml.getData('param.simulation1.p'))
        i = 1
        self.itr = np.squeeze(self.df.iloc[:,i:i+1].to_numpy())
        self.N = len(self.itr)
        i += 1
        self.t = np.squeeze(self.df.iloc[:,i:i+1].to_numpy())
        i += 1
        self.x = np.squeeze(self.df.iloc[:,i:i+self.n].to_numpy())
        i += self.n
        self.y = np.squeeze(self.df.iloc[:,i:i+self.p].to_numpy())
        i += self.p
        self.r = np.squeeze(self.df.iloc[:,i:i+self.p].to_numpy())
        i += self.p
        self.e = np.squeeze(self.df.iloc[:,i:i+self.p].to_numpy())
        i += self.p
        self.de = np.squeeze(self.df.iloc[:,i:i+self.p].to_numpy())
        i += self.p
        self.ie = np.squeeze(self.df.iloc[:,i:i+self.p].to_numpy())
        i += self.p
        self.u = np.squeeze(self.df.iloc[:,i:i+self.m].to_numpy())
        i += self.m