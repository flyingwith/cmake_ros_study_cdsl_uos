#! /usr/bin/env python

import os, sys, numpy as np, pandas as pd
import rospy
import actionlib
import ${TARGET_ROS_NAME}.msg

sys.path.append('${APP_LIB_DIR}/tools')
from py_xmlreader import PyXmlReader

class Simulation1Client:
    def __init__(self, data_dir_name):
        self.data_dir_name = data_dir_name
        self.xml = PyXmlReader(f'${APP_DATA_DIR}/{self.data_dir_name}/parameter.xml')
        self.client = actionlib.SimpleActionClient('simulation1', ${TARGET_ROS_NAME}.msg.simulation1Action)
        self.client.wait_for_server()

    def runSimulation(self):
        goal = ${TARGET_ROS_NAME}.msg.simulation1Goal(parameter=self.xml.readXml())
        self.client.send_goal(
            goal, 
            active_cb = self.activeCallback, 
            feedback_cb = self.feedbackCallback,
            done_cb = self.doneCallback
        )
        self.client.wait_for_result()

    def activeCallback(self):
        # rospy.loginfo(f'The goal is sent.')
        pass

    def feedbackCallback(self, feedback):
        # rospy.loginfo(f'The feedback is received: message = {feedback.message}')
        pass

    def doneCallback(self, state, result):
        # rospy.loginfo(f'The result is received: state = {state}, message = {result.message}')
        N = len(result.itr)
        n = int(self.xml.getData('param.simulation1.n'))
        m = int(self.xml.getData('param.simulation1.m'))
        p = int(self.xml.getData('param.simulation1.p'))
        itr = np.array(result.itr)
        t = np.array(result.t)
        x = np.array(result.x).reshape((N,n))
        y = np.array(result.y).reshape((N,p))
        r = np.array(result.r).reshape((N,p))
        e = np.array(result.e).reshape((N,p))
        de = np.array(result.de).reshape((N,p))
        ie = np.array(result.ie).reshape((N,p))
        u = np.array(result.u).reshape((N,m))
        df = pd.DataFrame()
        df['itr'] = itr
        df['t'] = t
        for i in range(n):
            df[f'x{i}'] = x[:,i]
        for i in range(p):
            df[f'y{i}'] = y[:,i]
        for i in range(p):
            df[f'r{i}'] = r[:,i]
        for i in range(p):
            df[f'e{i}'] = e[:,i]
        for i in range(p):
            df[f'de{i}'] = de[:,i]
        for i in range(p):
            df[f'ie{i}'] = ie[:,i]
        for i in range(m):
            df[f'u{i}'] = u[:,i]
        # print(df)
        data_file_path_name = f'${APP_DATA_DIR}/{self.data_dir_name}/data.csv'
        df.to_csv(data_file_path_name)
        with open(f'{data_file_path_name}.ready', 'x') as f:
            f.write('')

if __name__ == '__main__':
    if len(sys.argv)!=2 and len(sys.argv)!=4:
        rospy.loginfo(f'usage: {sys.argv[0]} data_dir_name')
        sys.exit(1)
    else:
        data_dir_name = sys.argv[1]

    try:
        rospy.init_node('simulation1_client')
        sim = Simulation1Client(data_dir_name)
        sim.runSimulation()
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)