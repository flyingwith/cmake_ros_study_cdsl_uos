#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from ${TARGET_NAME}_manager.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    # print(f'sys.argv = {sys.argv}')
    # When a python node starts by roslaunch, there are two additional arguments:
    # __name:=<node_name>
    # __log:=<log file>
    if len(sys.argv) == 3 or len(sys.argv) == 5:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)

    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))