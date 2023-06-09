#! /usr/bin/env python
import rospy
import actionlib
import ${APP_NAME}.msg

def fibonacci_client():
    client = actionlib.SimpleActionClient('fibonacci', ${APP_NAME}.msg.FibonacciAction)
    client.wait_for_server()

    goal = ${APP_NAME}.msg.FibonacciGoal(order=20)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)