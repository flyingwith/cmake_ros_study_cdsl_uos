#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <${APP_INTERFACE_NAME}/FibonacciAction.h>

using namespace ${APP_INTERFACE_NAME};
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const FibonacciResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->sequence.back());
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal just went active");
}

void feedbackCb(const FibonacciFeedbackConstPtr& feedback)
{
    ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "fibonacci_client_callback");

    Client ac("fibonacci", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
}