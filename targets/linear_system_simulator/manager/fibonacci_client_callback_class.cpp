#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <${TARGET_NAME}_manager/FibonacciAction.h>

using namespace ${TARGET_NAME}_manager;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

class MyNode
{
public:
    MyNode() : ac("fibonacci", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");
    }

    void doStuff(int order)
    {
        FibonacciGoal goal;
        goal.order = order;

        ac.sendGoal(goal,
                    boost::bind(&MyNode::doneCb, this, _1, _2),
                    Client::SimpleActiveCallback(),
                    Client::SimpleFeedbackCallback());
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const FibonacciResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Answer: %i", result->sequence.back());
        ros::shutdown();
    }

private:
    Client ac;
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "fibonacci_client_callback_class");
    MyNode my_node;
    my_node.doStuff(20);
    ros::spin();
    return 0;
}