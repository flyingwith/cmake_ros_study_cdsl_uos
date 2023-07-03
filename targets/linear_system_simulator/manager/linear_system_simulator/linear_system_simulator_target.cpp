#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "${TARGET_NAME}_manager/report_t.h"

#include "simulation.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_system_simulator_target");
    ros::NodeHandle n;
    ros::Publisher target = n.advertise<${TARGET_NAME}_manager::report_t>("report", 1000);
    ros::Rate loop_rate(1000); // Hz

    int count = 0;
    while (ros::ok())
    {







        // std_msgs::String msg;
        // std::stringstream ss;
        // ss << "hello world " << count;
        // msg.data = ss.str();
        // ROS_INFO("%s", msg.data.c_str());
        // target.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}