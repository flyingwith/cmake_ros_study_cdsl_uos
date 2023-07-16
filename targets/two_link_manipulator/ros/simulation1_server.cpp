#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <${TARGET_ROS_NAME}/simulation1Action.h>

#include "simulation1.h"

using namespace APP_NAMESPACE;
using namespace APP_NAMESPACE::TARGET_NAMESPACE;

class Simulation1Server
{
public:
    Simulation1Server(std::string name) 
        : m_server(m_node, name, boost::bind(&Simulation1Server::callback, this, _1), false)
        , m_actionName(name)
    {
        m_server.start();
    }

    ~Simulation1Server(void)
    {
    }

    void callback(const ${TARGET_ROS_NAME}::simulation1GoalConstPtr &goal)
    {
        bool success = true;
        Simulation1* sim = new Simulation1(goal->parameter.c_str(), Simulation1::MODE_STRING);
        clear_result();
        push_result(sim);

        while (sim->getStatus() != Simulation1::STATUS_FINE) {
            if (m_server.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", m_actionName.c_str());
                m_server.setPreempted();
                success = false;
                break;
            }
            sim->update();
            push_result(sim);
            m_feedback.message = m_actionName + ": itr = " + std::to_string(sim->itr);
            m_server.publishFeedback(m_feedback);
        }
        
        if(success) {
            m_result.message = m_actionName + ": completed";
            m_server.setSucceeded(m_result);
        }

        delete sim;
    }

    void clear_result()
    {
        m_result.itr.clear();
        m_result.t.clear();
        m_result.q.clear();
        m_result.dq.clear();
        m_result.r.clear();
        m_result.e.clear();
        m_result.de.clear();
        m_result.v.clear();
        m_result.tau.clear();
    }

    void push_result(Simulation1* sim)
    {
        m_result.itr.push_back(sim->itr);
        m_result.t.push_back(sim->t);
        for (int i=0; i<2; i++) m_result.q.push_back(sim->q[i]);
        for (int i=0; i<2; i++) m_result.dq.push_back(sim->dq[i]);
        for (int i=0; i<2; i++) m_result.r.push_back(sim->r[i]);
        for (int i=0; i<2; i++) m_result.e.push_back(sim->e[i]);
        for (int i=0; i<2; i++) m_result.de.push_back(sim->de[i]);
        for (int i=0; i<2; i++) m_result.v.push_back(sim->v[i]);
        for (int i=0; i<2; i++) m_result.tau.push_back(sim->tau[i]);
    }

protected:
    ros::NodeHandle m_node;
    actionlib::SimpleActionServer<${TARGET_ROS_NAME}::simulation1Action> m_server; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string m_actionName;
    ${TARGET_ROS_NAME}::simulation1Feedback m_feedback;
    ${TARGET_ROS_NAME}::simulation1Result m_result;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulation1_server");
    Simulation1Server simulation1("simulation1");
    ros::spin();

    return 0;
}