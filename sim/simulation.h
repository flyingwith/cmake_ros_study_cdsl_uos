#ifndef SIMULATION_H
#define SIMULATION_H

#include "Eigen/Dense"
using namespace Eigen;

class Simulation
{
public:
    static const int STATUS_NONE = 0;
    static const int STATUS_INIT = 1;

    Simulation();
    ~Simulation();

    int getStatus() const {return m_status;}

private:
    int m_status;
    unsigned short n;
    VectorXd x;
    MatrixXd A;
};

#endif // SIMULATION_H