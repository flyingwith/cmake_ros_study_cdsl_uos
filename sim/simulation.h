#ifndef SIMULATION_H
#define SIMULATION_H

#include "Eigen/Dense"
using namespace Eigen;

#include "Array.hh"

class Simulation
{
public:
    static const int STATUS_NONE = 0;
    static const int STATUS_INIT = 1;
    static const int STATUS_FINE = 2;

    Simulation();
    ~Simulation();

    int update();

    int getStatus() const {return m_status;}
    
    VectorXd x;

private:
    int m_status;
    unsigned short n;
    MatrixXd A;
    double t;
    double dt;
    double tf;
};

#endif // SIMULATION_H