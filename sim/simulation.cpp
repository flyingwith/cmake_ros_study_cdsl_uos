#include <iostream>
#include "simulation.h"

Simulation::Simulation()
    : m_status(STATUS_NONE)
{
    n = 2;
    x = VectorXd::Zero(n);
    A = MatrixXd::Zero(n,n);
    x(0) = 1.0;
    x(1) = -2.1;
    A(0,0) = -0.1;
    A(0,1) = 0.0;
    A(1,0) = 0.0;
    A(1,1) = -0.1;

    // std::cout << "x = \n" << x << "\n";
    // std::cout << "A = \n" << A << "\n";
    // std::cout << "Ax = \n" << A*x << "\n";

    t = 0.0;
    dt = 0.001;
    tf = 0.1;

    m_status = STATUS_INIT;
}

Simulation::~Simulation()
{
    std::cout << "Simulation Destructor\n";
}

int Simulation::update()
{
    x = A*x;
    t += dt;

    if (t > tf) m_status = STATUS_FINE;

    return 0;
}