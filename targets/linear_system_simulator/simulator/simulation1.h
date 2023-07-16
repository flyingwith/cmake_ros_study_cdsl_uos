#ifndef SIMULATION1_H
#define SIMULATION1_H

#include "xmlreader.h"
#include "mathtools.h"
#include "target.h"

namespace APP_NAMESPACE
{

namespace TARGET_NAMESPACE
{

class Simulation1
{
public:
    static const int STATUS_NONE = 0;
    static const int STATUS_INIT = 1;
    static const int STATUS_FINE = 2;

    static const int MODE_FILE = 0;
    static const int MODE_STRING = 1;

    Simulation1(const char* xmlFilePathName) 
        : Simulation1(xmlFilePathName, MODE_FILE) {}
    Simulation1(const char* xmlFilePathNameOrString, int mode);
    ~Simulation1();

    int update();

    int getStatus() const { return m_status; }
    
    // -------------------------------------------------------------------------
    // system
    double t; // current time
    unsigned short n; VectorXd x; // state
    unsigned short m; VectorXd u; // control input
    unsigned short p; VectorXd y, y_p, dy; // output, previous, derivative
    MatrixXd A; // n x n
    MatrixXd B; // n x m
    MatrixXd C; // p x n

    // -------------------------------------------------------------------------
    // reference

    VectorXd r;
    double s0;
    VectorXd r0;

    // -------------------------------------------------------------------------
    // control

    MatrixXd Kp, Kd, Ki; // m x p
    VectorXd e, de, ie; // error, derivative, integral

    // -------------------------------------------------------------------------
    // simulation
    unsigned long itr; // iteration
    double ts; // sampling time
    double t0; // initial time
    double t1; // final time
    VectorXd x0; // initial state
    
private:
    int m_status;
    int m_mode;
};

} // TARGET_NAMESPACE

} // APP_NAMESPACE

#endif // SIMULATION1_H