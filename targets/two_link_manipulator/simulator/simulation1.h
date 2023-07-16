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

    int getStatus() const { return m_status; }

    int update();

    std::tuple<Matrix2d, Vector2d, Vector2d> getMcg(const Vector2d &q, const Vector2d &dq, const std::vector<double> p);

    // -------------------------------------------------------------------------
    // system
    double t; // current time
    Vector2d q, dq;
    Vector2d tau;
    std::vector<double> pr; // parameters l1, l2, l11, l12, l22, m1, m2, m12
    double gc; // gravitational constant
    Matrix2d M;
    Vector2d c, g;
    
    // -------------------------------------------------------------------------
    // reference

    VectorXd r;
    double s0;
    VectorXd r0;

    // -------------------------------------------------------------------------
    // control

    Matrix2d Kp, Kd;
    Vector2d e, de; // error, derivative
    Vector2d v;
    std::vector<double> pr_e; // estimated parameters for l1, l2, l11, l12, l22, m1, m2, m12
    Matrix2d M_e;
    Vector2d c_e, g_e;

    // -------------------------------------------------------------------------
    // simulation
    unsigned long itr; // iteration
    double ts; // sampling time
    double t0; // initial time
    double t1; // final time
    VectorXd q0, dq0; // initial state
    
private:
    int m_status;
    int m_mode;
};

} // TARGET_NAMESPACE

} // APP_NAMESPACE

#endif // SIMULATION1_H