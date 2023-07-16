#include <math.h>
#include "simulation1.h"

namespace APP_NAMESPACE
{

namespace TARGET_NAMESPACE
{

Simulation1::Simulation1(const char* xmlFilePathNameOrString, int mode)
    : m_status(STATUS_NONE)
    , m_mode(mode)
{
    // read parameter.xml
    try{
        XmlReader xml;
        if (m_mode == MODE_FILE) xml.loadFile(xmlFilePathNameOrString);
        else if (m_mode == MODE_STRING) xml.writeXml(xmlFilePathNameOrString);
        else throw std::runtime_error("invalid mode");

        // system
        q = Vector2d::Zero();
        dq = Vector2d::Zero();
        tau = Vector2d::Zero();
        pr.push_back(atof(xml.getData("param.simulation1.l1")));
        pr.push_back(atof(xml.getData("param.simulation1.l2")));
        pr.push_back(pr[0]*pr[0]);
        pr.push_back(pr[0]*pr[1]);
        pr.push_back(pr[1]*pr[1]);
        pr.push_back(atof(xml.getData("param.simulation1.m1")));
        pr.push_back(atof(xml.getData("param.simulation1.m2")));
        pr.push_back(pr[5]+pr[6]);
        gc = atof(xml.getData("param.simulation1.gc"));
        
        // reference
        r = Vector2d::Zero();
        s0 = atof(xml.getData("param.simulation1.s0"));
        r0 = stoa(xml.getData("param.simulation1.r0"));
        
        // control
        Kp = stoa(xml.getData("param.simulation1.Kp"), 2, 2);
        Kd = stoa(xml.getData("param.simulation1.Kd"), 2, 2);
        e = Vector2d::Zero();
        de = Vector2d::Zero();
        v = Vector2d::Zero();
        pr_e.push_back(atof(xml.getData("param.simulation1.l1_e")));
        pr_e.push_back(atof(xml.getData("param.simulation1.l2_e")));
        pr_e.push_back(pr_e[0]*pr_e[0]);
        pr_e.push_back(pr_e[0]*pr_e[1]);
        pr_e.push_back(pr_e[1]*pr_e[1]);
        pr_e.push_back(atof(xml.getData("param.simulation1.m1_e")));
        pr_e.push_back(atof(xml.getData("param.simulation1.m2_e")));
        pr_e.push_back(pr_e[5]+pr_e[6]);
        
        // simulation
        itr = -1;
        ts = atof(xml.getData("param.simulation1.ts"));
        t0 = atof(xml.getData("param.simulation1.t0"));
        t1 = atof(xml.getData("param.simulation1.t1"));
        q0 = stoa(xml.getData("param.simulation1.q0"));
        dq0 = stoa(xml.getData("param.simulation1.dq0"));
        
        // initialize
        if (update()) throw std::runtime_error("update error");
    }
    catch (const std::exception &e) {
        throw std::runtime_error("Failed to initialize: " + std::string(e.what()));
    }

    m_status = STATUS_INIT;
}

Simulation1::~Simulation1()
{
}

int Simulation1::update()
{
    if (m_status == STATUS_FINE) return 1;

    if (itr == -1) {
        itr = 0;
        t = t0;
        q = q0;
        dq = dq0;
        std::tie(M,c,g) = getMcg(q, dq, pr);
        std::tie(M_e,c_e,g_e) = getMcg(q, dq, pr_e);
        if (t < s0) r = Vector2d::Zero();
        else r = r0;
        e = r - q;
        de = -dq;
        v = Kd*de + Kp*e;
        tau = M_e*v + c_e + g_e;
    }
    else {
        itr += 1;
        t += ts;
        q += ts*dq;
        dq += ts*M.inverse()*(tau - c - g);
        std::tie(M,c,g) = getMcg(q, dq, pr);
        std::tie(M_e,c_e,g_e) = getMcg(q, dq, pr_e);
        if (t < s0) r = Vector2d::Zero();
        else r = r0;
        e = r - q;
        de = -dq;
        v = Kd*de + Kp*e;
        tau = M_e*v + c_e + g_e;
    }

    if (t+ts > t1) m_status = STATUS_FINE;

    return 0;
}

std::tuple<Matrix2d, Vector2d, Vector2d> Simulation1::getMcg(
    const Vector2d &q, const Vector2d &dq, const std::vector<double> p)
{
    double l1=p[0], l2=p[1], l11=p[2], l12=p[3], l22=p[4], m1=p[5], m2=p[6], m12=p[7];
    double c1=cos(q(0)), c2=cos(q(1)), c12=cos(q(0)+q(1));
    double s1=sin(q(0)), s2=sin(q(1)), s12=sin(q(0)+q(1));
    double dq11=dq(0)*dq(0), dq12=dq(0)*dq(1), dq22=dq(1)*dq(1);

    Matrix2d M;
    M(0,0) = l22*m2 + 2*l12*m2*c2 + l11*m12;
    M(0,1) = l22*m2 + l12*m2*c2;
    M(1,0) = l22*m2 + l12*m2*c2;
    M(1,1) = l22*m2;

    Vector2d c;
    c(0) = -m2*l12*s2*dq22 - 2*m2*l12*s2*dq12;
    c(1) = m2*l12*s2*dq11;

    Vector2d g;
    g(0) = m2*l2*gc*c12 + m12*l1*gc*c1;
    g(1) = m2*l2*gc*c12;

    return {M, c, g};
}

} // TARGET_NAMESPACE

} // APP_NAMESPACE