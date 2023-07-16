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
        n = atoi(xml.getData("param.simulation1.n"));
        m = atoi(xml.getData("param.simulation1.m"));
        p = atoi(xml.getData("param.simulation1.p"));
        if (n <=0 || m <= 0 || p <=0) throw std::runtime_error("invalid dimensions");
        x = VectorXd::Zero(n);
        u = VectorXd::Zero(m);
        y = VectorXd::Zero(p);
        y_p = VectorXd::Zero(p);
        dy = VectorXd::Zero(p);
        A = stoa(xml.getData("param.simulation1.A"), n, n);
        B = stoa(xml.getData("param.simulation1.B"), n, m);
        C = stoa(xml.getData("param.simulation1.C"), p, n);
        
        // reference
        r = VectorXd::Zero(p);
        s0 = atof(xml.getData("param.simulation1.s0"));
        r0 = stoa(xml.getData("param.simulation1.r0"));
        
        // control
        Kp = stoa(xml.getData("param.simulation1.Kp"), m, p);
        Kd = stoa(xml.getData("param.simulation1.Kd"), m, p);
        Ki = stoa(xml.getData("param.simulation1.Ki"), m, p);
        e = VectorXd::Zero(p);
        de = VectorXd::Zero(p);
        ie = VectorXd::Zero(p);
        
        // simulation
        itr = -1;
        ts = atof(xml.getData("param.simulation1.ts"));
        t0 = atof(xml.getData("param.simulation1.t0"));
        t1 = atof(xml.getData("param.simulation1.t1"));
        x0 = stoa(xml.getData("param.simulation1.x0"));
        
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
        x = x0;
        y = y_p = C*x;
        dy = VectorXd::Zero(p);
        if (t < s0) r = VectorXd::Zero(p);
        else r = r0;
        e = r - y;
        de = ie = VectorXd::Zero(p);
        u = Kp*e + Kd*de + Ki*ie;
    }
    else {
        itr += 1;
        t += ts;
        x += ts*(A*x + B*u);
        y = C*x;
        dy = (y - y_p)/ts;
        y_p = y;
        if (t < s0) r = VectorXd::Zero(p);
        else r = r0;
        e = r - y;
        de = -dy;
        ie += ts*e;
        u = Kp*e + Kd*de + Ki*ie;
    }

    if (t+ts > t1) m_status = STATUS_FINE;

    return 0;
}

} // TARGET_NAMESPACE

} // APP_NAMESPACE