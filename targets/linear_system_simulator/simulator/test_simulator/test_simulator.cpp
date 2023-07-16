#include <iostream>
#include "simulation1.h"

using namespace APP_NAMESPACE;
using namespace APP_NAMESPACE::TARGET_NAMESPACE;

int main(int argc, char* argv[])
{
    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\nSimulation1(const char* xmlFilePathName)\n" + std::string(80, '=') + "\n\n";
    
    try {
        Simulation1 sim("parameter.xml");
        
        while (sim.getStatus() != Simulation1::STATUS_FINE) {
            sim.update();
            std::cout << "itr = " << sim.itr << ", t = " << sim.t << ", e = " << sim.e << "\n";
        }
    }
    catch (const std::exception &e) {
        std::cout << e.what() << "\n";
    }

    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\nSimulation1(const char* xmlFilePathNameOrString, int mode)\n" + std::string(80, '=') + "\n\n";
    
    try {
        XmlReader xml("parameter.xml");
        xml.putData("param.simulation1.t1", std::to_string(0.1).c_str());
        Simulation1 sim(xml.readXml(), Simulation1::MODE_STRING);
        
        while (sim.getStatus() != Simulation1::STATUS_FINE) {
            sim.update();
            std::cout << "itr = " << sim.itr << ", t = " << sim.t << ", e = " << sim.e << "\n";
        }
    }
    catch (const std::exception &e) {
        std::cout << e.what() << "\n";
    }

    return 0;
}