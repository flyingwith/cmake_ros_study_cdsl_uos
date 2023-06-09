#include <iostream>
#include "simulation.h"

int main(int argv, char* argc[])
{
    Simulation sim;

    while (sim.getStatus() != Simulation::STATUS_FINE) 
    {
        sim.update();
        std::cout << sim.x << "\n";
    }

    return 0;
}