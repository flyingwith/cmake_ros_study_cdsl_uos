#include <iostream>
#include "mathtools.h"

using namespace APP_NAMESPACE;

int main(int argc, char *argv[])
{
    //==========================================================================
    std::cout << "\n" + std::string(80, '=') + "\ntest array tools\n" + std::string(80, '=') + "\n\n";

    try{
        int n = 2, m = 3;
        VectorXd x(n);
        MatrixXd M(n,m);

        std::cout << "\n<-- set the array to zero -->\n\n";

        x = VectorXd::Zero(n);
        M = MatrixXd::Zero(n,m);
        
        ainfo(x);
        ainfo(M);

        std::cout << "\n<-- save the values to string and set the array to ones -->\n\n";

        std::string s_x = atos(x);
        std::string s_M = atos(M);

        x = VectorXd::Ones(n);
        M = MatrixXd::Ones(n,m);

        ainfo(x);
        ainfo(M);

        std::cout << "\n<-- restore the array from the string -->\n\n";
        
        x = stoa(s_x.c_str());
        M = stoa(s_M.c_str(), n, m);

        ainfo(x);
        ainfo(M);
    }
    catch (const std::exception &e) {
        std::cout << e.what() << "\n";
        return 1;
    }

    return 0;
}