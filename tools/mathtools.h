#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <sstream>
#include "app.h"

#define ALIAS_FUNCTION(OriginalnamE, AliasnamE) \
template <typename... Args> \
inline auto AliasnamE(Args&&... args) -> decltype(OriginalnamE(std::forward<Args>(args)...)) { \
    return OriginalnamE(std::forward<Args>(args)...); \
}

namespace APP_NAMESPACE
{

class MathTools
{
public:
    // -------------------------------------------------------------------------
    // singleton class setup

    MathTools(MathTools const&) = delete;
    MathTools& operator=(MathTools const&) = delete;
    static std::shared_ptr<MathTools> getInstance() {
        static std::shared_ptr<MathTools> instance{new MathTools};
        return instance;
    }
    virtual ~MathTools() {}

    // -------------------------------------------------------------------------
    // array tools

    static void printArrayInfo(const MatrixXd& M) {
        std::cout << "size = " << M.size() << ", rows = " << M.rows() << ", cols = " << M.cols() << ", values = \n";
        std::cout << M.derived() << std::endl;
    }

    static std::string arrayToString(const MatrixXd& M) {
        return arrayToString(M, ',');
    }
    static std::string arrayToString(const MatrixXd& M, char delimiter) {
        std::stringstream ss;
        int k = 0;
        for (int i=0; i<M.rows(); i++) {
            for (int j=0; j<M.cols(); j++) {
                ss << M(i,j);
                if (++k < M.size()) ss << delimiter;
            }
        }
        return ss.str();
    }
    
    static MatrixXd stringToArray(const char* str) {
        return stringToArray(str, ',');
    }
    static MatrixXd stringToArray(const char* str, char delimiter) {
        std::stringstream ss(str);
        std::string s;
        std::vector<double> v;
        while (getline(ss, s, delimiter)) v.push_back(stof(s));
        MatrixXd M(v.size(),1);
        for (int i=0; i<v.size(); i++)
            M(i,0) = v[i];
        return M;
    }
    static MatrixXd stringToArray(const char* str, int rows, int cols) {
        return stringToArray(str, rows, cols, ',');
    }
    static MatrixXd stringToArray(const char* str, int rows, int cols, char delimiter) {
        std::stringstream ss(str);
        std::string s;
        std::vector<double> v;
        while (getline(ss, s, delimiter)) v.push_back(stof(s));
        MatrixXd M(rows,cols);
        if (v.size() == rows * cols) {
            int k=0;
            for (int i=0; i<rows; i++)
                for (int j=0; j<cols; j++)
                    M(i,j) = v[k++];
        }
        else throw std::runtime_error("Failed to convert the string to array");
        return M;
    }

    // -------------------------------------------------------------------------

protected:
    MathTools() {}
};

ALIAS_FUNCTION(MathTools::printArrayInfo, ainfo);
ALIAS_FUNCTION(MathTools::arrayToString, atos);
ALIAS_FUNCTION(MathTools::stringToArray, stoa);

}

#endif // MATHTOOLS_H