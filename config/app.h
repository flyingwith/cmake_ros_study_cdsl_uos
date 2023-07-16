#ifndef APP_H
#define APP_H

#define APP_NAME "${APP_NAME}"
#define APP_NAME_LOWERCASE "${APP_NAME_LOWERCASE}"
#define APP_NAMESPACE ${APP_NAME}
#define APP_VER "${APP_VER}"

#define APP_SOURCE_DIR "${APP_SOURCE_DIR}"
#define APP_BINARY_DIR "${APP_BINARY_DIR}"
#define APP_DATA_DIR "${APP_DATA_DIR}"

#define APP_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}"
// Linux / Windows / Darwin

#include <string>
#include <iostream>
#include <sstream> // stringstream
#include <stdarg.h> // va_list, va_start, va_arg, va_end

// Eigen
#include "Eigen/Dense"
using namespace Eigen;

// QuadProg++
// #include "Array.hh"
// #include "QuadProg++.hh"

#endif // APP_H