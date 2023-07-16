from libcpp.string cimport string
from libcpp.memory cimport shared_ptr

cdef extern from "mathtools.h" namespace "${APP_NAME}":
    cdef cppclass MathTools:
        MathTools() except +

    cdef shared_ptr[MathTools] MathTools_getInstance "${APP_NAME}::MathTools::getInstance"()