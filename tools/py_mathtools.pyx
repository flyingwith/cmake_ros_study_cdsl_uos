from cython.operator cimport dereference as deref
from libcpp.string cimport string
from libcpp.memory cimport shared_ptr
from mathtools cimport MathTools
from mathtools cimport MathTools_getInstance

import numpy as np

cdef class PyMathTools:
    cdef MathTools *c_mathtools
    
    def __cinit__(self):
        self.c_mathtools = &deref(MathTools_getInstance())
    
    def __dealloc__(self):
        pass

    @staticmethod
    def printArrayInfo(M: np.ndarray):
        if M.ndim not in {1,2}:
            raise RuntimeError('invalid dimension of the numpy array')
        print(f'size = {M.size}, shape = {M.shape}, values = ')
        print(M)

    @staticmethod
    def arrayToString(M: np.ndarray, delimiter=None) -> str:
        if delimiter == None:
            delimiter = ','
        ret = ''
        if M.ndim == 1:
            for i in range(M.size):
                ret += f'{M[i]}'
                if i < M.size-1:
                    ret += delimiter
        elif M.ndim == 2:
            k = 0
            for i in range(M.shape[0]):
                for j in range(M.shape[1]):
                    ret += f'{M[i,j]}'
                    if k < M.size-1:
                        ret += delimiter
                        k += 1
        else:
            raise RuntimeError('Invalid dimension of the array')
        return ret

    @staticmethod
    def stringToArray(string, shape=None, delimiter=None):
        if delimiter == None:
            delimiter = ','
        v = string.split(delimiter)
        if shape == None or (len(shape) == 1 and len(v) == shape[0]):
            M = np.ndarray(len(v))
            for i in range(len(v)):
                M[i] = float(v[i])
        elif len(shape) == 2 and len(v) == shape[0]*shape[1]:
            M = np.ndarray((shape[0],shape[1]))
            k = 0
            for i in range(shape[0]):
                for j in range(shape[1]):
                    M[i,j] = float(v[k])
                    k += 1
        else:
            raise RuntimeError('Failed to convert the string to array')
        return M

ainfo = PyMathTools.printArrayInfo
atos = PyMathTools.arrayToString
stoa = PyMathTools.stringToArray