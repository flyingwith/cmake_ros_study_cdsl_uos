# -*- coding: utf-8 -*-

import pathlib, sys, os
import numpy as np

# install
# |- tools
# |  |- py_mathtools.so
# |  |- test_mathtools
# |     |- test_mathtools
# |     |- test_py_mathtools

sys.path.append(str(pathlib.Path(os.getcwd()).parent))
import py_mathtools
from py_mathtools import *

def main():
    # =========================================================================
    print('\n'+'='*80+'\ntest array tools\n'+'='*80+'\n')

    try:
        n = 2
        m = 3
        x = np.ndarray(n)
        M = np.ndarray((n,m))

        print('\n<-- set the array to zero -->\n')

        x = np.zeros(n)
        M = np.zeros((n,m))

        ainfo(x)
        ainfo(M)

        print('\n<-- save the values to string and set the array to ones -->\n')

        s_x = atos(x)
        s_M = atos(M)

        x = np.ones(n)
        M = np.ones((n,m))

        ainfo(x)
        ainfo(M)

        print('\n<-- restore the array from the string -->\n')

        x = stoa(s_x)
        M = stoa(s_M, M.shape)

        ainfo(x)
        ainfo(M)

    except RuntimeError as err:
        print(err)

if __name__ == '__main__':
    main()