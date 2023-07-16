# -*- coding: utf-8 -*-

from setuptools import Extension, setup
from Cython.Build import cythonize

import os
os.environ["CC"] = "clang"
os.environ["CXX"] = "clang++"

include_dirs = [x for x in '${APP_TOOLS_INCLUDE_DIRS}'.split(';') if x != '']
library_dirs = [x for x in '${APP_TOOLS_LIBRARY_DIRS}'.split(';') if x != '']
libraries = [x for x in '${APP_TOOLS_LIBRARIES}'.split(';') if x != '']
extra_compile_args = [x for x in '${APP_TOOLS_COMPILE_OPTIONS}'.split(';') if x != '']
extra_link_args = [x for x in '${APP_TOOLS_LINK_OPTIONS}'.split(';') if x != '']

print('-'*80)
print(f'include_dirs = {include_dirs}')
print(f'library_dirs = {library_dirs}')
print(f'libraries = {libraries}')
print(f'extra_compile_args = {extra_compile_args}')
print(f'extra_link_args = {extra_link_args}')
print('-'*80)

ext_modules = [
    Extension(
        'py_xmlreader',
        sources = ['py_xmlreader.pyx'],
        language = 'c++',
        include_dirs = include_dirs,
        library_dirs = library_dirs,
        libraries = libraries,
        extra_compile_args = extra_compile_args,
        extra_link_args = extra_link_args
    ),
    Extension(
        'py_mathtools',
        sources = ['py_mathtools.pyx'],
        language = 'c++',
        include_dirs = include_dirs,
        library_dirs = library_dirs,
        libraries = libraries,
        extra_compile_args = extra_compile_args,
        extra_link_args = extra_link_args
    ),
]

setup(
    name = 'c++ class wrapper of XmlReader, MathTools,...',
    ext_modules = cythonize(
        ext_modules, 
        build_dir = '${APP_TOOLS_BINARY_DIR}',
        annotate = True
    ),
    zip_safe = False,
)