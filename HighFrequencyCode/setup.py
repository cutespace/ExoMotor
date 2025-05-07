# setup.py
from setuptools import setup, Extension
import pybind11

kinco_mod = Extension(
    'kinco_backend',
    ['kinco_backend.cpp'],
    include_dirs=[pybind11.get_include()],
    language='c++',
)

setup(
    name='kinco_backend',
    ext_modules=[kinco_mod],
)
