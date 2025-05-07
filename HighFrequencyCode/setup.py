# highfrequencycode/setup.py
# Author: Gemini AI
# Description: Setup script for compiling the Kinco HFB C++ backend.

from setuptools import setup, Extension
import pybind11
import os
import sys

# Module name must match the PYBIND11_MODULE name in the C++ file
module_name = 'kinco_hfb'

cpp_args = []
link_args = []

if sys.platform == 'win32':
    # MSVC specific arguments
    cpp_args = ['/std:c++17', '/EHsc', '/DWIN32', '/GR', '/MD'] # /GR for RTTI, /MD for multithreaded DLL
    # Add any necessary libraries for Windows, e.g., Kernel32.lib (usually linked by default)
    # link_args = ['/LIBPATH:...'] 
else:
    # GCC/Clang specific arguments (for Linux/macOS)
    cpp_args = ['-std=c++17', '-Wall', '-Wextra', '-fPIC', '-pthread']
    link_args = ['-pthread']


ext_modules = [
    Extension(
        module_name,                        # Name of the module to be imported in Python
        ['kinco_backend.cpp'],              # List of C++ source files
        include_dirs=[
            pybind11.get_include(),         # Pybind11 include path
            pybind11.get_include(True),     # Pybind11 include path (user part, if different)
            # Add any other necessary include directories here
            # e.g., if you have other C++ libraries in a subfolder like 'include'
            # os.path.join(os.path.dirname(__file__), 'include') 
        ],
        language='c++',
        extra_compile_args=cpp_args,
        extra_link_args=link_args,
        # Depending on the compiler and OS, you might need to specify libraries explicitly
        # libraries=['kernel32'] # Example for Windows, though often not needed for standard libs
    ),
]

setup(
    name=module_name,
    version='0.1.0',
    author='Your Name / Gemini AI',
    author_email='your.email@example.com',
    description='Python C++ backend for high-frequency Kinco motor communication',
    long_description='This package provides a C++ backend with Python bindings for reading Kinco motor position data at high frequencies via Modbus RTU.',
    ext_modules=ext_modules,
    # Optional: if you have Python-only submodules or packages
    # packages=find_packages(where='src'), 
    # package_dir={'': 'src'},
    # Tell setuptools to run pybind11 during build process if needed
    # (usually pybind11.get_include() handles this part for headers)
    # cmdclass={'build_ext': pybind11.setup_helpers.BuildExt},
    zip_safe=False, # C extensions are generally not zip-safe
    python_requires='>=3.7', # Example minimum Python version requirement
) 