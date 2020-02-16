"""A setuptools based setup module.
See:
https://packaging.python.org/guides/distributing-packages-using-setuptools/
https://github.com/pypa/sampleproject
"""

# Always prefer setuptools over distutils
from os import path
from setuptools import setup, find_packages

# Utility function to read the README file.
# Used for the long_description.  It's nice, because now 1) we have a top level
# README file and 2) it's easier to type in the README file than to put a raw
# string in below ...
def read(fname):
    return open(path.join(path.dirname(__file__), fname)).read()

setup(
    name = "control_utilities",
    version = "0.0.1",
    author = "Aaron Young",
    description = ("Simulator for PyChrono and matplotlib. To be used to test controls algorithms"),
    license = "BSD",
    packages=find_packages(),
    # long_description=read('README.md'),
    # classifiers=[
    #     "Development Status :: 3 - Alpha",
    #     "Topic :: Utilities",
    #     "License :: OSI Approved :: BSD License",
    # ],
    install_requires=[
        'matplotlib',
        'numpy',
        'scipy',
    ]
)
