from setuptools import find_packages
from setuptools import setup

setup(
    name='rogilink_flex_lib',
    version='0.0.0',
    packages=find_packages(
        include=('rogilink_flex_lib', 'rogilink_flex_lib.*')),
)
