from setuptools import find_packages
from setuptools import setup

setup(
    name='rogilink_flex_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('rogilink_flex_interfaces', 'rogilink_flex_interfaces.*')),
)
