from setuptools import find_packages
from setuptools import setup

setup(
    name='dynamixel_controller',
    version='0.0.0',
    packages=find_packages(
        include=('dynamixel_controller', 'dynamixel_controller.*')),
)
