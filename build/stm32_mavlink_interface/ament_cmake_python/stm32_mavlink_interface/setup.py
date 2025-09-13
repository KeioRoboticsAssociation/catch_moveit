from setuptools import find_packages
from setuptools import setup

setup(
    name='stm32_mavlink_interface',
    version='1.0.0',
    packages=find_packages(
        include=('stm32_mavlink_interface', 'stm32_mavlink_interface.*')),
)
