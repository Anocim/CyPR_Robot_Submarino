from setuptools import find_packages
from setuptools import setup

setup(
    name='orca_msgs',
    version='0.5.0',
    packages=find_packages(
        include=('orca_msgs', 'orca_msgs.*')),
)
