from setuptools import find_packages
from setuptools import setup

setup(
    name='orin_car',
    version='0.0.1',
    packages=find_packages(
        include=('orin_car', 'orin_car.*')),
)
