from setuptools import find_packages
from setuptools import setup

setup(
    name='pipe_swarm',
    version='0.0.0',
    packages=find_packages(
        include=('pipe_swarm', 'pipe_swarm.*')),
)
