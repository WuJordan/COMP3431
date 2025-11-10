from setuptools import find_packages
from setuptools import setup

setup(
    name='wall_follower',
    version='0.0.0',
    packages=find_packages(
        include=('wall_follower', 'wall_follower.*')),
)
