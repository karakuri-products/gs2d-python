import os, sys
from setuptools import setup, find_packages

def read_requirements():
    """Parse requirements from requirements.txt."""
    reqs_path = os.path.join('.', 'requirements.txt')
    with open(reqs_path, 'r') as f:
        requirements = [line.rstrip() for line in f]
    return requirements

setup(
    name='gs2d',
    version='0.0.1',
    description='gs2d: Generic Serial-bus Servo Driver library for Python',
    long_description='gs2d makes us feel good!',
    author='Hideyuki Takei',
    author_email='hide@krkrpro.com',
    install_requires=read_requirements(),
    url='https://github.com/krkrpro/gs2d-python',
    license='Apache License Version 2.0',
    packages=find_packages(exclude=('tests', 'docs'))
)

