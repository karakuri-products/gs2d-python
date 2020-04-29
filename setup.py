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
    version='0.0.2',
    description='gs2d: The Library for Generic Serial-bus Servo Driver kr-sac001 for Python',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='Hideyuki Takei',
    author_email='hide@krkrpro.com',
    install_requires=read_requirements(),
    url='https://github.com/karakuri-products/gs2d-python',
    license='Apache License Version 2.0',
    packages=find_packages(exclude=('tests', 'docs'))
)
