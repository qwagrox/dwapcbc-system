#!/usr/bin/env python3
"""
DWAPCBC - Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve
Setup script for package installation
"""

from setuptools import setup, find_packages
import os

# 读取README
def read_long_description():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            return f.read()
    return ''

# 读取requirements
def read_requirements():
    requirements_path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    if os.path.exists(requirements_path):
        with open(requirements_path, 'r', encoding='utf-8') as f:
            return [line.strip() for line in f if line.strip() and not line.startswith('#')]
    return [
        'numpy>=1.20.0',
        'scipy>=1.7.0',
        'matplotlib>=3.4.0',
        'pandas>=1.3.0',
        'networkx>=2.6.0',
        'shapely>=1.8.0',
    ]

setup(
    name='dwapcbc',
    version='1.0.0',
    author='DWAPCBC Team',
    author_email='your.email@example.com',
    description='Dynamic Waypoint Allocation Piecewise Cubic Bezier Curve Path Planning Library',
    long_description=read_long_description(),
    long_description_content_type='text/markdown',
    url='https://github.com/qwagrox/dwapcbc-system',
    project_urls={
        'Bug Tracker': 'https://github.com/qwagrox/dwapcbc-system/issues',
        'Documentation': 'https://github.com/qwagrox/dwapcbc-system/blob/main/README.md',
        'Source Code': 'https://github.com/qwagrox/dwapcbc-system',
    },
    packages=find_packages(exclude=['tests', 'docs', 'examples', 'backend']),
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Robotics',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.8',
    install_requires=read_requirements(),
    extras_require={
        'dev': [
            'pytest>=7.0.0',
            'pytest-cov>=3.0.0',
            'black>=22.0.0',
            'flake8>=4.0.0',
            'mypy>=0.950',
        ],
        'api': [
            'flask>=2.0.0',
            'flask-cors>=3.0.0',
        ],
        'all': [
            'pytest>=7.0.0',
            'pytest-cov>=3.0.0',
            'black>=22.0.0',
            'flake8>=4.0.0',
            'mypy>=0.950',
            'flask>=2.0.0',
            'flask-cors>=3.0.0',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwapcbc-plan=dwapcbc.cli:main',
            'dwapcbc-batch=dwapcbc.batch_test:main',
        ],
    },
    include_package_data=True,
    package_data={
        'dwapcbc': ['scenarios/*.json'],
    },
    keywords=[
        'path planning',
        'robotics',
        'mobile robots',
        'bezier curve',
        'dijkstra',
        'motion planning',
        'autonomous navigation',
    ],
    zip_safe=False,
)

