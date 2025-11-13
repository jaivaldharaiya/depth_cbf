from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtle_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        # Include data files
        (os.path.join('share', package_name, 'data'), 
         glob(os.path.join('data', '*.*'), recursive=True)),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'osqp',  # For CBF optimization
    ],
    zip_safe=True,
    maintainer='Package Maintainer',
    maintainer_email='maintainer@example.com',
    description='Point Cloud-Based Control Barrier Function Regression for Safe and Efficient Vision-Based Control on TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_controller = turtle_pkg.main:main',
            'cbf_controller = turtle_pkg.main:main',
            'simple_test = simple_test:main',
        ],
    },
)