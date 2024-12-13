import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mtms_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='connect2brain',
    maintainer_email='connect2brain@aalto.fi',
    description='mTMS device simulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtms_simulator = mtms_simulator.mtms_simulator:main'
        ],
    },
)
