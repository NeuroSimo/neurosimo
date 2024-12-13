from setuptools import setup
import os
from glob import glob

package_name = 'pulse_sequence_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyösti Alkio',
    maintainer_email='kyosti.alkio@aalto.fi',
    description='mTMS pulse sequence controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_pulse_sequence = pulse_sequence_controller.start_pulse_sequence:main',
        ],
    },
)
