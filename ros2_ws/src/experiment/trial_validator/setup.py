from setuptools import setup
import os
from glob import glob

package_name = 'trial_validator'

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
    maintainer='Connect2Brain',
    maintainer_email='connect2brain@aalto.fi',
    description='Trial validator',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trial_validator = trial_validator.trial_validator:main',
        ],
    },
)
