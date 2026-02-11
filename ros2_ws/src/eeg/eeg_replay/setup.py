from setuptools import setup
import os
from glob import glob

package_name = 'eeg_replay'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Olli-Pekka Kahilakoski',
    maintainer_email='okahilak@gmail.com',
    description='EEG replay',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eeg_replay = eeg_replay.eeg_replay:main',
        ],
    },
)
