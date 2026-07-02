from setuptools import setup

package_name = 'filesystem_watcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'watchdog'],
    zip_safe=True,
    maintainer='Olli-Pekka Kahilakoski',
    maintainer_email='okahilak@gmail.com',
    description='Shared directory-watching utility for ROS2 Python nodes.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
