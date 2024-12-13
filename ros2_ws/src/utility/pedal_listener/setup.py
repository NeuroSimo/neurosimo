from setuptools import setup

package_name = 'pedal_listener'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Olli-Pekka Kahilakoski',
    maintainer_email='olli-pekka.kahilakoski@aalto.fi',
    description='Pedal listener',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = pedal_listener.pedal_listener:main',
        ],
    },
)
