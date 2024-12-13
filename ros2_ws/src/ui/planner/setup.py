import os
from glob import glob
from setuptools import setup

package_name = 'planner'

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
    maintainer='Olli-Pekka Kahilakoski',
    maintainer_email='olli-pekka.kahilakoski@aalto.fi',
    description='mTMS planner',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_target = planner.add_target:main',
            'remove_target = planner.remove_target:main',
            'toggle_select_target = planner.toggle_select_target:main',
            'set_target = planner.set_target:main',
            'rename_target = planner.rename_target:main',
            'toggle_visible = planner.toggle_visible:main',
            'change_comment = planner.change_comment:main',
            'toggle_navigation = planner.toggle_navigation:main',
            'clear_state = planner.clear_state:main',
            'change_target_index = planner.change_target_index:main',
            'add_pulse_sequence = planner.add_pulse_sequence:main',
            'rename_pulse_sequence = planner.rename_pulse_sequence:main',
            'toggle_select_pulse_sequence = planner.toggle_select_pulse_sequence:main',
            'remove_pulse_sequence = planner.remove_pulse_sequence:main',
            'remove_pulse = planner.remove_pulse:main',
            'set_pulse_sequence_isi = planner.set_pulse_sequence_isi:main',
            'set_pulse_sequence_intensity = planner.set_pulse_sequence_intensity:main',
            'set_pulse_isi = planner.set_pulse_isi:main',
            'set_pulse_intensity = planner.set_pulse_intensity:main',
            'toggle_select_pulse = planner.toggle_select_pulse:main',
            'toggle_visible_pulse = planner.toggle_visible_pulse:main',
            'change_pulse_index = planner.change_pulse_index:main',
            'master_state = planner.master_state:main',
            'add_pulse_to_pulse_sequence = planner.add_pulse_to_pulse_sequence:main',
            'set_target_orientation = planner.set_target_orientation:main'
        ],
    },
)
