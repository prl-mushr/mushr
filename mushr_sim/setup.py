from setuptools import find_packages, setup
from glob import glob
import os

# def recursive_files(root_dir: str):
#     matches = []
#     for path, _, files in os.walk(root_dir):
#         for f in files:
#             matches.append(os.path.join(path, f))
#     return matches

def config_data_files(share_name, config_dir):
    """Install config preserving subdirectory structure (avoids filename collisions)."""
    result = []
    for path, _, files in os.walk(config_dir):
        if files:
            dest = os.path.join('share', share_name, path)
            result.append((dest, [os.path.join(path, f) for f in files]))
    return result

setup(
    name="mushr_sim",
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + "mushr_sim"]),
        ('share/' + "mushr_sim", ['package.xml']),

        (os.path.join('share', "mushr_sim", 'launch'),
            glob('launch/*.launch.py') +
            glob('launch/*.launch.xml') +
            glob('launch/*.py') +
            glob('launch/*.xml') +
            glob('launch/*.yaml') +
            glob('launch/*.yml')
        ),

        # (os.path.join('share', "mushr_sim", 'config'),
        #     recursive_files('config')
        # ),
        *config_data_files("mushr_sim", "launch"),
        *config_data_files("mushr_sim", "maps"),
        *config_data_files("mushr_sim", "config"),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg',
    maintainer_email='sg@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_vesc_driver = mushr_sim.fake_vesc_driver:main',
            'foxglove_teleop = mushr_sim.foxglove_teleop:main',
            'keyboard_teleop = mushr_sim.keyboard_teleop:main',
            'sim_node = mushr_sim.mushr_sim:main',
            'clicked_point_to_reposition = mushr_sim.clicked_point_to_reposition:main',
            'throttle_interpolator = mushr_sim.throttle_interpolator:main',
            'wait_for_map_server = mushr_sim.wait_for_lifecycle_node:main',
        ],
    },
)