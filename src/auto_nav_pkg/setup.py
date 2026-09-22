from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_nav_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='openai@example.com',
    description='Obstacle-race task manager and monitoring nodes for a quadruped robot.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'obstacle_manager_node = auto_nav_pkg.obstacle_manager:main',
            'nav_executor_node = auto_nav_pkg.nav_executor:main',
            'nav_agent_node = auto_nav_pkg.nav_agent:main',
            'yolo_relative_nav_node = auto_nav_pkg.yolo_relative_nav:main',
            'orange_pole_detector_node = auto_nav_pkg.orange_pole_detector:main',
            # 2026-05-03 01:40 CST: HSV pole detector now has its own route
            # trigger node so YOLO and HSV pipelines can be launched/rolled
            # back independently.
            'orange_pole_relative_nav_node = auto_nav_pkg.orange_pole_relative_nav:main',
            'orange_pole_relative_nav_stable_node = auto_nav_pkg.orange_pole_relative_nav_stable:main',
            'orange_pole_lidar_slalom_node = auto_nav_pkg.orange_pole_lidar_slalom:main',
            'orange_hurdle_jump_test_node = auto_nav_pkg.orange_hurdle_jump_test:main',
            'limit_bar_duck_test_node = auto_nav_pkg.limit_bar_duck_test:main',
            'slope_branch_detector_node = auto_nav_pkg.slope_branch_detector:main',
            'upstairs_detector_node = auto_nav_pkg.upstairs_detector:main',
            'obstacle_race_supervisor_node = auto_nav_pkg.obstacle_race_supervisor:main',
            'apriltag_align_test_node = auto_nav_pkg.apriltag_align_test:main',
        ],
    },
)
