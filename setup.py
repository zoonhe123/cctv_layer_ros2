from setuptools import find_packages, setup
import glob,os

package_name = 'cctv_layer_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zh',
    maintainer_email='zh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pub_detection = cctv_layer_ros2.pub_detection:main',
        'move_obstacle = cctv_layer_ros2.move_obstacle:main',
        'position_plotter = cctv_layer_ros2.position_plotter:main',
        'burger_position_monitor = cctv_layer_ros2.burger_position_monitor:main',
        ],
    },
)
