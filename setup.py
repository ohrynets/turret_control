from setuptools import find_packages, setup

package_name = 'turret_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='oleg.grinets@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turret_node = turret_control.move_turret_node:main',
            'dron_marker_node = turret_control.drone_marker_node:main',
            'joy_to_turret_node = turret_control.joy_to_turret_node:main',
            'target_on_image_node = turret_control.target_on_image_node:main',
            'image_compressor_node = turret_control.image_compressor_node:main',
            'drone_box_predictor_node = turret_control.drone_box_predictor_node:main',
        ],
    },
)
