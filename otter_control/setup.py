from setuptools import setup

package_name = 'otter_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Offroad Robotics @ Ingenuity Labs',
    maintainer_email='15mrc5@queensu.ca',
    description='This package is to be used with the OtterROS package for creating controllers for the Otter USV.',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmpc_controller = otter_control.nmpc_controller:main',
            'head_controller = otter_control.los_head_controller:main',
        ],
    },
)
