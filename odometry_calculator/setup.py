from setuptools import find_packages, setup

package_name = 'odometry_calculator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mirac',
    maintainer_email='mirac@todo.todo',
    description='Package for computing odometry from RPM data in ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_calculator = odometry_calculator.odometry_calculator:main',
        ],
    },
)
