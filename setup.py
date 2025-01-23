from setuptools import find_packages, setup

package_name = 'viam_rover_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roland Meertens',
    maintainer_email='rolandmeertens@gmail.com',
    description='ROS2 package for the VIAM rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viam_rover_control = viam_rover_package.viam_rover_control:main',
        ],
    },
)
