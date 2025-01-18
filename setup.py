from setuptools import find_packages, setup

package_name = 'viam_rover_package'

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
    maintainer='roland',
    maintainer_email='roland@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viam_rover_encoder = viam_rover_package.viam_rover_encoder:main',
            # 'viam_rover_motor_control = viam_rover_package.viam_rover_motor_control:main'
        ],
    },
)
