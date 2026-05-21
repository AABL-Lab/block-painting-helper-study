from setuptools import find_packages, setup

package_name = 'bph_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robots/ur_moveit.launch.py']),
        ('share/' + package_name + '/launch', ['launch/study.launch.py']),
        ('share/' + package_name + '/launch', ['launch/robots/ur3e.launch.py']),
        ('share/' + package_name + '/launch', ['launch/robots/2dof.launch.py']),        ('share/' + package_name + '/launch', ['launch/robots/kinova.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kat',
    maintainer_email='katallen@gmail.com',
    description='launch file package for study bringup with various arm hardware',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_2d_position_controller = bph_bringup.fake_position_controller:main',
        ],
    },
)
