from setuptools import find_packages, setup

package_name = 'ur3_controller'

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
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='UR3 teleop controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'ur3_quest_teleop = ur3_controller.ur3_quest_teleop:main',
        ],
    },
)
