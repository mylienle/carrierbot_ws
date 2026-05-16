from setuptools import setup

package_name = 'carrierbot_mqtt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/reach_goal.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duybuntu',
    maintainer_email='quangduy160204@gmail.com',
    description='MQTT bridge nodes for CarrierBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reach_goal = carrierbot_mqtt.reach_goal:main',
        ],
    },
)
