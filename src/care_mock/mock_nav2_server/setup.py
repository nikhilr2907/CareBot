from setuptools import find_packages, setup

package_name = 'mock_nav2_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
        ('share/' + package_name + '/launch', ['launch/mock_and_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Mock Nav2 + sensor publisher for end-to-end testing of the CareRobotics WebSocket bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_nav2_server = mock_nav2_server.mock_node:main',
        ],
    },
)
