from setuptools import find_packages, setup

package_name = 'robot_data_publisher'

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
    maintainer='gyeum',
    maintainer_email='giyeong626@motomind.co.kr',
    description='robot_data_publisher',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_data_publisher = robot_data_publisher.robot_data_publisher_node:main',
    ],
},

)
