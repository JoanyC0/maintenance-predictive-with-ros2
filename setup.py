from setuptools import find_packages, setup

package_name = 'maintenance_pred_ros2'

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
    maintainer='jacquescormery',
    maintainer_email='jacquescormery@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
'obd_publisher_node = maintenance_pred_ros2.obd_publisher:main',
'model_subscriber_node = maintenance_pred_ros2.lstm_subscriber:main',
'hi_analyzer_node = maintenance_pred_ros2.hi_analyzer_node:main',

        ],
    },
)
