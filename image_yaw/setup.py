from setuptools import setup
import os
import glob

package_name = 'image_yaw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    	('share/' + package_name, ['package.xml']),
    	(os.path.join('share', package_name, 'weights'), glob.glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dodo',
    maintainer_email='dodo@todo.todo',
    description='YOLO person tracking -> publish yaw error (ex) for yaw control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_yaw_node = image_yaw.image_yaw_node:main',
        ],
    },
)
