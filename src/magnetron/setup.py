import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'magnetron'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages( where='.', include=['magnetron']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure glob() returns a valid list
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shawnchan',
    maintainer_email='shawnkengkiat@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # install_requires=['setuptools', 'pytest'],

    entry_points={
        'console_scripts': [
            'toggle_io = magnetron.toggle_io:main',
            'servo_node = magnetron.servo_node:main',
            'hook_control_node = magnetron.hook_control_node:main',
            'stepper_node = magnetron.stepper_node:main',
            'screw_control_node = magnetron.screw_control_node:main',
            'screw_control_rotation_node = magnetron.screw_control_node_rotation:main'
        ],
    },
)
