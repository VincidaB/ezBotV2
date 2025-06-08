# setup.py
from setuptools import setup

package_name = 'remote_calc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='vincent',
    maintainer='vincent',
    description='…',
    license='…',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_serial_tf_broadcaster = remote_calc.aruco_serial_tf_broadcaster:main',
        ],
    },
)
