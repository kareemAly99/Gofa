from setuptools import setup
from glob import glob

package_name = 'gofa_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # RViz configs
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        # Meshes (carrier)
        ('share/' + package_name + '/resource/carrier_meshes', glob('resource/carrier_meshes/*.STL')),
        # Package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kareem Aly',
    maintainer_email='kareem.aly@tuhh.de',
    description='Visualization package for GOFA robot with carrier',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_tanks = gofa_viz.scripts.add_tanks:main',
        ],
    },
)