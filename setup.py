from setuptools import find_packages, setup
from glob import glob

package_name = 'quadrotor_formation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='greenzhu',
    maintainer_email='131279450+GreenZhu233@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_formation_node = quadrotor_formation.circle_formation:main',
            'circle_formation_lifecycle_node = quadrotor_formation.circle_formation_lifecycle:main',
        ],
    },
)
