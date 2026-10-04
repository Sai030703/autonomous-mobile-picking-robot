import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_amr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.srdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sai Charan Ramamurthy',
    maintainer_email='saicharanr2001@gmail.com',
    description='Custom diff-drive AMR for warehouse order picking.',
    license='MIT',
    entry_points={'console_scripts': []},
)
