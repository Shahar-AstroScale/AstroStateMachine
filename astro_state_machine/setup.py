from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'astro_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    setup_requires=['strenum'],
    zip_safe=True,
    maintainer='gidi',
    maintainer_email='gideon.szanton@algowis.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astro_state_machine = astro_state_machine.astro_state_machine:main',
        ],
    },
)
