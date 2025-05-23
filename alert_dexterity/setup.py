import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'alert_dexterity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join("share", package_name, "states"), glob("states/*.py")),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guts',
    maintainer_email='ninad.kulkarni@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_approach = alert_dexterity.auto_approach:main',
        ],
    },
)
