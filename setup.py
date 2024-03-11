from setuptools import find_packages, setup

package_name = 'laser_pkg'

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
    maintainer='leonardo',
    maintainer_email='leonardo.cardenas.martine02@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtgoal = laser_pkg.mtgoal:main',
            'listener = laser_pkg.subscriber_member_function:main',
            'bug = laser_pkg.bug:main',
        ],
    },
)
