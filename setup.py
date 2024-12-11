from setuptools import setup, find_packages

package_name = 'fleet_adapter_kabam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gary Bey',
    maintainer_email='gary.bey@kabam.ai',
    description='Fleet adapter compatible with Kabam Robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_kabam.fleet_adapter:main'
        ],
    },
)
