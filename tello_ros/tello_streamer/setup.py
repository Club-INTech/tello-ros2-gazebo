from setuptools import setup

package_name = 'tello_streamer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsimafei',
    maintainer_email='tsimafei.liashkevich@telecom-sudparis.eu',
    description='Video streamer for tello edu simulated by gazebo',
    license='No license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'streamer = tello_streamer.tello_streamer:main',
        ],
    },
)
