from setuptools import setup

package_name = 'aruco_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'aruco_py.markers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Musau',
    maintainer_email='pmusau13ster@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'createMarker = aruco_py.create_marker:main',
            'detectVideo = aruco_py.detect_aruco_video:main',
        ],
    },
)
