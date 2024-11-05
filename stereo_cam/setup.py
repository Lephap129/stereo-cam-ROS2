from setuptools import find_packages, setup

package_name = 'stereo_cam'

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
    maintainer='phapanh',
    maintainer_email='lephap129@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo = stereo_cam.stereo:main',
            'talker = stereo_cam.publisher_member_function:main',
            'listener = stereo_cam.subscriber_member_function:main',
        ],
    },
)
