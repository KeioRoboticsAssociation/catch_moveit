from setuptools import find_packages, setup

package_name = 'd415_rgb_depth_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/d415_rgb_depth_3d.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaoru',
    maintainer_email='kaoru.yoshida@keio.jp',
    description='D415: tap pixel -> 3D target pose publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd415_rgb_depth_3d_node = d415_rgb_depth_3d.d415_rgb_depth_3d_node:main',
        ],
    },
)
