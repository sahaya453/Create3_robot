from setuptools import find_packages, setup

package_name = 'create3_pkg'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'twist_cmd_vel2 = create3_pkg.twist_cmd_vel2:main'
#          'MazeController = create3_pkg.MazeController:main'        
#    "animationController = create3_pkg.animationController:main"
        ],
    },
)
