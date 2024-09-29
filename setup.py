from setuptools import find_packages, setup

package_name = 'turtlebot3_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pravallika',
    maintainer_email='lakshmipravallika.adibhatla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtlebot3_controller = turtlebot3_controller.turtlebot3_controller:main',
                            'node_2 = turtlebot3_controller.node_2:main',

        ],
    },
)

