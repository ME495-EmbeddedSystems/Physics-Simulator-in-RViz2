from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml','launch/show_turtle.launch.xml','launch/show_turtle.launch.py',
                                   'launch/run_turtle.launch.xml',
                                    'launch/turtle_arena.launch.xml',
                                    'urdf/turtle.urdf.xacro',
                                   'config/view_robot.rviz',
                                   'config/turtle_arena.rviz',
                                   'config/turtle.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adityanair',
    maintainer_email='aditya.nair0123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_turtle = turtle_brick.run_turtle:main',
            'arena = turtle_brick.arena:arena',
            'catcher = turtle_brick.catcher:main'
        ],
    },
)
