from setuptools import find_packages, setup

package_name = 'turtlebot_final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADDED PART
        ("share/" + package_name + "/launch", ["launch/gazebo.launch.py", "launch/gazebo_ver2.launch.xml", "launch/gazebo_ver3.launch.yml"]) # 
        # ADDED PART
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claypool',
    maintainer_email='theurgeboat@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controller_node = turtlebot_final_project.turtlesim_controller:main"
        ],
    },
)
