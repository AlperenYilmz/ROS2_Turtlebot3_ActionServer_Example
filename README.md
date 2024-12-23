## ROS2 Turtlebot3 Action Server
- This package is a comprehensive example for beginner level ROS2 applications.
- The only source file is responsible for creating a lifecycle node, lifecycle publisher, calling Turtlesim's kill/spawn services, creating an action server to move Turtlesim and Turtlebot simultaneously.

## Launch
- There's 3 launch files in Python, XML and Yaml for educational assortment. All handle the same process.

## What does it do?
- Action server starts unconfigured,
- Turtlesim starts with default turtle1 entity, (one might as well just run blank turtlesim)
- Turtlebot3 gazebo sim starts in empty world with imu sensor and camera

## Switching between states
- Action server is not initialized until activate state
