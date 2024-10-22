This simulation package provides a quick and easy way to try out the autonomous mobile robots from Neobotix in ROS-2. It comes with the most commonly used configuration but is open for any kind of modification.

Please find our documentations in https://neobotix-docs.de/ros/ros2/simulation.html

# Neobotix Neo Simulation2 Package - Contribution Overview

This README provides documentation on the changes and updates made to the `neo_simulation2` package, focusing on improvements, restructurings, and newly added functionalities. The aim is to make the package more modular and user-friendly for new users and developers working with it for the first time.

## Changes Overview

### Branch Merge and Updates
  - Updated both launch file comments to improve clarity.

### Simulation and Navigation Launch Files
- Added a condition to check if the provided robot name is valid.
- Introduced a condition to check if the robot supports an arm based on the `arm_type` argument, specifically for `mpo_500` and `mpo_700`.
- Included ROS2 control nodes for handling arms.
  - Currently, only UR (Universal Robotics) arms are supported.
  - Only `mpo_500` and `mpo_700` support UR arms (`ur5e`, `ur10e`), and these configurations can be tested using the [Neo MPO MoveIt2 package](https://github.com/AdarshKaran/neo_mpo_moveit2/tree/functionality/mpo_500_support).

### Modularization
- Modularized the description files for `mpo_500`, `mpo_700`, `mp_400`, and `mp_500` into xacro files, reducing redundancy and improving reusability. Improved the folder and file structure to better support modularized URDF files.
- Updated Caster Model for MPO-700 -> [Issue #99](https://github.com/neobotix/neo_simulation2/issues/99) regarding the caster model.

## Launch Commands
To use the simulation launch file, you can pass different arguments to configure the simulation according to your requirements.

Use the command below to see the available launch arguments and their respective descriptions:

```sh
ros2 launch neo_simulation2 simulation.launch.py --show-args
```

Output:

```
Arguments (pass arguments as '<name>:=<value>'):

    'my_robot':
        Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"
        (default: 'mpo_700')

    'world':
        Available worlds: "neo_track1", "neo_workshop"
        (default: 'neo_workshop')

    'arm_type':
        Arm Types:
         Elite Arms: ec66, cs66
         Universal Robotics: ur5, ur10, ur5e, ur10e
        (default: '')
```

### Example Launch Command
To launch the simulation with `mpo_500` and a `ur5` arm, use the following command:

```sh
ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_500 arm_type:=ur5
```

