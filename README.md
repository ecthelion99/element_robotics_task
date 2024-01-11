# element_robotics_task
Trask for element robotics, robotics engineer application


## Install

To install clone this repo.

### Requirements

Docker is the only requirement. Tested with the official [Docker Engine](/https://docs.docker.com/engine/install/) install and [Post Install](/https://docs.docker.com/engine/install/linux-postinstall/) instructions.

## Usage

1. Build the image

    ```bash
    cd ~/element_robotics_task
    ./run.py build
    ```

2. Enter the docker environment. Do this in three terminals.
    ```bash
    ./run.py
    ```

3. Launch the simulation in the first docker terminal

    ```bash
    ros2 launch bringup simulation.launch.py
    ```

    This will launch gazebo, rviz, ros_gz_bridge, robot_localisation and the required navigation files.

4. Launch the waypoint_action_client in a second docker terminal

    ```bash
    ros2 launch run waypoint_action_node
    ```

    This will prompt you to enter points in x, y, z, yaw form. It will prompt you for more until you press enter without a point. This client submits an NavigateThroughPoses
    action request. This method was chosen as it utilises the Nav2 action that already exists and is the idiomatic ros2/nav2 way (the use of these action at least) to complete
    such a task.
    
    The node also subscribes to the /estop topic which takes an empty message and cancels the request. This was chosen as it allows easy integration into a GUI and allows
    other nodes to do things when the /estop is signalled. (This should be a service not a topic)

4. Launch the estop node in the third docker terminal

    ```bash
    ros2 run waypoint_action_client estop_node
    ```

    This will prompt you to to press enter and an empty message will be sent on the /estop topic (again this should really be a service)

5. The container can be deleted using:
    ```bash
    ./run.py rm
    ```
    And the image can be deleted using:
    ```bash
    ./run.py rmi
    ```

## Design decisions
 Many design decisions were driven by the amount of time before the deadline for the task.

 Control of the X1 used gazebo's `diff_drive_system`. This was chosen over `ros2_control` as we were required to use the `.sdf` model provided and `ros2_control` integrate with `.sdf` is more difficult than when using `urdf` files.

 Much of the Nav2 configuration was based on the `nav2_bringup` configurations. The significant notable exception was the costmap, where only the `VoxelLayer` and `InflationLayer` was used. This is because we are operating in 3d terrain and require a map that takes this into consideration. A mapping algorithm with a wider range and consideration of gradient would be better for this application however.

## Bugs
- Behavoir when given a goal to close to an obstacle is not ideal. It should give up but it doesn't seem to. This likley requires some modifications to the behavior tree.
- `NavigateThroughPoses` action  from `waypoint_action_node` ignored if a goal is already published in Rviz.

## Limitations
Many of these limitations where choices made so I could complete as much of the task as possible
- Terrain: The terrain is flat with some obstacle as the provided world file did not work out of the box (physics stopped working). Attempts were made to fix this my own world file base on the same data but did not have time to complete this.
- Localisation: Currently the localisation only uses odometry. IMU and LiDAR/Visual Odometry should be integrated. The transform from map => odom is currently static and should be replaced with a slam package.
- LiDAR: LiDAR is used. Ideally a depth camera would be used instead as this is meant to be a space environment and LiDAR are not neccisarily suitible for such environments.
- ESTOP: The estop functionality only interupts the action started by the `waypoint_action_client`. This should be made safer and the `/cmd_vel` should be set to zeros to stop the rover.
- No Tests: Testing was not completed. This would have involved launching a headless simulation. Providing a set of goals and testing the `true` position when a goal is reached. The touch sensor also would have been used to check for collisions.
- RVIZ issues: When using NavigateThroughPoses the goals are not shown as poses in rviz. When the action is cancelled the path still displays.
- Range: There is a limited range.

### TODO
- [ ] Make Tests
- [ ] Make Moon Terrain.
- [ ] Integrate IMU into `robot_localization`.
- [ ] Integrate `rtab_map` for slam and odometry.
- [ ] Improve ETOP functionality.
- [ ] Publish points from `waypoint_action_client` to a specific topic so rviz can display them.
- [ ] Clear the path after `NavigateThroughPoses` action is cancelled so it doesnt show in rviz.
