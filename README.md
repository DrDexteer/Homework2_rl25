# Robotics Lab – Homework 2 (iiwa + KDL + Vision)

This project contains the ROS 2 nodes and configuration used to solve Homework 2
of the Robotics Lab course. 

## 1. Setup and build

From your ROS 2 workspace (e.g. `~/ros2_ws`):

```bash
colcon build
source install/setup.bash
```

All the commands below assume that the workspace has been built and sourced.

---

## 2. Common robot bringup

Most experiments share the same bringup step for the iiwa:

```bash
ros2 launch iiwa_bringup iiwa.launch.py   command_interface:=velocity   robot_controller:=velocity_controller
```

- This spawns the iiwa in Gazebo and activates the `velocity_controller`.
- In the vision part, the launch file has been modified to load the
  `aruco_world.sdf` containing the ArUco tag.

In a second terminal (also sourced) you can then launch the `ros2_kdl_node` in
the desired mode, as described in the next sections.

---

## 3. Kinematic control

The main node is:

- package: `ros2_kdl_package`
- executable: `ros2_kdl_node`
- parameters default file: `config/params.yaml`


After launching iiwa (Section 2), run:

```bash
ros2 launch ros2_kdl_package kdl_node.launch.py
```

The actual numeric values of the trajectory (duration, radius, end position,
etc.) are read from `config/params.yaml`.

You can monitor the tracking error and joint motion using:

```bash
rqt_plot
```

---

### 3.1 –  Null-space velocity controller

For point 1(b) the controller is extended with a null–space term that pushes the
joints away from their limits while performing the Cartesian tracking task.

After launching iiwa (Section 2), in the params.yaml file switch the `ctrl_mode` parameter to `:=velocity_ctrl_null`  and then run:

```bash
ros2 launch ros2_kdl_package kdl_node.launch
```

The parameter `ctrl_mode:=velocity_ctrl_null` switches the node to use the
`velocity_ctrl_null()` function.

You can compare the joint positions and velocities with the plain
`velocity_ctrl` case using `rqt_plot` on `/joint_states/position` and
`/joint_states/velocity`.

---

### 3.3 – Point 1(c): Action-based linear trajectory execution

In this point the same linear trajectory is executed through a ROS 2 action
server instead of the timer-based loop.

#### 3.3.1 – Start the action server

Launch iiwa (Section 2), then run `ros2_kdl_node` in action mode:

```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args \
  --params-file $(ros2 pkg prefix ros2_kdl_package)/share/ros2_kdl_package/config/params.yaml \
  -p cmd_interface:=velocity \
  -p ctrl_mode:=velocity_ctrl_null \
  -p action_mode:=true
```
(or by launch file )

- `action_mode:=true`  starts the
  `execute_linear_traj` action server.
- The controller used inside the action can be `velocity_ctrl` or
  `velocity_ctrl_null` depending on the `ctrl_mode` parameter.

You can check that the action server is up with:

```bash
ros2 action list | grep execute_linear_traj
```

#### 3.3.2 – Run the action client

The action client is implemented as a separate node:

- package: `ros2_kdl_package`
- executable: `execute_linear_traj_client`
- action type: `ros2_kdl_interfaces/action/ExecuteLinearTraj`

In a new terminal:

```bash
ros2 run ros2_kdl_package execute_linear_traj_client --ros-args \
  -p action_name:=execute_linear_traj \
  -p end_position:="[0.40, 0.10, 0.25]" \
  -p traj_duration:=5.0 \
  -p acc_duration:=0.5 \
  -p trajectory_len:=150 \
  -p s_type:=trapezoidal
```

The client will:

- wait for the action server,
- send a single goal with the specified end position and timing,
- print feedback with the current Cartesian error,
- and report the final error norm in the result.

You can also interactively inspect the action with:

```bash
ros2 action info /execute_linear_traj
```

---

## 4. Point 2 – Vision-based control with ArUco

### 4.1 – Point 2(a): Gazebo world with ArUco tag

The ArUco marker is modelled as a small static plane (size `0.1 × 0.1` m)
textured with `aruco-201.png` and added to a custom world file
`aruco_world.sdf`. The `iiwa.launch.py` file has been updated to load this world
so that the ArUco tag is visible from the robot camera.

Thus, the same iiwa bringup as in Section 2 automatically spawns the robot and
the ArUco tag:

```bash
ros2 launch iiwa_bringup iiwa.launch.py \
  command_interface:=velocity \
  robot_controller:=velocity_controller
```

You can verify in Gazebo that both the iiwa and the marker (`arucotag`) are
present.

---

### 4.2 – Run `ros2_kdl_node` in vision mode

With iiwa running, start the KDL node launch file  that starts also the aruco single launch to detect the marker:
`(ctrl_mode:="vision_ctrl", action_mode:=false)`

```bash
ros2 launch ros2_kdl_package kdl_node.launch.py
```


- The node subscribes to `/aruco_single/pose`.
- It computes the camera Jacobian and the interaction matrix `L(s)`.
- It sends joint velocity commands that try to align the camera optical axis
  with the marker direction.

You can monitor the commands with:

```bash
rqt_plot /velocity_controller/commands
```

and visualise the camera image with:

```bash
rqt_image_view
```

---

### 4.3 – Point 2(c): ROS 2 service to move the ArUco tag

In this point, a custom ROS 2 service is used to update the ArUco pose in
Gazebo by wrapping the Gazebo service `/world/default/set_pose` through
`ros_gz_bridge`.

1. Ensure the `ros_gz_bridge` parameter bridge is launched (the
   `iiwa.launch.py` file includes a node like):

   ```bash
   ros2 run ros_gz_bridge parameter_bridge \
     '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose'
   ```

   (In practice this is started from the launch file.)

2. Make sure `ros2_kdl_node` is running (any mode), so that the custom
   `set_aruco_pose` service is available:

   ```bash
   ros2 service list | grep set_aruco_pose
   ```

3. Call the service from the ROS 2 side to move the marker:

   ```bash
   ros2 service call /set_aruco_pose ros2_kdl_interfaces/srv/SetArucoPose \
     "{name: 'arucotag',
       pose: {position: {x: -0.60, y: -0.40, z: 0.35},
              orientation: {x: 0.0, y: 1.0, z: 0.0, w: 1.0}}}"
   ```
If the bridge and Gazebo are running correctly, the ArUco marker will jump to
the new pose in the simulation, and the vision controller will react by
re-orienting the camera towards the new position.

---

## 5. Useful debugging tools

Some useful ROS 2 tools while testing the homework:

```bash
# List topics, services, actions
ros2 topic list
ros2 service list
ros2 action list

# Inspect controllers
ros2 control list_controllers

# Visualise the computation graph
rqt_graph

# Plot signals (e.g., joint states, velocity commands)
rqt_plot

# Visualise the camera image
rqt_image_view
```
