# Traffic-Sign Navigation with YOLO and Nav2

This project makes the rUBot navigate to an inspection pose, read a traffic
sign and choose a safe route according to the detected class. The design keeps
perception and navigation in separate ROS 2 packages:

| Package | Responsibility |
| --- | --- |
| `my_robot_ai_identification` | Camera image to traffic-sign class |
| `my_robot_ai_navigation` | Traffic-sign class to route decision |
| `my_robot_nav_control` | General navigation tools not tied to AI |

The navigation behaviour can be tested without YOLO. This is the recommended
first step in The Construct and before every real-robot test.

## 1. System architecture

```text
/camera/image_raw
        |
        v
traffic_sign_yolo
(my_robot_ai_identification)
        |
        | /traffic_sign  [std_msgs/msg/String]
        v
traffic_sign_navigation
(my_robot_ai_navigation)
        |
        v
       Nav2
```

The source of `/traffic_sign` is interchangeable:

```text
The Construct       ros2 topic pub -----------+
                                                |
Gazebo + YOLO       simulated camera -> YOLO --+--> /traffic_sign
                                                |
Real rUBot          USB camera -> YOLO --------+
```

The accepted values are:

| Message | Navigation behaviour |
| --- | --- |
| `left` | Left waypoint, then final pose |
| `right` | Right waypoint, then final pose |
| `stop` | Wait at the inspection pose, then continue |
| Any other value | Remain at the inspection pose |

This is a route-selection exercise in a known map. It does not estimate the
3D pose of the sign. The left and right waypoints must be configured in known
free areas of the map.

## 2. Build the new packages

Build the perception and AI-navigation packages:

```bash
cd ~/my_rUBot_mecanum
colcon build --symlink-install \
  --packages-select my_robot_ai_identification my_robot_ai_navigation
source install/setup.bash
```

The new implementation does not remove the previous YOLO nodes or launch
files. Both versions can be tested during the migration.

## 3. Configure the navigation route

The route files are located in:

```text
src/AI_Projects/my_robot_ai_navigation/config/
|-- traffic_sign_navigation_sw.yaml
`-- traffic_sign_navigation_real.yaml
```

Each pose uses `[x, y, yaw]` in the `map` frame:

```yaml
traffic_sign_navigation:
  ros__parameters:
    use_sim_time: true
    initial_pose: [0.0, 0.0, 0.0]
    inspection_pose: [0.5, -1.0, 1.57]
    left_waypoint: [-0.8, 0.3, 1.57]
    right_waypoint: [0.8, 0.3, 1.57]
    final_pose: [0.0, 1.3, 1.57]
    stop_duration: 3.0
```

Adapt every pose to the selected world and map. The inspection pose must face
the sign. All goals must be outside occupied and inflated costmap cells.

The navigation sequence is:

```text
initial pose
    |
    v
inspection pose
    |
    v
wait for /traffic_sign
    |-- left  --> left waypoint  --+
    |-- right --> right waypoint --+--> final pose
    |-- stop  --> wait -----------+
    `-- unknown --> remain stopped
```

## 4. Validation in The Construct without Ultralytics

Ultralytics is not required for this test. The terminal simulates the output
of the future YOLO detector.

### 4.1 Start Gazebo Sim

```bash
ros2 launch my_robot_bringup my_robot_bringup_gz.launch.py \
  world:=square_sign_left_ign.world \
  robot_model:=rubot/rubot_mecanum.urdf.xacro \
  x:=0.0 y:=0.0 yaw:=0.0
```

### 4.2 Start Nav2

```bash
ros2 launch my_robot_navigation2 navigation2_robot.launch.py \
  use_sim_time:=true \
  map_file:=map_square_sign_ign.yaml \
  params_file:=rubot_sw_ign.yaml
```

Check the robot localization in RViz before continuing.

### 4.3 Start traffic-sign navigation

```bash
ros2 launch my_robot_ai_navigation traffic_sign_navigation.launch.py \
  config_file:=traffic_sign_navigation_sw.yaml
```

Wait until the robot reaches the inspection pose and the terminal shows:

```text
Waiting for a traffic sign on /traffic_sign ...
```

### 4.4 Simulate the classification result

Publish one message from another sourced terminal:

```bash
ros2 topic pub --once /traffic_sign std_msgs/msg/String "{data: left}"
```

Restart the complete navigation test and repeat it with:

```bash
ros2 topic pub --once /traffic_sign std_msgs/msg/String "{data: right}"
ros2 topic pub --once /traffic_sign std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /traffic_sign std_msgs/msg/String "{data: unknown}"
```

Use these commands to inspect the interface:

```bash
ros2 topic info /traffic_sign
ros2 topic echo /traffic_sign
```

This test validates the route configuration, subscriber, decision logic and
Nav2 integration independently of the camera and YOLO.

## 5. Optional Gazebo validation with YOLO

Run this section on a local system or container where Ultralytics is already
available. The Construct test in the previous section remains the required
fallback.

First verify the simulated camera topic:

```bash
ros2 topic list | grep image
ros2 topic hz /camera/image_raw
```

Start the simple classifier interface:

```bash
ros2 launch my_robot_ai_identification traffic_sign_yolo.launch.py \
  image_topic:=/camera/image_raw \
  model:=yolo11n_cls_g1.pt \
  confidence:=0.70 \
  stable_frames:=3 \
  use_sim_time:=true
```

The node publishes:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/traffic_sign` | `std_msgs/msg/String` | Stable normalized class |
| `/traffic_sign/image` | `sensor_msgs/msg/Image` | Annotated image |

Verify the classifier before starting autonomous navigation:

```bash
ros2 topic echo /traffic_sign
ros2 run rqt_image_view rqt_image_view
```

Then start `traffic_sign_navigation.launch.py` as in Section 4.3. The
navigation node is unchanged: only the publisher of `/traffic_sign` has been
replaced.

## 6. Validation with the real rUBot

Perform the validation in stages. Do not diagnose navigation and perception
at the same time.

### 6.1 Start and verify the hardware

```bash
ros2 launch my_robot_bringup my_robot_bringup_robust_hw.launch.py
```

Check the essential interfaces:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic list | grep image_raw
```

The default YOLO camera topic is `/image_raw`. Use the topic reported by the
last command if the camera driver uses a different name.

### 6.2 Start localization and Nav2

```bash
ros2 launch my_robot_navigation2 navigation2_robot.launch.py \
  use_sim_time:=false \
  map_file:=my_map.yaml \
  params_file:=rubot_real.yaml
```

Place the physical robot at the configured initial pose. Verify its pose and
costmaps in RViz. Use **2D Pose Estimate** when necessary.

### 6.3 Validate navigation with a manual message

Start the real route configuration:

```bash
ros2 launch my_robot_ai_navigation traffic_sign_navigation.launch.py \
  config_file:=traffic_sign_navigation_real.yaml
```

When the robot reaches the inspection pose, publish:

```bash
ros2 topic pub --once /traffic_sign std_msgs/msg/String "{data: left}"
```

Do not continue until the inspection, left, right, stop and unknown cases are
safe with manual messages.

### 6.4 Add the real YOLO classifier

Start the detector with the real camera topic:

```bash
ros2 launch my_robot_ai_identification traffic_sign_yolo.launch.py \
  image_topic:=/image_raw \
  model:=yolo11n_cls_g1.pt \
  confidence:=0.70 \
  stable_frames:=3 \
  use_sim_time:=false
```

Before moving the robot, show each sign to the camera and verify:

```bash
ros2 topic echo /traffic_sign
```

Finally, restart `traffic_sign_navigation.launch.py` and execute the complete
camera-to-navigation experiment.

## 7. Troubleshooting

| Symptom | Check |
| --- | --- |
| No `/traffic_sign` with manual publication | Source the workspace and check topic type/name |
| Robot does not leave the inspection pose | Publish only after the waiting message appears |
| Goal is rejected | Check the pose in the global costmap and inflation layer |
| Wrong route is selected | Check lower-case values: `left`, `right`, `stop` |
| No camera images | Check the actual `image_raw` topic and camera device |
| YOLO produces no class | Reduce confidence temporarily and inspect the annotated image |
| Detection changes rapidly | Increase `stable_frames` |
| Simulation waits forever | Check `use_sim_time`, `/clock` and that Nav2 is active |

Test each layer separately in this order:

```text
Nav2 goal
  -> manual /traffic_sign
  -> YOLO topic output
  -> complete integrated system
```

## 8. Legacy implementation preserved for comparison

The following implementation is the previous working solution. Its nodes,
launch files and configuration files are intentionally preserved until the
new interface has been validated in simulation and on the real robot.

### Previous project description

We will describe the Computer Vision based method to identify the Traffic Sign.

Training models: 
- https://roboflow.com/
- https://github.com/ultralytics/ultralytics
- https://docs.ultralytics.com/es/usage/python/#how-do-i-train-a-custom-yolo-model-using-my-dataset

For this project we have created a new package "my_robot_ai_identification" where we have used 2 strategies to perform signal identification:
- Keras with tensorflow
- YOLO 

### Legacy 1. ROS 2 package installation

The needed Installation for YOLO identification is only to install "ultralytics" on the ROS2 Humble environment. Open a terminal and type:
````shell
pip install ultralytics
python3 -m pip uninstall -y opencv-python opencv-contrib-python numpy
python3 -m pip install numpy==1.26.4 opencv-python==4.9.0.80 --force-reinstall
python3 -m pip uninstall -y transforms3d
python3 -m pip install transforms3d==0.4.2
````

For simulation, you won't be able to use TheConstruct environment. You have to use your **Docker container ROS2 custom environment**:
- Use VScode and clone your project repository
- Edit `docker-compose.yaml` from `network_config/humble`
- Comment or delete the environment variables: 
    - ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    - ROS_STATIC_PEERS=192.168.1.54
    - CYCLONEDDS_URI=file:///config/cyclonedds_pc.xml
    - Choose DISPLAY:
        - DISPLAY=${DISPLAY} #Ubuntu
        - DISPLAY=host.docker.internal:0.0 #Windows 
- Verify in PC-win `entrypoint_pc.sh` has `LF` NOT `CRLF`
- Open a terminal in `network_config/humble` and write:
    ````bash
    docker compose up
    ````
- Install for graphical interface:
    - For Ubuntu: https://mac.getutm.app/
    - For Windows: https://sourceforge.net/projects/vcxsrv/files/latest/download
- Open a VScode window attached to the created container
- Clone your project repository
- open `.bashrc` file and add:
    ````xml
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source /root/my_rUBot_mecanum/install/setup.bash
    export GAZEBO_MODEL_PATH=/root/my_rUBot_mecanum/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
    cd /root/my_rUBot_mecanum
    ````
You have now your custom docker ROS2 workspace ready!


### Legacy 2. Robot navigation

To proceed with the signal identification we first bringup the robot and navigate from initial pose to final target.

- **Bringup the robot**
    - In simulation:
    ````shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml use_sim_time:=true x0:=0.5 y0:=-1.5 yaw0:=1.57 robot:=rubot/rubot_mecanum.urdf custom_world:=square4m_sign_left.world
    ````
    >Important: Include a traffic signal in the world. When using "square4m_sign.world" you can change the sign model on line 30 changing the traffic sign model name

- Generate a map
    - In simulation:
        ````shell
        ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=true
        ````
    - In real robot (rUBot or LIMO):
        ````shell
        ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=false
        ````
- Save the map in my_robot_navigation2/map folder with:
    ````shell
    cd src/Navigation_Projects/my_robot_navigation2/map/
    ros2 run nav2_map_server map_saver_cli -f map_square4m_sign
    ````
- **Navigate using the Map**:
    - In simulation:
        ````bash
        ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=true map_file:=map_square4m_sign.yaml params_file:=rubot_sw_lidar.yaml
        ````
        >In case we want to priorize the lidar data from odometry data we will use `rubot_sw_lidar.yaml`. Equivalent names are found for rUBot real robot.

        ![](./Images/07_Yolo/10_nav_sw.png)

    - In real robot rUBot:
        ````shell
        ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=map_project.yaml params_file:=rubot_real_lidar.yaml
        ````
        > Be sure the Odometry message is zero when starting the navigation.

### Legacy 3. Signal detection

First we want to test the classification model prediction:
- We have created a new node `rubot_rt_yolo_cls.py` that:
    - Subscribes to a ROS2 camera image topic.
    - Loads a YOLO classification model.
    - Runs real-time image classification.
    - Detects the most probable traffic sign class.
    - Publishes classification results in a custom ROS2 message.
    - Publishes an annotated image with prediction labels.
    - Uses configurable ROS2 parameters for model, topic, confidence, and image size.
    ````bash
    ros2 run my_robot_ai_identification rubot_rt_yolo_cls_exec \
    --ros-args \
    -p modelYolo:=best_cls.pt -p image_topic:=/image_raw
    ````
    > in Gazebo simulation environment the camera topic is `/camera/image_raw` 
Once the identification is validated, we have created a new node `rubot_identification_yolo_cls.py` that:
- Subscribes to a camera image topic.
- Loads a YOLO classification model.
- Classifies each camera image to detect a traffic sign.
- Publishes the prediction result to /Yolov8_Inference.
- Publishes an annotated image to /inference_result.
- Loads known traffic sign positions from a YAML file.
- Reads the robot pose from TF: map -> base_link.
- Checks if the robot is close enough to the detected sign.
- Applies simple traffic-sign logic: STOP, Ceda, Prohibido, Derecha, Izquierda.
- Creates a navigation waypoint near the detected sign.
- Publishes the waypoint to /traffic_waypoint.
- Uses cooldown and hold times to avoid repeated reactions.

In `config` folder we have to review the following files for the correct parameters:
- `sign_positions_real.yaml` 
- `yolo_params_real.yaml` 

The schematic nodes, topics and messages are shown below:
    ![](./Images/07_Yolo/09_yolo_detection_topics.png)

**Gazebo Simulation** Test:
- If you want to execute in Gazebo simulation, you have to execute:
    ````shell
    ros2 launch my_robot_ai_identification rubot_identification_yolo.launch.py yolo_params_file:=yolo_params_sw.yaml signs_file:=sign_positions_sw.yaml
    ````
    > Review the correct parameters!

**Hardware** Test in real robot:
- If you want to execute on real `rUBot robot`, you have to execute:
    ````shell
    ros2 launch my_robot_ai_identification rubot_identification_yolo.launch.py yolo_params_file:=yolo_params_real.yaml signs_file:=sign_positions_real.yaml
    ````
    > Review the correct parameters!

To obtain a **wp published** in `/traffic_waypoint` topic, you have to:
- the detected sign has to be in the `sign_positions_real.yaml` file with the correct coordinates and name. You can add more signs and positions if needed.
- place the robot closer to the signal (less than the threshold of 0.75 m, review the parameter `front_distance` in `yolo_params_real.yaml`). Not considered in the current simplified version.
- You need to launch `navigation2_robot.launch.py` to have the TF between `map` and `base_link` frames, needed for the correct detection of the sign position respect to the robot. Not considered in the current simplified version.

### Legacy 4. Custom navigation with signal detection

Considering the previous `object_detection` node, we have created a new `custom_nav2` node that:
- Creates a ROS2 navigation task using Nav2.
- Reads `yolo_targets_real.yaml` the initial pose, traffic sign waypoint and final target pose.
- Uses BasicNavigator from Nav2 Simple Commander.
- Sets the robot initial pose in the map.
- Waits until Nav2 becomes active.
- Navigates the robot to the predefined signal waypoint.
- Subscribes to /traffic_waypoint.
- Waits for a waypoint generated by the YOLO traffic-sign node.
- Navigates to the received traffic waypoint if available.
- Continues to the final target pose.


In `config` folder we have to review the following files for the correct parameters:
- `yolo_targets_real.yaml`

The schematic nodes, topics and messages are shown below:
    ![](./Images/07_Yolo/12_yolo_custom_nav2.png)

**Gazebo Simulation** Test:
- Launch the `custom_nav2` node with:
    ````shell
    ros2 launch my_robot_ai_identification rubot_targets_yolo.launch.py targets_file:=yolo_targets_sw.yaml
    ````
    > Review the correct parameters!

**Hardware** Test in real robot:
- Launch the `custom_nav2` node with:
    ````shell
    ros2 launch my_robot_ai_identification rubot_targets_yolo.launch.py targets_file:=yolo_targets_real.yaml
    ````
    > Review the parameters!

We have created a full launch file `ai_navigation.launch.py` that launches the 3 nodes together:
- `Navigation2 stack`
- `object_detection` node
- and starts after a controlled delay, the `custom_nav2` node that integrates the detected waypoint in the navigation2 stack

You can launch all nodes with only one launch file:
```bash
ros2 launch my_robot_ai_identification ai_navigation.launch.py \
  map_file:=map_square4m_sign.yaml \
  params_file:=rubot_sw_lidar.yaml \
  use_sim_time:=true \
  yolo_params:=yolo_params_sw.yaml \
  nav_params:=yolo_targets_sw.yaml \
  signs_file:=sign_positions_sw.yaml \
  nav_start_delay:=2.0
```


| AI Identification and Navigation video | Code execution video |
|----------|------------|
| [▶️ rUBot Traffic signal Detection & Autonomous Navigation](./Images/07_Yolo/Yolo_left.mp4) | [▶️ rUBot code execution](./Images/07_Yolo/YoloSignalWaypoint.webm) |
