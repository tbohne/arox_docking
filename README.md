# Autonomous Docking: AROX -> Inductive Charging Station

- <u>Assumed:</u> **State: CONTAINER_PROXIMITY**
    - robot drove near the container, e.g. based on GPS
    - *postcondition*: robot faces container in certain area
--------------------------------------------------------------------------
- **State: DETECT_CONTAINER** (laser scans, possibly camera)
    - *precondition*: robot faces container in certain area
    - *postcondition*: localized container + ramp
- **State: ALIGN_ROBOT_TO_RAMP** (defined pose)
    - *preconditions*: 
        - robot in front of container
        - localized container + ramp
        - ramp lowered
    - *postcondition*: robot aligned in front of ramp
- **State: DRIVE_INTO_CONTAINER**
    - *preconditions*:
        - robot aligned in front of ramp
        - free space in container
    - *postcondition*: robot inside the container
- **State: LOCALIZE_CHARGING_STATION** (laser scans, possibly camera)
    - *precondition*: robot inside the container
    - *postcondition*: localized charging station
- **State: ALIGN_ROBOT_TO_CHARGING_STATION** (defined pose)
    - *precondition*: localized charging station
    - *postcondition*: robot aligned with charging station
- **State: DOCK**
    - *precondition*: robot aligned with charging station
    - *postcondition*: robot charging

## Undocking

- <u>Assumed:</u> **State: INSIDE_CONTAINER**
    - *postcondition*: robot inside the container
- **State: DETECT_ENTRY**
    - *precondition*: robot inside the container
    - *postcondition*: localized container entry
- **State: DRIVE_OUT_OF_CONTAINER**
    - *precondition*: localized container entry
    - *postcondition*: robot located in front of the container

# Dependencies / Compatible Branches

- [arox_docker](https://git.ni.dfki.de/arox/arox_docker): dockerization of the AROX system
    - branch: `noetic`
    - compatible branches within docker container:
        - [arox_navigation_flex](https://git.ni.dfki.de/arox/arox_core/arox_navigation_flex): `low_yaw_goal_tolerance`
        - [arox_launch](https://git.ni.dfki.de/arox/arox_core/arox_launch): `frame_launch`
        - [arox_indoor_navi](https://git.ni.dfki.de/arox/arox_core/arox_indoor_navi): `sim_launch_detection_config`
        - [map_langsenkamp](https://git.ni.dfki.de/zla/map_langsenkamp): `feature_new_map`
- [arox_description](https://git.ni.dfki.de/arox/arox_core/arox_description): ROS launch files and URDF model for the AROX system
    - branch: `feature_flying_sick_tim`
- [container_description](https://git.ni.dfki.de/arox/container_description): ROS launch files and URDF model for the mobile container (charge station)
    - branch: `feature_simple_collisions`
- [innok_heros_description](https://git.ni.dfki.de/arox/innok_heros/innok_heros_description): URDF description for Innok Heros robot
    - branch: `arox_noetic`
- [innok_heros_driver](https://git.ni.dfki.de/arox/innok_heros/innok_heros_driver): ROS driver for the Innok Heros robot platform
    - branch: `master`
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/): URDF and gazebo plugin to provide simulated data from Velodyne laser scanners
    - branch: `master`
- [gazebo_langsenkamp](https://git.ni.dfki.de/zla/gazebo_langsenkamp): Langsenkamp world (test field)
    - branch: `master`

## Usage

- run simulation (with GUI): `roslaunch arox_description launch_arox_sim.launch gui:=true`
- spawn container: `roslaunch container_description spawn.launch`
- spawn AROX: `roslaunch arox_description spawn.launch`
- run AROX controllers: `roslaunch arox_description run_controllers.launch`
- run docker container named 'arox_docking': `aroxstartdocker arox_docking` (alias)
    - launch outdoor simulation: `roslaunch arox_launch arox_sim_outdoor.launch`
- provide docking / undocking actions: `roslauch arox_docking docking.launch`
- start dummy docking / undocking procedure: `rosrun arox_docking start_smach`

## Plan Executor (within docker container)

- access exploration GUI: `http://localhost/exploration_gui/`
- run AROX engine: `rosrun arox_engine arox_engine.py`
- run AROX planner: `rosrun arox_planning arox_planner.py`

## Control AROX

- launch keyboard control: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Visualize Sensor Data

- [rViz](https://wiki.ros.org/rviz)
    - fixed frame: `map`
    - open the provided config `conf.rviz`

## Open Container

- `rostopic pub -1 /container/rampA_position_controller/command std_msgs/Float64 "data: 2.0"`

