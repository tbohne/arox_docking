# Autonomes Docking: AROX -> induktive Ladestation

**Teilprobleme:**
- <u>VORAUSGESETZT</u> - **Schritt 0: Roboter anhand von Sensordaten vor Container navigieren** (GPS, Laserscans, ggf. Kamera)
    - *Nachbedingung*: Roboter steht in best. Bereich vor Container
--------------------------------------------------------------------------
- **Schritt 1: Container erkennen** (Laserscans, ggf. Kamera)
    - *Vorbedingung*: Roboter steht in best. Bereich vor Container
    - *Nachbedingung*: Container + Rampe lokalisiert
- **Schritt 2: Roboter vor der Rampe ausrichten** (definierte Pose +/- x)
    - *Vorbedingungen*: 
        - Roboter vor Container
        - Container + Rampe lokalisiert
        - Rampe unten
    - *Nachbedingung*: Roboter vor Rampe ausgerichtet
- **Schritt 3: In Container fahren und anhalten**
    - *Vorbedingungen*:
        - Roboter vor Rampe ausgerichtet
        - Freiraum im Container
    - *Nachbedingung*: Roboter im Container
- **Schritt 4: Ladestation lokalisieren** (Laserscans, ggf. Kamera)
    - *Vorbedingung*: Roboter im Container
    - *Nachbedingung*: Ladestation lokalisiert
- **Schritt 5: An Ladestation ausrichten** (definierte Pose +/- x)
    - *Vorbedingung*: Ladestation lokalisiert
    - *Nachbedingung*: Roboter an Ladestation ausgerichtet
- **Schritt 6: Docking an Ladestation**
    - *Vorbedingung*: Roboter an Ladestation ausgerichtet
    - *Nachbedingung*: Roboter lädt

# Dependencies / Compatible Branches

- [arox_docker](https://git.ni.dfki.de/arox/arox_docker): dockerization of the AROX system
    - branch: `noetic`
    - compatible branches within docker container:
        - [arox_navigation_flex](https://git.ni.dfki.de/arox/arox_core/arox_navigation_flex): `low_yaw_goal_tolerance`
        - [arox_launch](https://git.ni.dfki.de/arox/arox_core/arox_launch): `frame_launch`
        - [arox_indoor_navi](https://git.ni.dfki.de/arox/arox_core/arox_indoor_navi): `sim_launch_detection_config`
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

