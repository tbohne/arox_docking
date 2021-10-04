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

# Compatible Branches

## within docker container
- `arox_navigation_flex`: `low_yaw_goal_tolerance`
- `arox_launch`: `frame_launch`
- `arox_indoor_navi`: `sim_launch_detection_config`

## out of docker container
- `container_description`: `feature_simple_collisions`
- `arox_description`: `feature_flying_sick_tim`