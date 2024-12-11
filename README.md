[![build](https://github.com/KABAM-Robotics/fleet_adapter_kabam/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/KABAM-Robotics/fleet_adapter_kabam/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

## **What Is This?**

This repository contains a full-control Robot Middleware Framework (RMF) fleet adapter compatible with Kabam robots (Eg. Co-Lab, Halo, etcetera) which bridges RMF interactions with Kabam Robot Application Programming Interface (API) to allow for autonomous navigation given an RMF deployment site.

## **Dependencies** :books:

*    Ubuntu `22.04` LTS
*    ROS2 - Humble
*    [Open-RMF on Humble Hawksbill - Sync (2023-12-29)](https://github.com/open-rmf/rmf/releases/tag/release-humble-231229)

## **Build** :hammer:

```bash
cd $HOME
```

```bash
git clone git@bitbucket.org:cognicept/fleet_adapter_kabam.git --depth 1 --single-branch --branch master 
```

```bash
cd $HOME/fleet_adapter_kabam
```

```bash
docker build -t fleet_adapter_kabam:humble .
```

```bash
docker run -it --rm \
  --name fleet_adapter_kabam_c \
  --network=host \
  -v /dev/shm:/dev/shm \
  -v ./configs/config.yaml:/fleet_adapter_kabam_ws/src/fleet_adapter_kabam/config/config.yaml \
  -v ./configs/nav_graph.yaml:/fleet_adapter_kabam_ws/src/fleet_adapter_kabam/config/nav_graph.yaml \
  fleet_adapter_kabam:humble bash -c \
"ros2 run fleet_adapter_kabam fleet_adapter \
--config_file /fleet_adapter_kabam_ws/src/fleet_adapter_kabam/config/config.yaml \
--nav_graph /fleet_adapter_kabam_ws/src/fleet_adapter_kabam/config/nav_graph.yaml \
--server_uri ws://localhost:8000/_internal"
```

## **Configure**
For configuration for custom Kabam Robotics robot deployment, there are 2 files a user is required to modify:

### 📃 `config.yaml`
Update the following variables in this file:

- 🔸 `robot_api_prefix`: IP address of robot. Please consult vendor for information.
- 🔸 `maps`: Transforms between RMF and Smart+ Maps.
  ```bash
  maps:
  INSERT_SMART+_MAP_NAME:
    ori_diff: 1.57  # degree
    rmf: [[12.43046686746988, -9.682210090361446], # at_exit
        [22.813091114457833, -9.68421686746988], # next_to_dock
        [29.149710090361452, -9.692390813253013], 
        [35.00181475903615, -9.680301204819278]] # at_glass_door_int
    robot: [[3.612, 27.603],
        [3.155, 16.132],
        [3.551, 8.656],
        [3.156, 3.652]]
  ```
- 🔸 `robots`: RMF waypoint specifications.
  ```bash
  robots:
  # Here the user is expected to append the configuration for each robot in the
  # fleet.
  # Configuration for first robot in this fleet
  INSERT_ROBOT_NAME_HERE:
    robot_config:
      max_delay: 10.0 # allowed seconds of delay of the current itinerary before it gets interrupted and replanned
      map_exit_point:
        lift1:
           INSERT_SMART+_MAP_NAME: "INSERT_LIFT_ACCESS_RMF_WAYPOINT_NAME"
    rmf_config:
      robot_state_update_frequency: 0.5
      start:
        map_name: "INSERT_SMART+_MAP_NAME"
        waypoint: "INSERT_START_RMF_WAYPOINT_NAME"
        orientation: 0
      charger:
        waypoint: "INSERT_DOCK_RMF_WAYPOINT_NAME"
  ```

### 📃 `nav_graph.yaml`
Please generate and replace this file with the `.yaml` navigation graph, given a user's custom `.building.yaml` RMF Map file.
Follow the [official instructions here.](https://github.com/open-rmf/rmf_traffic_editor?tab=readme-ov-file#building-map-tools)

## **Verify** :white_check_mark:

Once `fleet_adapter_kabam` is running, you should see something similar to the following output:

```bash
[INFO] [1728452767.484285693] [CoLab_fleet_adapter]: Parameter [discovery_timeout] set to: 60.000000
```
`fleet_adapter_kabam` would start looking for **RMF Traffic Node**. 

## **Contributions & Feedback**

**We welcome contributions!** Please see the [contribution guidelines](https://github.com/KABAM-Robotics/fleet_adapter_kabam/blob/main/CONTRIBUTING.md).

For **feature requests** or **bug reports**, please file a [GitHub Issue](https://github.com/KABAM-Robotics/fleet_adapter_kabam/issues).

## **Maintainer(s)**
- Gary Bey

