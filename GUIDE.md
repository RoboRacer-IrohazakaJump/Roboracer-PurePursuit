# Running the Algorithm Code
## Environment Setup
1. First, you need to pull [AutoDRIVE Simulator docker image](https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_sim) from DockerHub
```
docker pull autodriveecosystem/autodrive_roboracer_sim:2025-cdc-tf-practice
```
2. Next, you need to pull [AutoDRIVE Devkit docker image](https://hub.docker.com/r/autodriveecosystem/autodrive_roboracer_api) from DockerHub (or you can try using our docker image)
```
# Using AutoDRIVE Devkit docker image
docker pull autodriveecosystem/autodrive_roboracer_api:2025-cdc-tf-practice
# Our submitted docker image
docker pull hatsuharuyasa/autodrive_submission:phase-1
```
## Container Execution
1. Enable display forwarding for simulator, and run the simulator container at `entrypoint`
```
xhost local:root
docker run --name autodrive_roboracer_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_roboracer_sim:2025-cdc-tf-practice
```
3. Enable display forwarding for devkit, and run the devkit container at `entrypoint`
```
xhost local:root
docker run --name autodrive_roboracer_api --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env DISPLAY --privileged --gpus all hatsuharuyasa/autodrive_submission:phase-1
```
## GUI Mode Operations
1. Launch AutoDRIVE Simulator in `graphics` mode (camera rendering will be enabled)
```
./AutoDRIVE\ Simulator.x86_64
```
3. Launch AutoDRIVE Devkit in `graphics` mode (RViz rendering will be enabled)
```
ros2 launch autodrive_roboracer bringup_graphics.launch.py
```
If you run both containers on the same machine, just hit the `connection` button in the simulator GUI menu panel, then the status next to it will become `Connected!`. Once the connection has been established, you can choose between `Manual` and `Autonomous` driving modes for the `Driving Mode`.

# Implement Different Algorithm
If you want to implement different algorithm, pull the AutoDRIVE devkit docker image, then:
1. Navigate to `src` directory in the docker, and then create package:
```
ros2 pkg create --build-type ament_python algo_name
```
2. Natigate to `algo_name/algo_name` and create and python file to write your algorithm. Additionally, make sure there is `__init__.py` in that directory.
3. Next, you need to add an entry point / nodes within the `console_scripts` brackets of the `entry_points` field in `src/algo_name/setup.py` file. For example:
```
entry_points={
    'console_scripts': [
        'controller_node = algo_name.controller_node:main',
    ],
}
```
4. Back to `/home/autodrive_devkit`, build your package
```
colcon build --packages-select algo_name
```
5. Next, source the setup files
```
source install/setup.bash
```
6. Finally, you can try out your algorithm by executing:
```
ros2 run algo_name controller_node
```
# Automate Running
To automate your algorithm, so that when you hit the connection button, the vehicle runs right after the status becomes `Connected!`, you can edit the `autodrive_devkit.sh` in `/home` directory as can be seen in [here](autodrive_devkit.sh).

## References
If you ever encounter any error, or the information of the vehicle, sensors, etc., please refer to the following references:
1. [Technical Guide: Vehicle, Softwares, etc.](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2025/)
2. [ROS2 Tutorial - Packages, Nodes, Communications](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
3. If you wonder about how the messsage from each sensor looks like, you search it on google. For example, "LaserScan ROS2": [LaserScan](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
