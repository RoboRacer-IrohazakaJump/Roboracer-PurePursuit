# The Pure Pursuit Algorithm Approach

The pure pursuit algorithm is the common algorithm for autonomous mobile robot for controlled navigation over a fixed path. To fully utilize this algoritm, the system requires a good localization for accurately guide the mobile robot towards the set path. If the localization algorithm could not appropriately locate the robot position then it will propagately affecting the performance of the pure pursuit algorithm.

We adapt this navigation system based on the available hardware and sensor from the mobile robot which can be acquire from the technical guide here at [Autodrive Ecosystem Technical Guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2025/). In summary we are taking account the fact that the autonomous mobile robot has LIDAR and Wheel encoder both left and right. It also has the ackermaan steering with differential drive.

## Methodology
Our methodology can be read in more detail [here](METHOD.md)

## Installation guide
The installation guide for the submission or the devkit environment can be found [here](GUIDE.md)