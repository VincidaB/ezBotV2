This repository is for the second revision or the ezBot robot that can be seen at [insert link of ensmasteel's repo]
Here can be found ROS2 humble nodes, launch files, and simulation tools for the new robot.
To see the elctronics design, see [insert other repository].
For the CAD design, see [insert catia repository]



For the simulation, the following dependencies need to be installed :
`nvidia-container-toolkit`

# To run the simulation

```bash
git clone https://github.com/VincidaB/ezBotV2.git
cd ezBotV2
./docker/build-all.sh
xhost +local:* 
./docker/run.sh
```

In the container : 

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ezbot-v2-simulation multirobot-simulation.launch.py
```