#/bin/bash

#docker run -it --rm -v $(pwd):/ezbot-v2 -w /ezbot-v2 -e "TERM=xterm-256color" ros:jazzy-ros-base /bin/bash
#docker run -it --rm -v $(pwd):/ezbot-v2 -w /ezbot-v2 -e "TERM=xterm-256color" osrf/ros:jazzy-desktop-full /bin/bash
#docker run -it --rm --rm --net=host --gpus=all -e DISPLAY=$DISPLAY -v $(pwd):/ezbot-v2 -w /ezbot-v2 -e "TERM=xterm-256color" rosdep_done /bin/bash



# this is getting way more FPS than the previous one
# docker run -it --rm --rm --net=host --ipc=host --pid=host --privileged -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp --gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v $(pwd):/ezbot-v2 -w /ezbot-v2 -e "TERM=xterm-256color" rosdep_done /bin/bash 
docker run -it --rm --rm --net=host --ipc=host --pid=host --privileged -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp --gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v $(pwd):/ezbot-v2 -w /ezbot-v2 -e "TERM=xterm-256color" ezbot-v2-dev /bin/bash 
# I also added --ipc=host --pid=host to have containers be able to see each other.
# I have not tested this on a network with two machines yet
# I also added --privileged to have access to all the devices, not tested yet (well, I can see them in /dev/ but I have not tested them yet)

