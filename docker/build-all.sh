#/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $SCRIPT_DIR/..

docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -f docker/dockerfile-pre-rosdep . -t ezbot-dev-pre-rosdep:latest --progress=plain
docker build -f docker/dockerfile-post-rosdep --no-cache . -t ezbot-v2-dev --progress=plain
#docker build -f docker/dockerfile-post-rosdep --no-cache . -t ezbot-v2-dev --progress=plain