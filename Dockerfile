FROM ros:melodic-ros-core-bionic as base

SHELL ["/bin/bash", "-o", "pipefail","-c"]

WORKDIR LexxHard-MainBoard-Updator
COPY ./entrypoint.sh .
WORKDIR catkin_ws
COPY ./catkin_ws .

RUN         apt-get update;\
            apt-get install -y --no-install-recommends build-essential;\
            rm -rf /var/lib/apt/lists/*;

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make;"

ENTRYPOINT ["../entrypoint.sh"]