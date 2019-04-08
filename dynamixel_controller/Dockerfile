FROM ros:kinetic
LABEL maintainer "hirokiyokoyama <hiroki.yokoyama@okadanet.org>"
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-dynamixel-controllers \
    && rm -rf /var/lib/apt/lists/
COPY ./pan_tilt.launch /
COPY ./pan_tilt.urdf /
COPY ./dynamixel_joint_state_publisher.py /opt/ros/kinetic/lib/robot_state_publisher
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "/pan_tilt.launch"]
