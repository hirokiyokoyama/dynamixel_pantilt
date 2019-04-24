FROM ros:kinetic
LABEL maintainer "hirokiyokoyama <h-yokoyama@craft-server.co.jp>"
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-dynamixel-controllers \
    ros-kinetic-xacro \
    && rm -rf /var/lib/apt/lists/*
COPY ./pan_tilt.launch /
COPY ./dynamixel_joint_state_publisher.py /opt/ros/kinetic/lib/robot_state_publisher
COPY ./urdf /urdf

# to resolve *.local hostname
RUN apt-get update && apt-get install --no-install-recommends -y \
    avahi-daemon \
    libnss-mdns \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i -e 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf 

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["roslaunch", "/pan_tilt.launch"]
