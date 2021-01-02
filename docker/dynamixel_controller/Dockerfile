FROM ros:kinetic
LABEL maintainer "hirokiyokoyama <h-yokoyama@craft-server.co.jp>"
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-dynamixel-controllers \
    ros-kinetic-xacro \
    && rm -rf /var/lib/apt/lists/*
COPY ./dynamixel_joint_state_publisher.py /opt/ros/kinetic/lib/robot_state_publisher
COPY ./urdf /urdf
COPY ./launch /launch

# to resolve *.local hostname
RUN apt-get update && apt-get install --no-install-recommends -y \
    avahi-daemon \
    libnss-mdns \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i -e 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf

RUN cd / && git clone https://github.com/arebgun/dynamixel_motor.git

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["roslaunch", "/launch/pan_tilt.launch"]

