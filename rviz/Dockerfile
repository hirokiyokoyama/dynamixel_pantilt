FROM ros:kinetic
LABEL maintainer "hirokiyokoyama <hiroki.yokoyama@okadanet.org>"
RUN apt-get update && apt-get install -y \
    ros-kinetic-rviz \
    && rm -rf /var/lib/apt/lists/
COPY ./entrypoint.sh /
COPY ./pantilt.rviz /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["rosrun", "rviz", "rviz", "-d", "/pantilt.rviz"]
