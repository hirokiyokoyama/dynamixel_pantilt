FROM ros:kinetic
LABEL maintainer "hirokiyokoyama <hiroki.yokoyama@okadanet.org>"
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/
COPY ./pan_tilt.launch /
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "/pan_tilt.launch"]
