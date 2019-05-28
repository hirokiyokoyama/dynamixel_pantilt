docker run -it --rm --privileged hirokiyokoyama/dynamixel-jetson /dynamixel_motor/dynamixel_driver/scripts/set_servo_config.py --ccw-angle-limit=1023 1 2
docker run -it --rm --name=dynamixel --privileged --net=host -e ROS_HOSTNAME=tx2-1.local hirokiyokoyama/dynamixel-jetson
