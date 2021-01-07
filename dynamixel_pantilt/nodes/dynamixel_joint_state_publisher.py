#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState as JointState
from dynamixel_msgs.msg import JointState as JointStateDynamixel

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_state_publisher')
        
        self.controllers = self.declare_parameter('dynamixel_controllers', []).value
        self.joint_states = dict({})
        
        for controller in sorted(self.controllers):
            self.get_logger().info('Getting joint name from %s' % controller)
            client = self.create_client(GetParameters, '/{}/get_parameters'.format(controller))
            client.wait_for_service()
            future = client.call_async(GetParameters.Request(names=['joint_name']))
            rclpy.spin_until_future_complete(self, future)
            joint = None
            if future.result():
                joint = future.result().values[0].string_value
            if joint is None:
                self.get_logger().error('Could not get joint name from %S' % controller)
                continue
            self.joint_states[joint] = JointStateMessage(joint, 0.0, 0.0, 0.0)
            self.create_subscription(JointStateDynamixel, '/{}/state'.format(controller), self.controller_state_handler, 1)
            
        # Start publisher
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 1)
        rate = self.declare_parameter('rate', 20).value
        self.get_logger().info("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
        self.timer = self.create_timer(1./rate, self.publish_joint_states)
           
    def controller_state_handler(self, msg):
        js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)
        self.joint_states[msg.name] = js
       
    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
       
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
           
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.joint_states_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    try:
        s = JointStatePublisher()
        rclpy.spin(s)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
