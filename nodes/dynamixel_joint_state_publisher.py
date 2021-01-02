#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

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
        
        rate = self.declare_parameter('rate', 20).value
        r = self.create_rate(rate)
        
        self.controllers = self.declare_parameter('dynamixel_controllers', []).value
        self.joint_states = dict({})
        
        for controller in sorted(self.controllers):
            joint = controller#TODO: rospy.get_param(controller)['joint_name']
            self.joint_states[joint] = JointStateMessage(joint, 0.0, 0.0, 0.0)
            
        # Start controller state subscribers
        [self.create_subscription(JointStateDynamixel, c + '/state', self.controller_state_handler) for c in self.controllers]
     
        # Start publisher
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 1)
       
        self.get_logger().info("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
       
        while rclpy.ok():
            self.publish_joint_states()
            r.sleep()
           
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
