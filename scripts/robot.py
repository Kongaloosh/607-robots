#!/usr/bin/env python

__author__ = 'kongaloosh'

from beginner_tutorials.msg import *
from beginner_tutorials.srv import *
import dynamixel
import sys
import rospy


class Robot(object):

    def __init__(self):

        serial_port = '/dev/ttyUSB0'
        self.serial = dynamixel.SerialStream(port=serial_port,
                                        baudrate=1000000,
                                        timeout=1)
        self.net = dynamixel.DynamixelNetwork(self.serial)
        servos = [2, 3]
        for servo_id in servos:
            new_dynamixel = dynamixel.Dynamixel(servo_id, self.net)
            self.net._dynamixel_map[servo_id] = new_dynamixel

        if not self.net.get_dynamixels():
            print("no servos found")
            sys.exit(0)

        print("setting register constants...")
        for actuator in self.net.get_dynamixels():
            actuator._set_ccw_compliance_slope(32)
            actuator._set_cw_compliance_slope(32)
            actuator._set_ccw_compliance_margin(1)
            actuator._set_cw_compliance_margin(0)

        rospy.init_node('robot', anonymous=True)
        self.observation_publisher = rospy.Publisher('robot_observations', servo_state, queue_size=10)

        rospy.Timer(rospy.Duration(1.0/10), self.observation_callback)

    def observation_callback(self):
        # Make a header ???
        self.observation_publisher.publish(*self.get_observations())

    def get_observations(self):
        """Turns the Current Values of the Robot Servos Into a Tuple to be Passed as a Message"""
        state = ()
        for actuator in self.net.get_dynamixels():
            state += (
                actuator.current_load,
                actuator.current_speed,
                actuator.current_temperature,
                actuator.current_voltage,
                actuator.moving,
                actuator.goal_position,
                actuator.current_position
            )

        return state