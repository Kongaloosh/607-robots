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
        # Publishes robot state
        self.observation_publisher = rospy.Publisher('robot_observations', servo_state, queue_size=10)
        # service for controlling servos
        self.robot_controller_server = rospy.Service('robot_controller', robot_command, self.command_handler)
        rospy.wait_for_service('robot_controller')
        self.start_controller = rospy.ServiceProxy('robot_controller', robot_command, self.command_handler)
        self.start_controller(512,512)
	# timer which defines callback for the publisher
        rospy.Timer(rospy.Duration(1.0/10), self.observation_callback)

    def observation_callback(self, timer):
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

    def command_handler(self, request):
        actuator = self.net.get_dynamixels()[0]
        actuator.moving_speed = 100
        actuator.torque_enable = 1
        actuator.torque_limit = 800
        actuator.max_torque = 800
        actuator.goal_position = request.goal_pos_2

        actuator = self.net.get_dynamixels()[1]
        actuator.moving_speed = 100
        actuator.torque_enable = 1
        actuator.torque_limit = 800
        actuator.max_torque = 800
        actuator.goal_position = request.goal_pos_3

        self.net.synchronize()
        return request.goal_pos_2, request.goal_pos_3

if __name__ == '__main__':
    try:
        robot = Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
