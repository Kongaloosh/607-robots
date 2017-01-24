#!/usr/bin/env python
__author__ = 'kongaloosh'

import dynamixel
import sys
import rospy
from beginner_tutorials.msg import servo_state
#serial_port = '/dev/tty.usbserial-AI03QD8V'
serial_port = '/dev/ttyUSB0'


def get_observations(net):
    """Turns the Current Values of the Robot Servos Into a Tuple to be Passed as a Message"""
    state = ()
    for actuator in net.get_dynamixels():
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


def talker():
    """Brodcasts the Current State of the Robot"""
    serial = dynamixel.SerialStream(
                                port=serial_port,
                                baudrate=1000000,
                                timeout=1
        )
    net = dynamixel.DynamixelNetwork(serial)
    servos = [2, 3]
    for servo_id in servos:
        new_dynamixel = dynamixel.Dynamixel(servo_id, net)
        net._dynamixel_map[servo_id] = new_dynamixel

    if not net.get_dynamixels():
        print("no servos found")
        sys.exit(0)

    pub = rospy.Publisher('robot_obeservation', servo_state, queue_size=10)     # publishing to 'chatter' topic with string type
    rospy.init_node('talker', anonymous=True)                   # initializes node with name
    rate = rospy.Rate(10)                                       # rate of sending messages (10hz)
    while not rospy.is_shutdown():
        pub.publish(*get_observations(net))
        rate.sleep()                                            # sleeps to achieve right frequency

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
