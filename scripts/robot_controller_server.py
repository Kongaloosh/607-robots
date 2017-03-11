#!/usr/bin/env python
from beginner_tutorials.srv import robot_command
import rospy
import sys
import dynamixel

serial_port = '/dev/ttyUSB0'
serial = dynamixel.SerialStream(port=serial_port,
                                baudrate=1000000,
                                timeout=1)
net = dynamixel.DynamixelNetwork(serial)
servos = [2, 3]
for servo_id in servos:
    new_dynamixel = dynamixel.Dynamixel(servo_id, net)
    net._dynamixel_map[servo_id] = new_dynamixel

if not net.get_dynamixels():
    print("no servos found")
    sys.exit(0)

print("setting register constants...")
for actuator in net.get_dynamixels():
    actuator._set_ccw_compliance_slope(32)
    actuator._set_cw_compliance_slope(32)
    actuator._set_ccw_compliance_margin(1)
    actuator._set_cw_compliance_margin(0)


def handle_command(request):
    actuator = net.get_dynamixels()[0]
    actuator.moving_speed = 100
    actuator.torque_enable = 1
    actuator.torque_limit = 800
    actuator.max_torque = 800
    actuator.goal_position = request.goal_pos_2

    actuator = net.get_dynamixels()[1]
    actuator.moving_speed = 100
    actuator.torque_enable = 1
    actuator.torque_limit = 800
    actuator.max_torque = 800
    actuator.goal_position = request.goal_pos_3
    net.synchronize()


def add_two_ints_server():
    rospy.init_node('robot_command_servo')
    s = rospy.Service('robot_command', robot_command, handle_command)
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()

'''
Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest
object for you (you're free to pass in your own instead). The return value is an AddTwoIntsResponse object.
If the call fails, a rospy.ServiceException may be thrown, so you should setup the appropriate try/except block.
'''
