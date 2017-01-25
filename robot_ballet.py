#
# This is a script which demonstrates all of the functionality requested in the assignment
#

__author__ = 'ksongaloosh'

import dynamixel
import sys
from datetime import datetime
from updated_lib_robotis_hack import *
import numpy as np

central_position = 512                  # (range: 0 to 1023)

serial_port = '/dev/tty.usbserial-AI03QD8V'
# serial_port = '/dev/ttyUSB0'

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

print("servos found")
print("setting register constants...")
for actuator in net.get_dynamixels():
    actuator._set_ccw_compliance_slope(32)
    actuator._set_cw_compliance_slope(32)
    actuator._set_ccw_compliance_margin(1)
    actuator._set_cw_compliance_margin(0)
print("controlling via encoder...")
print("     moving to centre")

while abs(net.get_dynamixels()[0].current_position - 512) > 2 and \
      abs(net.get_dynamixels()[1].current_position - 512) > 2:
    for actuator in net.get_dynamixels():
        actuator.moving_speed = 100
        actuator.torque_enable = 1
        actuator.torque_limit = 800
        actuator.max_torque = 800
        actuator.goal_position = 512
    net.synchronize()

# ===================================================================================================================
#                                           Move the Servos
# ===================================================================================================================

print("     performing ballet")
goal_pos = central_position + 10
while goal_pos < 600:
    while abs(net.get_dynamixels()[0].current_position - goal_pos) > 5 and \
          abs(net.get_dynamixels()[1].current_position - goal_pos) > 5:
        for actuator in net.get_dynamixels():
            actuator.moving_speed = 50
            actuator.torque_enable = 1
            actuator.torque_limit = 700
            actuator.max_torque = 700
            actuator.goal_position = goal_pos
        net.synchronize()
    goal_pos += 20

print("     moving the other way")
goal_pos = central_position
while goal_pos > 300:
    while abs(net.get_dynamixels()[0].current_position - goal_pos) > 5 and \
          abs(net.get_dynamixels()[1].current_position - goal_pos) > 5:
        for actuator in net.get_dynamixels():
            actuator.moving_speed = 50
            actuator.torque_enable = 1
            actuator.torque_limit = 700
            actuator.max_torque = 700
            actuator.goal_position = goal_pos
        net.synchronize()
    goal_pos -= 20


# ===================================================================================================================
#                                           Move the Servos (angular)
# ===================================================================================================================
print("Moving with angles")
D = USB2Dynamixel_Device(dev_name="/dev/tty.usbserial-AI03QD8V", baudrate=1000000)
s_list = find_servos(D)
s1 = Robotis_Servo(D,s_list[0])
s2 = Robotis_Servo(D,s_list[1])

# 6) Test out both servos to make sure your robot is working as expected

s1.move_angle(0.0); s2.move_angle(0.0)
s1.move_angle(0.5); s2.move_angle(-0.5)
s1.move_angle(-0.5); s2.move_angle(0.5)

# ===================================================================================================================
#                                           changing torque
# ===================================================================================================================

print("Moving for 10 seconds without torque...")
start = datetime.now()
goal_pos = central_position
while (abs(net.get_dynamixels()[0].current_position - goal_pos) > 5 and \
       abs(net.get_dynamixels()[1].current_position - goal_pos) > 5) and \
       (datetime.now()-start).seconds < 10:
    for actuator in net.get_dynamixels():
        actuator.moving_speed = 50
        actuator.torque_enable = 0
        actuator.torque_limit = 0
        actuator.max_torque = 700
        actuator.goal_position = goal_pos
    net.synchronize()

# ===================================================================================================================
#                                           changing compliance
# ===================================================================================================================

# here CW means "clock-wise" and CCW means "counter clock-wise"
print("moving 30 seconds with maxed compliance...")
start = datetime.now()
goal_pos = central_position
amount = 20
for actuator in net.get_dynamixels():
    actuator._set_ccw_compliance_slope(254)
    actuator._set_cw_compliance_slope(254)
    actuator._set_ccw_compliance_margin(0)
    actuator._set_cw_compliance_margin(1)
    print("cw angle limit           ", actuator.cw_angle_limit)
    print("ccw angle limit          ", actuator.ccw_angle_limit)
    print("cw compliance margin     ", actuator.cw_compliance_margin)
    print("ccw compliance margin    ", actuator.ccw_compliance_margin)
    print("cw compliance slope      ", actuator.cw_compliance_slope)
    print("ccw compliance slope     ", actuator.ccw_compliance_slope)

while(datetime.now()-start).seconds < 30:
    while (abs(net.get_dynamixels()[0].current_position - goal_pos) > 5 and \
          abs(net.get_dynamixels()[1].current_position - goal_pos) > 5):
        for actuator in net.get_dynamixels():
            actuator.moving_speed = 50
            actuator.torque_enable = 1
            actuator.torque_limit = 700
            actuator.max_torque = 700
            actuator.goal_position = goal_pos
        net.synchronize()
    goal_pos += amount
    if goal_pos > 700:
        amount = -20
    elif goal_pos < 300:
        amount = 20

print("wating for a minute with maxed compliance and disabled torque")
for actuator in net.get_dynamixels():
    actuator._set_torque_enable(0)
start = datetime.now()
while (datetime.now()-start).seconds <60:
    pass

# ===================================================================================================================
#                                           Minute or longer of control
# ===================================================================================================================

# resetting compliance
for actuator in net.get_dynamixels():
    actuator._set_ccw_compliance_slope(32)
    actuator._set_cw_compliance_slope(32)
    actuator._set_ccw_compliance_margin(1)
    actuator._set_cw_compliance_margin(0)

print("moving for 3 minutes...")
start = datetime.now()
amount = 100
goal_pos = central_position
while (datetime.now() - start).seconds / 60 < 3:
    if goal_pos > 750:
        amount = -100
    elif goal_pos < 250:
        amount = 100
    while abs(net.get_dynamixels()[0].current_position - goal_pos) > 5 and \
          abs(net.get_dynamixels()[1].current_position - goal_pos) > 5:
        for actuator in net.get_dynamixels():
            actuator.moving_speed = 50
            actuator.torque_enable = True
            actuator.torque_limit = 700
            actuator.max_torque = 700
            actuator.goal_position = goal_pos
        net.synchronize()
    goal_pos += amount
print("done")
# store the data
# real-time plotting

