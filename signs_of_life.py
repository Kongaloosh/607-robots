import dynamixel
import sys
from datetime import datetime
import numpy as np

__author__ = 'kongaloosh'

serial_port = '/dev/tty.usbserial-AI03QD8V'
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
else:
    print("servos found")
    sys.exit(0)

    
# move to original position

# central_position = 768                  # (range: 0 to 1023)
central_position = 512                  # (range: 0 to 1023)

# control encoder
for servo_id in servos:
    print(servo_id)
    position = central_position
    dynamixel.set_position(serial, servo_id, position)
    dynamixel.send_action_packet(serial)


# torque on an off
for i in servos:
    dynamixel.set_
    dynamixel.send_action_packet(serial)
    print(dynamixel.registers.MAX_TORQUE)

    position = central_position + 20
    dynamixel.set_position(serial, servo_id, position)
    dynamixel.send_action_packet(serial)

exit(0)

for i in servos:
    dynamixel.registers.TORQUE_ENABLE = True
    dynamixel.send_action_packet(serial)


# control trajectory of both motors (at once)
# trajectory for one minute or longer of control
start = datetime.now()
while (datetime.now() - start).seconds / 60 < 10:
    if position > 700:
        amount = -10
    else:
        amount = 10

initial_position = [400, 400]
chain.init(serial, servos, 180)
vector = chain.make_vector(initial_position+amount, [2,3], 100)
chain.move_to_vector(serial, vector)
chain.wait_for_move(serial, servos)


# store the data
# real-time plotting

