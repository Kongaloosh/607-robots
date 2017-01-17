from pydynamixel import dynamixel, registers
#
# You'll need to change this to the serial port of your USB2Dynamixel
serial_port = '/dev/tty.usbserial-AI03QD8V'

# You'll need to change this to the ID of your servo
servo_id = 2

# # Turn the LED on
# led_value = dynamixel.registers.LED_STATE.ON
#
# try:
#     ser = dynamixel.get_serial_for_url(serial_port)
#     dynamixel.set_led(ser, servo_id, led_value)
#     print('LED set successfully!')
# except Exception as e:
#     print('Unable to set LED.')
#     print(e)


# If this is the first time the robot was powered on,
# you'll need to read and set the current position.
# (See the documentation above.)
target_position = 400                                  # (range: 0 to 1023)
first_move = True

try:
    ser = dynamixel.get_serial_for_url(serial_port)

    if first_move == True:
        dynamixel.init(ser, servo_id)
    while(True):
        target_position += 1
        if target_position > 700:
            break

        dynamixel.set_position(ser, servo_id, target_position)
        dynamixel.send_action_packet(ser)

    print('Success!')

except Exception as e:
    print('Unable to move to desired position.')
    print(e)


servo_id = 3

# # Turn the LED on
# led_value = dynamixel.registers.LED_STATE.ON
#
# try:
#     ser = dynamixel.get_serial_for_url(serial_port)
#     dynamixel.set_led(ser, servo_id, led_value)
#     print('LED set successfully!')
# except Exception as e:
#     print('Unable to set LED.')
#     print(e)


# If this is the first time the robot was powered on,
# you'll need to read and set the current position.
# (See the documentation above.)
target_position = 400                                  # (range: 0 to 1023)
first_move = True

try:
    ser = dynamixel.get_serial_for_url(serial_port)

    if first_move == True:
        dynamixel.init(ser, servo_id)
    while(True):
        target_position += 1
        if target_position > 700:
            break

        dynamixel.set_position(ser, servo_id, target_position)
        dynamixel.send_action_packet(ser)

    print('Success!')

except Exception as e:
    print('Unable to move to desired position.')
    print(e)
