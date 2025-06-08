## SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
##
## SPDX-License-Identifier: MIT

"""CircuitPython Essentials Servo standard servo


given this circuitpython script used on esp32
modify it to read data from serial port and then control 6 servomotors
the serial protocol will be:
20,90,90,70,90,90,90,0;
each number followed by ','  in this case servo zero is set to 20 degree and servo 4 is set to 70 degree
if a servo don't need to be updated I will write x so if I want to change only the last servo I will write:
x,x,x,x,x,x,x,130;
servo_pins = [ board.GPIO39, board.GPIO38,board.GPIO45,board.GPIO21,board.GPIO16,board.GPIO18,board.GPIO8,board.GPIO47]  # Change to match your setup
echo "155,127,127,127,127,127,127,x;" > /dev/ttyACM0

90,90,90,90,90,90,90,20;

"""
import board
import pwmio
import adafruit_motor.servo
import busio
import time


def map_value(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Initialize serial communication
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.1)

# Servo setup




servo_pins = [ board.GPIO39, board.GPIO38,board.GPIO45,board.GPIO21,board.GPIO16,board.GPIO18,board.GPIO8,board.GPIO47]  # Change to match your setup
servo_with270_degree_indices = [0,3]  # list of the indices of the servos that have 270 degree capability inside servo_pins[], 3 --> means the 4th servo is a 270 degree servo
servo_with270_degree_pins = [board.GPIO39, board.GPIO21]  # Change to match your setup
servos = []

pwm0 = pwmio.PWMOut(board.GPIO39, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm0,min_pulse=500,max_pulse=2520))

pwm1 = pwmio.PWMOut(board.GPIO38, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm1,min_pulse=500,max_pulse=2430))

pwm2 = pwmio.PWMOut(board.GPIO45, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm2,min_pulse=500,max_pulse=2455))

pwm3 = pwmio.PWMOut(board.GPIO21, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm3,min_pulse=500,max_pulse=2565))

pwm4 = pwmio.PWMOut(board.GPIO16, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm4,min_pulse=500,max_pulse=2430))

pwm5 = pwmio.PWMOut(board.GPIO18, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm5,min_pulse=500,max_pulse=2468))

pwm6 = pwmio.PWMOut(board.GPIO8, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm6,min_pulse=500,max_pulse=2430))

pwm7 = pwmio.PWMOut(board.GPIO47, duty_cycle=0, frequency=50)
servos.append(adafruit_motor.servo.Servo(pwm7,min_pulse=500,max_pulse=2430))



# for pin in servo_pins:
#     pwm = pwmio.PWMOut(pin, duty_cycle=0, frequency=50)
#     servos.append(adafruit_motor.servo.Servo(pwm,min_pulse=500,max_pulse=2430))






# servos[1] = adafruit_motor.servo.Servo(pwm,min_pulse=470,max_pulse=2570)
# servos[2] = adafruit_motor.servo.Servo(pwm,min_pulse=470,max_pulse=2570)
# servos[3] = adafruit_motor.servo.Servo(pwm,min_pulse=470,max_pulse=2570)3300
# for pin in servo_pins:
#     print(pin)
#     pwm = pwmio.PWMOut(pin, duty_cycle=0, frequency=50)
#     #servos.append(adafruit_motor.servo.Servo(pwm))
#     if pin in servo_with270_degree_pins:   #270 degree servo
#         servos.append(adafruit_motor.servo.Servo(pwm,min_pulse=470,max_pulse=2570))
#     else:  #normal 0 180 degree servo
#         servos.append(adafruit_motor.servo.Servo(pwm,min_pulse=500,max_pulse=2500))

#         #servos.append(adafruit_motor.servo.Servo(pwm,min_pulse=480,max_pulse=2430))
#     #if the servo is 270 degree...use a different min_pulse and max_pulse
    
    
    
    
# print("end of pinlist")



def set_servo_smoothly(servo, target_angle, current_angle=None, step=5):
    """Move servo smoothly to target angle"""

    if target_angle == servo.angle:  #if the angle is already setted
        return

    if current_angle is None:
        current_angle = servo.angle if servo.angle is not None else 90

    
    while abs(current_angle - target_angle) > step:
        if current_angle < target_angle:
            current_angle += step
        else:
            current_angle -= step
        servo.angle = current_angle
        time.sleep(0.02
        )
    
    servo.angle = target_angle








def update_servos(data):
    """Parses the received data and updates servos accordingly."""
    if not data.endswith(';'):
        uart.write(b'error\n')
        return
    
    values = data[:-1].split(',')  # Remove trailing ';' before splitting
    if len(values) != len(servo_pins):
        uart.write(b'errorLL\n')
        return  # Ignore incorrect input length
    
    for i in range(len(servo_pins)):
        if values[i] != 'x':  # Update only if not 'x'
            try:
                value = int(values[i])
                angle = value #map_value(value, 0, 255, 0, 220)
                # print("servos[i]")
                # print(servos[i])
                # servos[i].angle = angle
                if i in servo_with270_degree_indices:  #to handle servos that can go to 270 degree
                    # print("270 degree servo detected")
                    angle = map_value(value, 0,270, 0, 180)
                    
                set_servo_smoothly(servos[i], angle,servos[i].angle)
            except ValueError:
                uart.write(b'error_for\n')
                return  # Ignore invalid values

    uart.write(b'ok!\n')  # Send confirmation message

while True:
    if uart.in_waiting:
        raw_data = uart.readline()
        #uart.write(b'wait\n')  # Send error message on decoding failure
        if raw_data:
            try:
                decoded_data = raw_data.decode('utf-8').strip()
                update_servos(decoded_data)
            except UnicodeError:
                uart.write(b'errorb try\n')  # Send error message on decoding failure
