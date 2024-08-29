#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from kady_custom_msg_package.msg import KadyCustomMsg  # should be Replaced with your actual message type
import RPi.GPIO as GPIO
import time

class Hover_Control:
    def __init__(self, tx_pin):
        self.tx_pin = tx_pin
        self.hex_value = 0
        self.bit_delay = 0.000037  # 37 microseconds in seconds
        self.stop_bit_delay = 0.000070  # 70 microseconds in seconds
        self.packet_delay = 0.000100  # 100 microseconds in seconds

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tx_pin, GPIO.OUT)

    def forward(self, value):
        if value == 0:
            self.hex_value = 0x000
        elif 0 < value <= 0.1:
            self.hex_value = 0x001
        elif 0.1 < value <= 0.2:
            self.hex_value = 0x002
        elif 0.2 < value <= 0.3:
            self.hex_value = 0x003
        elif 0.3 < value <= 0.4:
            self.hex_value = 0x004
        elif 0.4 < value <= 0.5:
            self.hex_value = 0x005
        elif 0.5 < value <= 0.6:
            self.hex_value = 0x006
        elif 0.6 < value <= 0.7:
            self.hex_value = 0x007
        elif 0.7 < value <= 0.8:
            self.hex_value = 0x008
        elif 0.8 < value <= 0.9:
            self.hex_value = 0x009
        elif 0.9 < value <= 1.0:
            self.hex_value = 0x00A
        else:
            self.hex_value = 0x00F  # Maximum value for overflow cases

        self.move(self.hex_value, direction="forward")

    def backward(self, value):
        if value <= 0:
            self.hex_value = 0x0FF
        elif 0 < value <= 0.1:
            self.hex_value = 0x0FE
        elif 0.1 < value <= 0.2:
            self.hex_value = 0x0FD
        elif 0.2 < value <= 0.3:
            self.hex_value = 0x0FC
        elif 0.3 < value <= 0.4:
            self.hex_value = 0x0FB
        elif 0.4 < value <= 0.5:
            self.hex_value = 0x0FA
        elif 0.5 < value <= 0.6:
            self.hex_value = 0x0F9
        elif 0.6 < value <= 0.7:
            self.hex_value = 0x0F8
        elif 0.7 < value <= 0.8:
            self.hex_value = 0x0F7
        elif 0.8 < value <= 0.9:
            self.hex_value = 0x0F6
        elif 0.9 < value <= 1.0:
            self.hex_value = 0x0F5
        else:
            self.hex_value = 0x0F0  # Minimum value for overflow cases

        self.move(self.hex_value, direction="backward")

    def move(self, hex_value, direction):
        start = time.time()
        while (time.time() - start) < 0.5:  # Move for 500 ms
            self.write_frame(0x100, self.bit_delay)
            if direction == "forward":
                self.write_frame(hex_value, self.bit_delay)
                self.write_frame(hex_value, self.bit_delay)
                self.write_frame(hex_value, self.bit_delay)
                self.write_frame(hex_value, self.bit_delay)
            else:
                self.write_frame(hex_value, self.bit_delay)
                self.write_frame(0x000, self.bit_delay)
                self.write_frame(hex_value, self.bit_delay)
                self.write_frame(0x000, self.bit_delay)
            time.sleep(self.packet_delay)
            self.write_frame(0x055, self.bit_delay)

    def write_frame(self, frame, delay):
        # Start bit
        start = time.time()
        GPIO.output(self.tx_pin, 0)
        time.sleep(max(0, delay - (time.time() - start)))

        # Send bits
        for _ in range(8):
            start = time.time()
            GPIO.output(self.tx_pin, frame & 1)
            frame >>= 1
            time.sleep(max(0, delay - (time.time() - start)))

        # Stop bit
        start = time.time()
        GPIO.output(self.tx_pin, 1)
        time.sleep(max(0, self.stop_bit_delay - (time.time() - start)))

    def cleanup(self):
        GPIO.cleanup()


class MotorControl:
    def __init__(self):
        # Initialize the node named 'speed_listener'
        rospy.init_node('speed_listener', anonymous=True)
        # Subscribe to the 'HAMADA' topic
        rospy.Subscriber("HAMADA", KadyCustomMsg, self.callback)  # should be Replaced with your actual message type
        rospy.loginfo("MotorControl node initialized and subscribed to 'HAMADA' topic")
        self.hover_1 = Hover_Control(15)
        self.hover_2 = Hover_Control(16)  # should be changed, but we have only one txd pin ?????

    def callback(self, data):
       
        speedFB = data.data
        speedRL = data.data               # This line should be adjusted according to our actual data structure.
        # Call another function with the extracted speeds 
        self.process_speeds(speedFB, speedRL)

    def process_speeds(self, speedFB, speedRL):
        print("Received speeds: speedFB = {}, speedRL = {}".format(speedFB, speedRL))

        if speedFB == 0 and speedRL == 0:
            # Stop motor
            self.hover_1.forward(0)
            self.hover_2.forward(0)
        elif speedRL == 0:
            # Move forward/backward
            if speedFB > 0:
                self.hover_1.forward(speedFB)
                self.hover_2.forward(speedFB)
            else:
                self.hover_1.backward(-speedFB)
                self.hover_2.backward(-speedFB)
        elif speedRL > 0:
            # Move clockwise
            self.hover_1.forward(speedRL)
            self.hover_2.backward(speedRL)
        elif speedRL < 0:
            # Move anticlockwise
            self.hover_1.backward(-speedRL)
            self.hover_2.forward(-speedRL)

    def start(self):
        # Keep the node running
        rospy.spin()

    def cleanup(self):
        self.hover_1.cleanup()
        self.hover_2.cleanup()


if __name__ == '__main__':
    try:
        # Create an instance of MotorControl
        motor_control = MotorControl()

        # Start the node
        motor_control.start()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Clean up GPIO settings
        motor_control.cleanup()
