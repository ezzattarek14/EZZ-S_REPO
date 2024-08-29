#NOTE the logic is based on two values for each motor one for forward speed and the other one for backward

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

import RPi.GPIO as GPIO          
from time import sleep

from gpiozero import Servo
from time import sleep

dc_1 = 24
dc_2 = 23
en = 25
servo_pin = 18

# Initialize pigpio
servo = Servo(18)

# Define the GPIO pin where the control wire is connected


GPIO.setmode(GPIO.BCM)
GPIO.setup(dc_1,GPIO.OUT)
GPIO.setup(dc_2,GPIO.OUT)
GPIO.output(dc_1,GPIO.LOW)
GPIO.output(dc_2,GPIO.LOW)
GPIO.setup(en,GPIO.OUT)
p=GPIO.PWM(en,1000)

def DC_Motor_Output(DC_F,DC_B) :
   if DC_F>0 :
       GPIO.output(dc_1,GPIO.HIGH)
       GPIO.output(dc_2,GPIO.LOW)
   elif DC_B :
       GPIO.output(dc_1,GPIO.LOW)
       GPIO.output(dc_2,GPIO.HIGH)
   else :
       GPIO.output(dc_1,GPIO.LOW)
       GPIO.output(dc_2,GPIO.LOW)

def Servo_Motor_Output(SERVO_F,SERVO_B) :
   if SERVO_F>0 :
       servo.max()  # Maximum forward rotation
   elif SERVO_B :
     servo.min()  # Maximum reverse rotation
   else:
        servo.mid()  # Stop rotation


def callback(data):
    # Assuming the data array contains values in the order: DC_F, DC_B, SERVO_F, SERVO_B
    if len(data.data) == 4:
        DC_F = data.data[0]
        DC_B = data.data[1]
        SERVO_F = data.data[2]
        SERVO_B = data.data[3]
        DC_Motor_Output(DC_F,DC_B) 
        Servo_Motor_Output(SERVO_F,SERVO_B)
        
        rospy.loginfo("Received data: DC_F=%f, DC_B=%f, SERVO_F=%f, SERVO_B=%f",
                      DC_F, DC_B, SERVO_F, SERVO_B)
    else:
        rospy.logwarn("Received unexpected data length: %d", len(data.data))

def listener():
    rospy.init_node('motor_servo_listener', anonymous=True)
    rospy.Subscriber('HAMADA', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
