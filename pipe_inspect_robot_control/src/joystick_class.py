#!/usr/bin/env python3

#Imports
from sensor_msgs.msg import Joy
import rospy

class Joystick():
    """This class implements a basic ROS joystick and maps the joystick buttons to values that are called in the code. 
    Thus, the controls for the robot can be changed here rather than multiple changes throughout the code. 

    Attributes:
        joystick_sub: ROS Subscruber responsible for listening the Joy node to receive the controller inputs from hardware

        self.left_joystick_lr_percentage: Status from -1 to 1
        self.left_joystick_ud_percentage: Status from -1 to 1
        self.left_trigger
        self.right_trigger
        self.dpad_ud
        self.dpad_lr

        self.button_a: 1 is pressed, 0 is unpressed
        self.button_b: 1 is pressed, 0 is unpressed
        self.button_x: 1 is pressed, 0 is unpressed
        self.button_y: 1 is pressed, 0 is unpressed
        self.button_l: Left shoulder button
        self.button_r: Right shoulder button

        self.sensor_up: Rotates the structured light sensor up
        self.sensor_down:  Rotates the structured light sensor down
        self.manual_move_forward_command: Drives the robot in direction of the structured light sensor during manual control
        self.manual_rotate_command: Rotates the robot within the pipe during manual control
        self.select_next_pipe_structure: Selects the next pipe structure from the controller
        self.select_previous_pipe_structure: Selects the previous pipe structure from the controller
        self.joystick_move_forward_command_percentage: Drives the robot in direction of the structured light sensor during intelligent control
        self.joystick_rotate_command_percentage: Rotates the robot within the pipe during intelligent control

	Methods: 
		callback_joystick_values: Callback for recieving hardware inputs and assigning them to class attributes
    """

    def __init__(self):
        joystick_sub = rospy.Subscriber("/joy", Joy, self.callback_joystick_values)

        self.left_joystick_lr_percentage = 0 #lr means left right
        self.left_joystick_ud_percentage = 0 #ud means up down
        self.left_trigger = 0
        self.right_trigger = 0
        self.dpad_ud = 0
        self.dpad_lr = 0

        self.button_a = 0
        self.button_b = 0
        self.button_x = 0
        self.button_y = 0
        self.button_l = 0
        self.button_r = 0

        self.sensor_up = 0
        self.sensor_down = 0
        self.manual_move_forward_command = 0
        self.manual_rotate_command = 0
        self.select_next_pipe_structure = 0 
        self.select_previous_pipe_structure = 0
        self.joystick_move_forward_command_percentage = 0
        self.joystick_rotate_command_percentage = 0
        
    def callback_joystick_values(self, msg): 
        """
        This callback recieves the message from the joystick and maps the values to joystick variables.
        """
        self.left_joystick_lr_percentage = msg.axes[0] #lr means left right
        self.left_joystick_ud_percentage = msg.axes[1] #ud means up down
        self.right_joystick_lr_percentage = msg.axes[3]
        self.right_joystick_ud_percentage = msg.axes[4]
        self.left_trigger = msg.axes[2]
        self.right_trigger = msg.axes[5]
        self.dpad_ud = msg.axes[7]
        self.dpad_lr = msg.axes[6]

        self.button_a = msg.buttons[0]
        self.button_b = msg.buttons[1]
        self.button_x = msg.buttons[2]
        self.button_y = msg.buttons[3]
        self.button_l = msg.buttons[4]
        self.button_r = msg.buttons[5]

        #Here you can change the control mapping for the based on your robot. 
        self.sensor_up = self.button_y
        self.sensor_down = self.button_a
        self.manual_move_forward_backward_command = self.dpad_ud
        self.manual_rotate_command = self.dpad_lr
        self.select_next_pipe_structure = self.button_r
        self.select_previous_pipe_structure = self.button_l
        self.joystick_move_forward_command_percentage = self.left_joystick_ud_percentage 
        self.joystick_rotate_command_percentage = self.right_joystick_lr_percentage