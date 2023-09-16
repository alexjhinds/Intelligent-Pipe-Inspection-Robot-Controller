#!/usr/bin/env python3

#Main Intelligent Robot Controller: Manual Control

#Imports
import rospy
import std_msgs
from pipe_inspect_robot_gazebo.msg import pipe_direction_options
from joystick_class import Joystick
from robot_movements import Robot


class Intelligent_Robot_Controller():
    """
    This class implements the controller for the robot. 
    
    The main behavior of the controller is to utilize the information from the structured light sensor and user input,
    and then act based on that information to navigate through the pipe. Specifically, the function checks the current pipe 
    environment, and executes a specific motor command based on the state of the simulated depth sensor and the current orientation
    of the robot. It then publishes a specific motor velocity command for the robot at the current point 
    It calculates the desired motor velocity 20 times per second, as denoted by the rospy.sleep(0.05)  


    Attributes: 
        self.joystick: Joystick object for utilizing the controller inputs
        self.robot: Robot object responslible for executing physical robot actions
        
        self.identified_pipe_structure: Currently selected pipe structure. 
        self.allow_manual_control: Boolean variable for whether manual control should be enabled at this time step
        self.pipe_direction_array: current pipe direction array, containing possible pipe routes in view from the sensor
        self.pipe_direction_options_sub: ROS Subscriber responsible for subscribing to teh pipe direction options array from the Simulated Sensor node
        self.pipe_identification_sub: ROS Subscriber responsible for listening to the current pipe identification from the user. 
    
    Methods: 
		pipe_direction_callback(): Callback function for receiving and assigning the class attribute the pipe direction array from the subscriber
		pipe_identification_callback(): Callback function for assigning class attribute identified_pipe_strutured the data from the subscriber
        control_robot():

    """
    def __init__(self, joystick : Joystick, robot : Robot):
        """
        Initialzer function for Intelligent Controller Class

        Input: 
            joystick: Josytick object for mapping hardware controls to variables
            robot: A Robot object which holds functions for physcally manipulating the simulated or real robot. 
        
        """

        self.joystick = joystick
        self.robot = robot
        
        self.identified_pipe_structure = 0
        self.allow_manual_control = 0
        self.pipe_direction_array = []
        self.pipe_direction_options_sub = rospy.Subscriber("/pipe_inspect_robot/pipe_direction_options",pipe_direction_options, self.pipe_direction_callback)
        self.pipe_identification_sub = rospy.Subscriber("/pipe_inspect_robot/current_pipe_identification",std_msgs.msg.Int8, self.pipe_identification_callback)
        
    def pipe_identification_callback(self, msg):
        self.identified_pipe_structure = msg.data

    def pipe_direction_callback(self, msg_data):
        self.pipe_direction_array = msg_data.pipe_direction_options_msg_vector

    def control_robot(self):
        """
        Functions carrys out desired action given current sensor information. 

        Inputs: 
            None
        
        Returns:
            None
        """
        if self.identified_pipe_structure == 0: #Straight pipe
            if self.joystick.joystick_move_forward_command_percentage != 0:
                self.robot.avoid_obstacles(self.joystick, self.pipe_direction_array)
            else:
                self.robot.stop_axial_motion()
                self.robot.stop_rotating()

            #Allow for any manual control. This overrides the abvoid obstacle functionality
            self.robot.manual_control(self.joystick)           


        if self.identified_pipe_structure == 1: #Simple gradual bend
            if self.joystick.joystick_move_forward_command_percentage != 0:
                self.robot.move_through_standard_bend(self.joystick, self.pipe_direction_array)
            else:
                self.robot.stop_axial_motion()
                self.robot.stop_rotating()


        if self.identified_pipe_structure == 2: #Mitered Joint
            if self.joystick.joystick_move_forward_command_percentage != 0:
                self.robot.move_through_mitered_bend(self.joystick, self.pipe_direction_array)
            else:
                self.robot.stop_axial_motion()
                self.robot.stop_rotating()
        
        #Always allow ability to control the sensor movements.
        self.robot.control_sl_sensor(self.joystick)
        
def main():
    #Create main robot control node
    rospy.init_node("main_robot_controller", anonymous=True)  

    #Create the joystick object
    physical_controller = Joystick()

    #Define Robot Parameters for Joystick Control
    ew1_motor_max_rotational_velocity = 10 #rad/s
    ew2_motor_max_rotational_velocity = 10 #rad/s
    w_1_4_motor_max_rotational_velocity = 10 #rad/s
    w_2_5_motor_max_rotational_velocity = 10 #rad/s
    w_3_6_motor_max_rotational_velocity = 10 #rad/s

    #Create robot object
    robot = Robot(ew1_motor_max_rotational_velocity,
                  ew2_motor_max_rotational_velocity,
                  w_1_4_motor_max_rotational_velocity,
                  w_2_5_motor_max_rotational_velocity,
                  w_3_6_motor_max_rotational_velocity)

    #Create intelligent robot controller object
    intelligent_robot = Intelligent_Robot_Controller(physical_controller, robot)
    
    while not rospy.is_shutdown():
        #Continuously call control robot function to determine best action at current time
        intelligent_robot.control_robot()

        # 20Hz freq.
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass