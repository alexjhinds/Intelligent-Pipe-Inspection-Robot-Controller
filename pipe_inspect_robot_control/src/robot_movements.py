#!/usr/bin/env python3

#Imports
import rospy
import std_msgs
from joystick_class import Joystick

class Robot():
    """
    The class Robot contains methods and attributes related to controlling the physical or simulated robot. It interfaces closely with the 
    ROS_control framework and publishes corresponding control values to the different motors in the robot. 

    The class also contains methods for executing intelligent control strategies. The functions execute a specific action at the frequency 
    of the Node that uses and initalizes this class. 

    Attributes: 
        # ROS publishers responsible for publishing a control value to the respective motors. 
        # Note: on the physical robot, a single motor controls two wheels, however, in this program, there is an individual motor for each wheel. 
        # This is to remain flexible incase any design changes occur, but doesn't change the behavior of the current program
        self.W3_publisher
        self.W1_publisher
        self.W4_publisher
        self.W6_publisher
        self.W2_publisher
        self.W5_publisher
        self.EWH1_publisher
        self.EWH2_publisher
        self.SL_control_publisher
        
        # Velocity Limits for respective motors
        self.ew1_motor_max_rotational_velocity
        self.ew2_motor_max_rotational_velocity
        self.w_1_4_motor_max_rotational_velocity
        self.w_2_5_motor_max_rotational_velocity
        self.w_3_6_motor_max_rotational_velocity

        self.progress_through_structure: State variable used during the miter bend control strategy keeping track of the robots progress through the bend.

        #Specific motor commands to each set of wheels
        self.ew1_motor_rotational_velocity_command
        self.ew2_motor_rotational_velocity_command
        self.w_1_4_motor_rotational_velocity_command
        self.w_2_5_motor_rotational_velocity_command
        self.w_3_6_motor_rotational_velocity_command

    Methods: 
        move_through_standard_bend: Control Strategy for moving through a standard gradual bend
        move_through_mitered_bend: Control Strategy for moving through a miter bend
        avoid_obstacles: Control strategy for avoiding other pipe routes seen by the structured light sensor
        set_axial_robot_motor_speed: Setter function for the axial motor speed
        set_axial_robot_motor_speed_miter: Setter function for the axial motor speed in the miter bend control strategy
        set_rotate_robot_motor_speed: setter function for the rotational robot speed
        set_rotate_robot_motor_speed_miter: Setter function for the rotational robot speed in the miter bend control strategy
        auto_adjust_sl_sensor: Function which allows for control of the structure light sensor angle given a value
        control_sl_sensor: Allows for the control of the structured light sensor given inputs from the josytick. 
        rotate_wheel_1_4: Command to drive the forward 2 wheels
        rotate_wheel_2_5: Command to drive the middle 2 wheels
        rotate_wheel_3_6: Command to drive the back 2 wheels
        rotate_robot: Command to drive the end wheels
        rotate_robot_opposite: Command to drive the end wheels in the reverse direction
        move_axially: Command to drive all 3 sets of axial wheels
        stop_axial_motion: Command to stop axial motion
        stop_rotating: Command to stop rotational robot motion. 
        manual_control: Function to manually control the robot from the joystick. 
    """

    def __init__(self, 
                 ew1_motor_max_rotational_velocity, 
                 ew2_motor_max_rotational_velocity, 
                 w_1_4_motor_max_rotational_velocity, 
                 w_2_5_motor_max_rotational_velocity, 
                 w_3_6_motor_max_rotational_velocity):
        """
        Initalizer function for the robot class. 

        Inputs: 
            # Max rotational velocty for specific motor in radians/second
            ew1_motor_max_rotational_velocity: 
            ew2_motor_max_rotational_velocity: 
            w_1_4_motor_max_rotational_velocity: 
            w_2_5_motor_max_rotational_velocity: 
            w_3_6_motor_max_rotational_velocity: 
        
        """
        
        self.W3_publisher = rospy.Publisher("/pipe_inspect_robot/W3_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.W1_publisher = rospy.Publisher("/pipe_inspect_robot/W1_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.W4_publisher = rospy.Publisher("/pipe_inspect_robot/W4_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.W6_publisher = rospy.Publisher("/pipe_inspect_robot/W6_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.W2_publisher = rospy.Publisher("/pipe_inspect_robot/W5_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.W5_publisher = rospy.Publisher("/pipe_inspect_robot/W2_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.EWH1_publisher = rospy.Publisher("/pipe_inspect_robot/EWH1_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.EWH2_publisher = rospy.Publisher("/pipe_inspect_robot/EWH2_joint_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.SL_control_publisher = rospy.Publisher("/pipe_inspect_robot/SL_mount_controller/command", std_msgs.msg.Float64, queue_size=10)
        
        # Velocity Limits for motors
        self.ew1_motor_max_rotational_velocity = ew1_motor_max_rotational_velocity
        self.ew2_motor_max_rotational_velocity = ew2_motor_max_rotational_velocity
        self.w_1_4_motor_max_rotational_velocity = w_1_4_motor_max_rotational_velocity
        self.w_2_5_motor_max_rotational_velocity = w_2_5_motor_max_rotational_velocity
        self.w_3_6_motor_max_rotational_velocity = w_3_6_motor_max_rotational_velocity

        self.progress_through_structure = 0

        self.ew1_motor_rotational_velocity_command = 0
        self.ew2_motor_rotational_velocity_command = 0
        self.w_1_4_motor_rotational_velocity_command = 0
        self.w_2_5_motor_rotational_velocity_command = 0
        self.w_3_6_motor_rotational_velocity_command = 0

    def move_through_standard_bend(self, joystick : Joystick, pipe_options_vector):
        """
        Standard Bend control strategy. 

        Inputs: 
            joystick: Joystick class for comminucating with the joystick hardware
            pipe_options_vector: pipe directions options array containing the angles of the pipe bend
        
        Returns: 
            None
        """


        #Set the +- threshold for determining if robot is in the correct position
        threshold_deg = 10
        self.progress_through_structure = 0

        # Check if there is actual pipe directions detected by the sensor. If so, execute the command movements.
        if pipe_options_vector[0] != -2:

            # Set the set point angle to align with based on the pipe_options vector, 
            # accomidating if there are multiple direction options detected at this moment.
            pipe_angle_to_align_with = sum(pipe_options_vector)/len(pipe_options_vector)

            # Based on the camera frame, there are specific set points for the robot to align. 
            # For this standard bend this value is 270 degrees
            robot_heading_for_alingment_angle = 270

            # Calculate the error between the current orientation from the sensor and the set point
            # The if statement determines whether the robot needs to rotate clockwise or counterclockwise 
            # to reach the set point faster. It sets the error accordingly. 
            if pipe_angle_to_align_with > 90:
                robot_orientation_error = robot_heading_for_alingment_angle - pipe_angle_to_align_with
            else:
                robot_orientation_error = -(90 + pipe_angle_to_align_with)

            #Before commanding the robot, check if the robot is within the end condition. 
            # If so, only forward movement is necessary. 
            if (pipe_angle_to_align_with < (robot_heading_for_alingment_angle + threshold_deg)) and (pipe_angle_to_align_with > (robot_heading_for_alingment_angle - threshold_deg)):
                self.stop_rotating()
                self.move_axially()
            
            # If the robot is not in the end condition:
            else:
                # The rotation consists of two commands, the axial movement, and the rotating within the pipe. 
                # Calculate forward robot speed
                drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
                
                #Set forward speed to the motors
                self.set_axial_robot_motor_speed(drive_motor_rotational_speed)
                
                #Execute the movement
                self.move_axially()

                #Now for the rotational speed determine the movement with the proportional constant set to 0.2
                speed_factor =  robot_orientation_error* 0.2
                
                # Calculate the rotational end wheel speed based on the error
                end_wheel_rotate_speed = min([joystick.joystick_move_forward_command_percentage*speed_factor, self.ew1_motor_max_rotational_velocity])
                
                #Set the wheel velocities to this value
                self.set_rotate_robot_motor_speed(end_wheel_rotate_speed)
                
                #Execute the movement
                self.rotate_robot()
        else:
            #If the robot is still in a straight pipe, then only forward movement is needed

            # Calculate forward robot speed
            drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
            
            #Set forward speed to the motors
            self.set_axial_robot_motor_speed(drive_motor_rotational_speed)

            #Execute the movement
            self.move_axially()
            
    def move_through_mitered_bend(self, joystick, pipe_options_vector):
        """
        Miter Bend control strategy.

        Inputs: 
            joystick: Joystick class for comminucating with the joystick hardware
            pipe_options_vector: pipe directions options array containing the angles of the pipe bend
        
        Returns: 
            None
        """


         #Set the +- threshold for determining if robot is in the correct position
        threshold_deg = 10

        #There are 3 states in the miter bend the robot needs to get through, with different behavior in each

        # 1. Approaching the miter bend: (self.progress_through_structure = 0)
        # 2. Alligning with the Miter bend  (self.progress_through_structure = 1)
        # 3. Driving through the miter bend  (self.progress_through_structure = 2)

        if self.progress_through_structure == 0:  
            if pipe_options_vector[0] != -2:
                self.progress_through_structure = 1 # Meaning that it is in the first part of the bend

        elif self.progress_through_structure == 1: 
            if pipe_options_vector[0] == -2:
                self.progress_through_structure = 2 # Then it has just hit the end of the wall. 

     
        if self.progress_through_structure == 0:
            #If the robot is still in a straight pipe, then only forward movement is needed

            # Calculate forward robot speed
            drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
            
            #Set forward speed to the motors
            self.set_axial_robot_motor_speed(drive_motor_rotational_speed)

            #Execute the movement
            self.move_axially()
        
        elif self.progress_through_structure == 1: 
            # then the robot needs to rotate to allign with the miter bend.

             # Set the set point angle to align with based on the pipe_options vector, 
            # accomidating if there are multiple direction options detected at this moment.
            pipe_angle_to_align_with = sum(pipe_options_vector)/len(pipe_options_vector)

            # Based on the camera frame, there are specific set points for the robot to align. 
            # For this miter bend this value is 90 degrees
            robot_heading_for_alingment_angle = 90

            # Calculate the error between the current orientation from the sensor and the set point
            # The if statement determines whether the robot needs to rotate clockwise or counterclockwise 
            # to reach the set point faster. It sets the error accordingly. 
            if pipe_angle_to_align_with > 270:
                robot_orientation_error = pipe_angle_to_align_with - robot_heading_for_alingment_angle 
            else:
                robot_orientation_error = -(pipe_angle_to_align_with - robot_heading_for_alingment_angle)
           
            #Before commanding the robot, check if the robot is within the end condition. 
            # If so, only forward movement is necessary. 
            if (pipe_angle_to_align_with < (robot_heading_for_alingment_angle + threshold_deg)) and (pipe_angle_to_align_with > (robot_heading_for_alingment_angle - threshold_deg)):
                self.stop_rotating()
                self.move_axially()
            
            # If the robot is not in the end condition:
            else:
                # The rotation consists of two commands, the axial movement, and the rotating within the pipe. 
                # Calculate forward robot speed
                drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
                
                #Set forward speed to the motors
                self.set_axial_robot_motor_speed(drive_motor_rotational_speed)
                
                #Execute the movement
                self.move_axially()

                #Now for the rotational speed determine the movement with the proportional constant set to 0.2
                speed_factor =  robot_orientation_error* 0.4
                
                # Calculate the rotational end wheel speed based on the error
                end_wheel_rotate_speed = min([joystick.joystick_move_forward_command_percentage*speed_factor, self.ew1_motor_max_rotational_velocity])
                
                #Set the wheel velocities to this value
                self.set_rotate_robot_motor_speed(end_wheel_rotate_speed)
                
                #Execute the movement
                self.rotate_robot()

        if self.progress_through_structure == 2:
            #Robot is halfway though the bend, and needs to command the forward motor much more than back two motors. 

            # Calculate forward robot speed. A larger amount of speed is necessary to drive the front motors to pull the robot through. This multiplier is set here for demonstration purposes
            drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity* 2
            
            #Calculate the rotate speed
            end_wheel_rotate_speed = self.ew2_motor_max_rotational_velocity

            #Set forward speed to the motors
            self.set_axial_robot_motor_speed_miter(drive_motor_rotational_speed)
            
            #Set the wheels to rotate to assist the robot in staying at the right angle. 
            self.set_rotate_robot_motor_speed_miter(end_wheel_rotate_speed)
            
            #Execute the movement
            self.move_axially()
            self.rotate_robot()
        
    def avoid_obstacles(self, joystick, pipe_options_vector):
        """
        Avoiding obstacles control strategy.

        Inputs: 
            joystick: Joystick class for comminucating with the joystick hardware
            pipe_options_vector: pipe directions options array containing the angles of the pipe bend
        
        Returns: 
            None
        """

        #Set the +- threshold for determining if robot is in the correct position
        threshold_deg = 10
        self.progress_through_structure = 0

        # Check if there is actual pipe directions detected by the sensor. If so, execute the command movements.
        if pipe_options_vector[0] != -2:

            # If there are multiple pipe routes, then passing between them would be ideal
            # This means we should allign with the average of the two values. 
            # There are two locations on a circle where the robot could be alligned with 

            if len(pipe_options_vector) >= 2:
                pipe_angle_to_align_with_1 = sum(pipe_options_vector)/len(pipe_options_vector)
                if pipe_angle_to_align_with_1 < 180:
                    pipe_angle_to_align_with_2 = pipe_angle_to_align_with_1 + 180
                else: 
                    pipe_angle_to_align_with_2 = pipe_angle_to_align_with_1 - 180
            else:
                pipe_angle_to_align_with_1 = pipe_options_vector[0] + 90
                if pipe_angle_to_align_with_1 < 180:
                    pipe_angle_to_align_with_2 = pipe_angle_to_align_with_1 + 180
                else: 
                    pipe_angle_to_align_with_2 = pipe_angle_to_align_with_1 - 180
            
            # Based on the camera frame, there are specific set points for the robot to align. 
            # For avoiding obstacles, there are two set points for the robot to allign with and still
            # make it through the other pipe routes. 
            robot_heading_for_alingment_angle = 90

            if abs(pipe_angle_to_align_with_1 - robot_heading_for_alingment_angle) < 90:
                pipe_angle_to_align_with = pipe_angle_to_align_with_1
            else:
                pipe_angle_to_align_with = pipe_angle_to_align_with_2

            # Calculate the error between the current orientation from the sensor and the set point
            # The if statement determines whether the robot needs to rotate clockwise or counterclockwise 
            # to reach the set point faster. It sets the error accordingly. 
            if pipe_angle_to_align_with > 180:
                robot_orientation_error = robot_heading_for_alingment_angle - (360 - pipe_angle_to_align_with)
            else:
                robot_orientation_error = robot_heading_for_alingment_angle - (pipe_angle_to_align_with)
            #Before commanding the robot, check if the robot is within the end condition. 
            # If so, only forward movement is necessary. 
            if (pipe_angle_to_align_with < (robot_heading_for_alingment_angle + threshold_deg)) and (pipe_angle_to_align_with > (robot_heading_for_alingment_angle - threshold_deg)):
                self.stop_rotating()

                drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
                
                #Set forward speed to the motors
                self.set_axial_robot_motor_speed(drive_motor_rotational_speed)

                #Execute the movement
                self.move_axially()
            
            # If the robot is not in the end condition:
            else:
                # The rotation consists of two commands, the axial movement, and the rotating within the pipe. 
                # Calculate forward robot speed
                drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
                
                #Set forward speed to the motors
                self.set_axial_robot_motor_speed(drive_motor_rotational_speed)
                
                
                self.move_axially()

                #Now for the rotational speed determine the movement with the proportional constant set to 0.2
                speed_factor =  robot_orientation_error* 0.2
                
                # Calculate the rotational end wheel speed based on the error
                end_wheel_rotate_speed = min([joystick.joystick_move_forward_command_percentage*speed_factor, self.ew1_motor_max_rotational_velocity])
                
                #Set the wheel velocities to this value
                self.set_rotate_robot_motor_speed(end_wheel_rotate_speed)
                
                #Execute the movement
                self.rotate_robot()
        else:
            #If the robot is still in a straight pipe, then only forward movement is needed

            # Calculate forward robot speed
            drive_motor_rotational_speed = joystick.joystick_move_forward_command_percentage * self.w_1_4_motor_max_rotational_velocity
            
            #Set forward speed to the motors
            self.set_axial_robot_motor_speed(drive_motor_rotational_speed)

            #Execute the movement
            self.move_axially()

    def set_axial_robot_motor_speed(self, value):
        """
        Function to set the axial motor speed from a given value

        Inputs: 
            value: rotational speed in radians/second for the motors
        Returns: 
            None
        """
        self.w_1_4_motor_rotational_velocity_command = value
        self.w_2_5_motor_rotational_velocity_command = value
        self.w_3_6_motor_rotational_velocity_command = value
    
    def set_axial_robot_motor_speed_miter(self, value):
        """
        Function to set the axial motor speed from a given value for the miter bend case

        Inputs: 
            value: rotational speed in radians/second for the motors
        Returns: 
            None
        """
        self.w_1_4_motor_rotational_velocity_command = value
        self.w_2_5_motor_rotational_velocity_command = value * 0.6 #Drive the back and middle motors slower
        self.w_3_6_motor_rotational_velocity_command = value * 0.6

    def set_rotate_robot_motor_speed(self, value):
        """
        Function to set the desired rotational speed for the motors that rotate the robot

        Inputs: 
            value: rotational speed in radians/second for the motor
        Returns: 
            None
        """
        self.ew1_motor_rotational_velocity_command = value
        self.ew2_motor_rotational_velocity_command = value
    
    def set_rotate_robot_motor_speed_miter(self, value):
        """
        Function to set the desired rotational speed for the motors that rotate the robot in the miter case

        Inputs: 
            value: rotational speed in radians/second for the motor
        Returns: 
            None
        """
        self.ew1_motor_rotational_velocity_command = value
        self.ew2_motor_rotational_velocity_command = value

    def auto_adjust_sl_sensor(self, value): 
        """
        Function to adjust the structured light sensor angle from a given positional input

        Inputs: 
            value: rotatational position in radians for the sensor move to
        Returns: 
            None
        """
        self.SL_control_publisher.publish(value)

    def control_sl_sensor(self, joystick):
        """
        Function to adjust the structured light sensor angle from the joystick

        Inputs: 
            joystick: Joystick object allowing for the sensor button statuses to be accessed
        Returns: 
            None
        """
        if joystick.sensor_up:
            self.SL_control_publisher.publish(-0.4)
        elif joystick.sensor_down:
            self.SL_control_publisher.publish(0.4)
        else:
            self.SL_control_publisher.publish(0)

    def rotate_wheel_1_4(self):
        """
        Function to command the front two driver wheels

        Inputs: 
            None
        Returns: 
            None
        """
        self.W1_publisher.publish(-self.w_1_4_motor_rotational_velocity_command)
        self.W4_publisher.publish(-self.w_1_4_motor_rotational_velocity_command)

    def rotate_wheel_2_5(self):
        """
        Function to command the middle two driver wheels

        Inputs: 
            None
        Returns: 
            None
        """
        self.W2_publisher.publish(-self.w_2_5_motor_rotational_velocity_command)
        self.W5_publisher.publish(-self.w_2_5_motor_rotational_velocity_command)

    def rotate_wheel_3_6(self):
        """
        Function to command the back two driver wheels

        Inputs: 
            None
        Returns: 
            None
        """
        self.W3_publisher.publish(-self.w_3_6_motor_rotational_velocity_command)
        self.W6_publisher.publish(-self.w_3_6_motor_rotational_velocity_command)
        
    def rotate_robot(self):
        """
        Function to command both of the end wheels

        Inputs: 
            None
        Returns: 
            None
        """
        self.EWH1_publisher.publish(self.ew1_motor_rotational_velocity_command)
        self.EWH2_publisher.publish(-self.ew2_motor_rotational_velocity_command)

    def rotate_robot_opposite(self):
        """
        Function to command both of the end wheels opposite of eachother

        Inputs: 
            None
        Returns: 
            None
        """
        self.EWH1_publisher.publish(-self.ew1_motor_rotational_velocity_command)
        self.EWH2_publisher.publish(-self.ew2_motor_rotational_velocity_command)

    def move_axially(self):
        """
        Function to command the robot to drive all 3 sets of axial motors

        Inputs: 
            None
        Returns: 
            None
        """
        self.rotate_wheel_1_4()
        self.rotate_wheel_3_6()
        self.rotate_wheel_2_5()

    def stop_axial_motion(self):
        """
        Function to stop the axial motors

        Inputs: 
            None
        Returns: 
            None
        """
        self.W3_publisher.publish(0)
        self.W1_publisher.publish(0)
        self.W4_publisher.publish(0)
        self.W6_publisher.publish(0) 
        self.W2_publisher.publish(0)
        self.W5_publisher.publish(0) 

    def stop_rotating(self):
        """
        Function to stop the robot from rotating

        Inputs: 
            None
        Returns: 
            None
        """
        self.EWH1_publisher.publish(0)
        self.EWH2_publisher.publish(0)

    def manual_control(self, joystick):
        """
        Function to manual control the robot given the joystick inputs

        Inputs: 
            joystick: Joystick object allowing for the manual control button statuses to be accessed
        Returns: 
            None
        """
        if joystick.dpad_lr == 1: #rotate left
            self.set_rotate_robot_motor_speed(20)
            self.rotate_robot()

        elif joystick.dpad_lr == -1: #rotate right
            self.set_rotate_robot_motor_speed(-20)
            self.rotate_robot()

        if joystick.dpad_ud == 1: #move forward
            self.set_axial_robot_motor_speed(10)
            self.move_axially()

        elif joystick.dpad_ud == -1: #move backwards
            self.set_axial_robot_motor_speed(-10)
            self.move_axially()
