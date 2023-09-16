#!/usr/bin/env python3

#Imports
import rospy
import std_msgs
from sensor_msgs.msg import JointState
from joystick_class import Joystick
from robot_movements import Robot


def main():
    """
    This joystick_teleop node is responsible for allowing the operator to use the joystick to manually control the robot, without using the intelligent 
    control methods. This node is disabled by default.
    """
    rospy.init_node("joystick_teleop", anonymous=True)

    #Create joystick object
    physical_controller = Joystick()

    #Define Robot Parameters for Joystick Control
    ew1_motor_max_rotational_velocity = 10 #rad/s
    ew2_motor_max_rotational_velocity = 10 #rad/s
    w_1_4_motor_max_rotational_velocity = 10 #rad/s
    w_2_5_motor_max_rotational_velocity = 10 #rad/s
    w_3_6_motor_max_rotational_velocity = 10 #rad/s

    #Create robot object, responsible for carrying out the appropriate movemnts.
    robot = Robot(ew1_motor_max_rotational_velocity,
                  ew2_motor_max_rotational_velocity,
                  w_1_4_motor_max_rotational_velocity,
                  w_2_5_motor_max_rotational_velocity,
                  w_3_6_motor_max_rotational_velocity)

    while not rospy.is_shutdown():
        #Continuously update the motor commands based on josytick readings. These values can be adjusted here. 
        robot.ew1_motor_rotational_velocity_command   = robot.ew1_motor_max_rotational_velocity   * physical_controller.joystick_rotate_command_percentage
        robot.ew2_motor_rotational_velocity_command   = robot.ew2_motor_max_rotational_velocity   * physical_controller.joystick_rotate_command_percentage
        robot.w_1_4_motor_rotational_velocity_command = robot.w_1_4_motor_max_rotational_velocity * physical_controller.joystick_move_forward_command_percentage
        robot.w_2_5_motor_rotational_velocity_command = robot.w_2_5_motor_max_rotational_velocity * physical_controller.joystick_move_forward_command_percentage
        robot.w_3_6_motor_rotational_velocity_command = robot.w_3_6_motor_max_rotational_velocity * physical_controller.joystick_move_forward_command_percentage


        #Move and respond to movements. controlling the robot involves moving forward and controlling the sensor, but other functions can be used
        robot.move_axially()
        robot.rotate_robot()
        robot.control_sl_sensor(physical_controller)

        #Run node 20Hz
        rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass