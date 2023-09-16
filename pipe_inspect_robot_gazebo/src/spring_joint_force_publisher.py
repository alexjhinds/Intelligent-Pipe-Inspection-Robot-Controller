#!/usr/bin/env python3

#Imports
import rospy
import std_msgs
from sensor_msgs.msg import JointState
import math

class SpringJoint:
    """This class implements simulated spring joints for the robot platform

    Attributes:
        spring_k_val: N/mm | Spring Constant
        spring_at_rest_Length: mm   | Spring length
        self.joint_number:  Represents which if the joints it refers to, the MP2, MP3, or MP4 joint is being controlled. 
        self.joint_angle: rad  | Represents the actual joint angle of the robot
        self.spring_length: mm   | Represents the length of the spring at the current moment
        self.torque: N*m  | Represents the torque due to the spring on the joint. Will be published to Gazebo

	Methods: 
		callback_recieving_joint_angle: Callback for recieving joint angles for the main body elements of the robot
        calculate_and_publish_force: Function to determine and publish the proper spring joint force
    """
    spring_k_val = 0.21015      
    spring_at_rest_Length = 72.6    
    
    

    def __init__(self, joint_number):
        self.joint_number = joint_number   
        self.joint_angle = 0               
        self.spring_length = 0             
        self.torque = 0                 
        
        topic_text = "/pipe_inspect_robot/MP{joint_number:}_joint_controller/command"
        topic_name = topic_text.format(joint_number = self.joint_number)
        self.joint_torque_publisher = rospy.Publisher(topic_name, std_msgs.msg.Float64, queue_size=10)

        rospy.Subscriber("/pipe_inspect_robot/joint_states", JointState, self.callback_recieving_joint_angle)
        
    def callback_recieving_joint_angle(self, msg):
        # The index of the joint coincidentally happens to match with name of the respective joint. Ex. MP2 is at index 2. 
        # This may change if other links are added to the robot model.
        position_index = self.joint_number
        self.joint_angle = msg.position[position_index]

    def calculate_and_publish_force(self):
        """
        Function calculates and publishes the spring force 

        Parameters:
            None
        Returns:
            None
        """
        #Determine spring length using Pythagorean Theorem
        self.spring_length = math.sqrt(6808.25-5402*math.cos(3.14-self.joint_angle))

        #Determine the spring force using Hooke's Law 
        spring_force = (self.spring_k_val*(self.spring_at_rest_Length - self.spring_length))

        #Determine the angle made between spring and one of the robot links
        angle_2 = math.acos((5476+math.pow(self.spring_length,2)-1332.25)/(2*5476*math.pow(self.spring_length,2)))

        #Determine torque using the force, angle, and moment arm
        self.torque = spring_force * math.sin(angle_2)*.074 

        #Publish torque
        self.joint_torque_publisher.publish(self.torque)


def main():
    #Create node
    rospy.init_node("spring_joint_force_publisher", anonymous=True)
    
    #Create a spring joint object for each spring joint on the robot
    M2_spring_joint = SpringJoint(2)
    M3_spring_joint = SpringJoint(3)
    M4_spring_joint = SpringJoint(4)

    #Calculate the spring force and publish that to the simulation 
    while not rospy.is_shutdown():
        M2_spring_joint.calculate_and_publish_force()
        M3_spring_joint.calculate_and_publish_force()
        M4_spring_joint.calculate_and_publish_force()
        rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass