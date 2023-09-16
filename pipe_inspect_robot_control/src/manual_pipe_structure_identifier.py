#!/usr/bin/env python3

#Imports
import rospy
import std_msgs
from joystick_class import Joystick

# 
class Pipe_Structure_Identifier():
    """
    the Pipe_Structure_Identifier class implements the manual pipe structure identifier. 
    Currently, pipe structures from this class are identified as integers. 

    Attributes:
        self.total_structures: Number of structures that can be identified by the user.
        self.pipe_structure_identification_pub: ROS publisher resposible for publishing the pipe identification value
        self.identified_pipe_structure: Current identified pipe structure.

        #These attributes are to prevent the user from rapidly changing the pipe structure by holding the buttons down
        self.select_next_pipe_structure_button_status: Holds the state from the previous button state
        self.select_previous_pipe_structure_button_status: Holds the state from the previous button state

    Methods: 
        publish_identified_pipe_struture(): Uses the status from the buttons to cycle through the list of integers from 1 to the self.total_structures. 
    
    """
    def __init__(self):
        self.total_structures = 2 
        self.pipe_structure_identification_pub = rospy.Publisher("/pipe_inspect_robot/current_pipe_identification", std_msgs.msg.Int8, queue_size=10)
        self.identified_pipe_structure = 0
        self.select_next_pipe_structure_button_status = 0
        self.select_previous_pipe_structure_button_status = 0

    # Selects and publishes a pipe structure based on the shoulder buttons of the joystick. 
    def publish_identified_pipe_struture(self, joystick):
        """
        Function cycles through the selected pipe types using the josytick buttons

        Input: 
            joystick: Joystick object used for reading the joystick buttins
        
        """
        if joystick.select_previous_pipe_structure == 1 and self.select_previous_pipe_structure_button_status == 0:
            if self.identified_pipe_structure == 0: 
                self.identified_pipe_structure = self.total_structures + 1
            self.identified_pipe_structure -= 1
        
        elif joystick.select_next_pipe_structure == 1 and self.select_next_pipe_structure_button_status == 0:
            if self.identified_pipe_structure == self.total_structures: 
                self.identified_pipe_structure = -1
            self.identified_pipe_structure += 1

        self.select_previous_pipe_structure_button_status = joystick.select_previous_pipe_structure
        self.select_next_pipe_structure_button_status = joystick.select_next_pipe_structure

        #Finally publish the identified pipe structure.
        self.pipe_structure_identification_pub.publish(self.identified_pipe_structure)
        

def main():
    #Create ros node.
    rospy.init_node("pipe_structure_identifier", anonymous=True)
    
    #Create pipe structure identifier object
    pipe_structure_identifier = Pipe_Structure_Identifier()
    
    #Create joystick object
    joystick = Joystick()

    # Publish the identified pipe structure in a loop
    while not rospy.is_shutdown():
        pipe_structure_identifier.publish_identified_pipe_struture(joystick)

        #20 Hz
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass