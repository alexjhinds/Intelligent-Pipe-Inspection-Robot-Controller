#!/usr/bin/env python3

#Imports
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import std_msgs
import math
from pipe_inspect_robot_gazebo.msg import pipe_direction_options

class Gui_Overlay():
	"""The GU_Overlay class is responsible for creating a graphical user interface (GUI) showing the operator the 
	current pipe direction, the selected pipe option, and whether manual control is enabled or disabled. 

	It uses the cv_bridge library to take an camera image from the structured light sensor and turn it into the image 
	used by OpenCV. Then, basic operations are done on the image to add text and direction line.

	The output ROS image message is used by RViz to display the current information for the user.

    Attributes:
		self.bridge: ROS OpenCV bridge object used to convert a ROS image message to an OpenCV image 
		self.pipe_direction_array: current pipe direction array, containing possible pipe routes in view from the sensor
		self.identified_pipe_structure: Currently selected pipe structure. 
		self.image_sub: ROS Subscriber responsible for subscribing to the camera image frame from the sensor
		self.pipe_direction_options_sub: ROS Subscriber responsible for subscribing to teh pipe direction options array from the Simulated Sensor node
		self.pipe_identification_sub: ROS Subscriber responsible for listening to the current pipe identification from the user.
		self.gui_image_publisher: ROS Publisher responsible for sending the completed GUI image as a ROS message.
		self.camera_frame_opencv: Initial OpenCV image object

	Methods: 
		draw_direction_line(): Draws a line pointing in the angular directions specified in the pipe direction array.
		pipe_direction_callback(): Callback function for receiving and assigning the class attribute the pipe direction array from the subscriber
		pipe_identification_callback(): Callback function for assigning class attribute identified_pipe_strutured the data from the subscriber
		camera_frame_callback(): Callback function for assigning the received camera image to the class attribute. 
		create_and_publish_gui_image_frame(): function to draw lines and write text on image and publish it as a image message
    """

	def __init__(self, bridge):
		"""
		Initalizer function for Gui_Overlay class. 

		Inputs: 
			bridge: A CvBridge object.
		"""
		self.bridge = bridge
		self.pipe_direction_array = []
		self.identified_pipe_structure = 0
		self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.camera_frame_callback)
		self.pipe_direction_options_sub = rospy.Subscriber("/pipe_inspect_robot/pipe_direction_options",pipe_direction_options, self.pipe_direction_callback)
		self.pipe_identification_sub = rospy.Subscriber("/pipe_inspect_robot/current_pipe_identification",std_msgs.msg.Int8, self.pipe_identification_callback) 
		self.gui_image_publisher = rospy.Publisher("gui_image",Image,queue_size=10)
		self.camera_frame_opencv = [] #empty image


	def draw_direction_line(self, angle_to_draw_line):
		"""
		This function draws a line on a Cv2 frame given a specific angle for the line to be drawn at. 
		Uses current reference to image object. 

		Inputs: 
			angle_to_draw_line: a angle in degrees denoting the desired angle to draw the line. Measured from the +x axis. 

		Returns: 
			None
		"""
		if angle_to_draw_line == -2: #-2 in the pipe direction options array means there are no direction options to draw a line for
			return
		center_x = 250
		center_y = 250
		dy = math.sin(angle_to_draw_line*math.pi/180) * 100
		dx = math.cos(angle_to_draw_line*math.pi/180) * 100
		cv2.line(self.camera_frame_opencv, (center_x, center_y), (center_x + int(dx), center_y + int(dy)), color=1, thickness=3) 

	def pipe_direction_callback(self, msg):
		self.pipe_direction_array = msg.pipe_direction_options_msg_vector

	def pipe_identification_callback(self, msg):
		self.identified_pipe_structure = msg.data

	def camera_frame_callback(self, msg):
		#Try to convert image
		try:
			self.camera_frame_opencv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		#Create the gui image frame and publish as soon as we recieve an image from the camera
		self.create_and_publish_gui_image_frame()


	def create_and_publish_gui_image_frame(self):
		"""
		This function creates the gui image by placing text and the line on the frame and publishing the resulting image
		Uses current objects image frame

		Inputs: 
			None
		
		Returns: 
			None
		"""

		# Initialze text to be placed on image
		if self.identified_pipe_structure == 0:
			pipe_structure_type_text = "Straight Pipe"
			manual_control_status_text = "Manual Control Enabled"
		elif self.identified_pipe_structure == 1:
			pipe_structure_type_text = "Standard Bend"
			manual_control_status_text = "Manual Control Disabled"
		elif self.identified_pipe_structure == 2:
			pipe_structure_type_text = "Miter Bend"
			manual_control_status_text = "Manual Control Disabled"

		# Place the identified pipe structure type on the frame
		self.camera_frame_opencv = cv2.putText(
								img = self.camera_frame_opencv,
								text = pipe_structure_type_text,
								org = (10, 30),
								fontFace = cv2.FONT_HERSHEY_DUPLEX,
								fontScale = 1,
								color = (0, 0, 255),
								thickness = 2)

		#place manual control status text on the frame
		self.camera_frame_opencv = cv2.putText(
								img = self.camera_frame_opencv,
								text = str(manual_control_status_text),
								org = (10,60),
								fontFace = cv2.FONT_HERSHEY_DUPLEX,
								fontScale = 1,
								color = (0, 0, 255),
								thickness = 2)
		
		#Draw lines on frame
		for option in range(0,len(self.pipe_direction_array)):
			self.draw_direction_line(self.pipe_direction_array[option])

		#Publish frame if possible
		try:
			self.gui_image_publisher.publish(self.bridge.cv2_to_imgmsg(self.camera_frame_opencv, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main():
	#Create the node
	rospy.init_node("create_gui_camera_overlay", anonymous=True)
	
	#Create the Cv2 bridge
	bridge = CvBridge()

	#Create the overlay object, which instantiates the ROS publishers and subscribers and allows them to work as the node runs.
	gui_overlay = Gui_Overlay(bridge)
	
	#Update at 20 Hz freq.
	while not rospy.is_shutdown():
		rospy.sleep(0.05)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
