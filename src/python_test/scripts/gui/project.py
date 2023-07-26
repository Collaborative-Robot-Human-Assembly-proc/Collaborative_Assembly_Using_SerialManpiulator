#!/usr/bin/env python3

import tkinter as tk
from PIL import Image, ImageTk
import rospy
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Char
import subprocess 

rospy.init_node('gui', anonymous=True)
pub = rospy.Publisher('data', Char, queue_size=10)

class RobotGUI:
    def __init__(self, master):
        self.master = master
        master.title("Robot Control")
        master.geometry("500x500")
        master.resizable(False, False)
        master.config(bg="white")

        # Create the start and stop buttons
        self.start_button = tk.Button(master, text="Start", command=self.start_robot, width=10, bg="green", fg="black", font=("Helvetica", 16, "bold"), bd=0, pady=10)
        self.stop_button = tk.Button(master, text="Stop", command=self.stop_robot, width=10, bg="red", fg="black", font=("Helvetica", 16, "bold"), bd=0, pady=10)
        
        """
        self.speech_recognition = tk.Button(master, text = "Speech Recognition", command=self.speech_recognition, width=10, bg="yellow", fg="black", font=("Helvetica", 16, "bold"), bd=0, pady=10)
        """
        
        # Create the robot image
        self.robot_image1 = Image.open("robot_image1.png")
        self.robot_image1 = self.robot_image1.resize((300, 300), Image.ANTIALIAS) # resize image to fit window
        self.robot_photo1 = ImageTk.PhotoImage(self.robot_image1)

        self.robot_image2 = Image.open("robot_image2.png")
        self.robot_image2 = self.robot_image2.resize((300, 300), Image.ANTIALIAS) # resize image to fit window
        self.robot_photo2 = ImageTk.PhotoImage(self.robot_image2)

        self.robot_label = tk.Label(master, image=self.robot_photo1, bg="white")


        # Create the title label
        self.title_label = tk.Label(master, text="Robot Control", font=("Helvetica", 20, "bold"), bg="white")

 	# Create the microphone image button
        self.mic_image = Image.open("mic_button.png")
        self.mic_image = self.mic_image.resize((70, 70), Image.ANTIALIAS) # resize image to fit button
        self.mic_photo = ImageTk.PhotoImage(self.mic_image)
        self.mic_button = tk.Button(master, image=self.mic_photo, bg="white", bd=0, command=self.activate_mic)

        # Pack the widgets
        self.title_label.pack(pady=20)
        self.robot_label.pack(pady=20)
        self.start_button.place(x=0, y=440)
        self.stop_button.place(x=350, y=440)
        self.mic_button.place(x=210, y=420)
        """
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/rgbd_camera/rgb/image_raw', ImageMsg, self.image_callback)
        """
        self.bridge = CvBridge()
        self.camera_window = None
        self.camera_label = None
        
    def start_robot(self):	
        self.title_label.config(text="Robot Started", fg="green")
        self.robot_label.config(image=self.robot_photo2)
        pub.publish(0)
        self.master.withdraw()  # Hide the main GUI window

        # Create a new GUI window for camera view
        self.camera_window = tk.Toplevel()
        self.camera_window.title("Camera View")
        self.camera_window.geometry("680x400")
        self.camera_window.config(bg="white")

        # Create the menu bar
        menu_bar = tk.Menu(self.camera_window)
        self.camera_window.config(menu=menu_bar)

        # Create the "File" menu
        file_menu = tk.Menu(menu_bar, tearoff=0)
        menu_bar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Back", command=self.back_to_main)

        # Create a label to display the camera feed
        self.camera_label = tk.Label(self.camera_window)
        self.camera_label.pack()

        self.image_subscriber = rospy.Subscriber('/rgbd_camera/rgb/image_raw', ImageMsg, self.image_callback)

        
        """
        subprocess.run("rosservice call /rws/set_motors_on \"{}\"", shell=True)
        subprocess.run("rosservice call /rws/pp_to_main \"{}\"",shell = True)
        subprocess.run("rosservice call /rws/start_rapid \"{}\"", shell = True)
        subprocess.run("rosrun egm_config egm_config.py", shell = True)	
        subprocess.run("rosservice call /rws/sm_addin/start_egm_joint \"{}\"", shell = True)
    	"""
    def stop_robot(self):
        self.title_label.config(text="Robot Stopped", fg="red")
        self.robot_label.config(image=self.robot_photo1)
        pub.publish(1)
        self.master.deiconify()  # Show the main GUI window

        if self.camera_window:
            # Close the camera window
            self.camera_window.destroy()

        """
        rosservice call /rws/sm_addin/stop_egm "{}"
	rosservice call /rws/stop_rapid "{}"
	rosservice call /rws/set_motors_off "{}" 
	"""
    def activate_mic(self):
        print("Microphone activated")
        pub.publish(2)

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Convert OpenCV image to PIL format
        pil_image = Image.fromarray(cv_image)

        # Resize the image to fit the label
        pil_image = pil_image.resize((480, 360), Image.ANTIALIAS)

        # Convert PIL image to Tkinter format
        tk_image = ImageTk.PhotoImage(pil_image)

        
        # Update the label with the new image
        self.camera_label.configure(image=tk_image)
        self.camera_label.image = tk_image   
        
    def back_to_main(self):
        if self.camera_window:
            self.camera_window.destroy()
            self.camera_window = None
        self.master.deiconify()  # Show the main GUI window

root = tk.Tk()
gui = RobotGUI(root)
root.mainloop()
