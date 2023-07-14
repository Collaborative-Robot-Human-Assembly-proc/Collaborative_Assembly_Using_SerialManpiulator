#!/usr/bin/env python3

import cv2

import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import rospy

from ultralytics import YOLO
import torch

from mpl_toolkits.mplot3d import Axes3D

import time


from sympy import symbols, Eq, solve
import subprocess
from sensor_msgs.msg import PointCloud2, PointField,Image,CameraInfo

import std_msgs.msg as std_msgs

import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header,Int16

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped





#set the frame id
camera_frame_id="safety_color_optical_frame"







    
    
    
    
    
    
    
    

 
class rs_camera:


	#####################Initiating camera pipline and profile and retrieving intrinsics######################
	def __init__(self):
                
		self.pipeline = rs.pipeline()
		self._cv_bridge = CvBridge()

		# Create a config and configure the pipeline to stream
		#  different resolutions of color and depth streams
		self.config = rs.config()

		# Get device product line for setting a supporting resolution
		self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
		self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
		self.device = self.pipeline_profile.get_device()
		

		found_rgb = False
		for s in self.device.sensors:
			if s.get_info(rs.camera_info.name) == 'RGB Camera':
				found_rgb = True
				break
		if not found_rgb:
			print("The demo requires  camera_info_msg.header = Header()Depth camera with Color sensor")
			exit(0)

		self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


		
		self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

		# Start streaming
		self.profile = self.pipeline.start(self.config)

		# Getting the depth sensor's depth scale (see rs-align example for explanation)
		self.depth_sensor = self.profile.get_device().first_depth_sensor()
		self.depth_scale = self.depth_sensor.get_depth_scale()
		self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
		self.depth_to_color_extrin =self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.color))
		self.color_to_depth_extrin =  self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.depth))
		self.depth_intrin= self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

		self.pose_flag=1

		
	####################################################################################################3	
	

        ################Prepare camera info for depth_to_pc.launch file######################
	def intiate_camera_info(self):
		self.camera_info_msg = CameraInfo()
		self.camera_info_msg.width = self.intr.width
		self.camera_info_msg.height = self.intr.height
		self.camera_info_msg.distortion_model = 'Inverse Brown Conrady'
		self.camera_info_msg.K = [self.intr.fx, 0.0, self.intr.ppx,
                         0.0, self.intr.fy, self.intr.ppy,
                         0.0, 0.0, 1.0]
		self.camera_info_msg.R = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
		self.camera_info_msg.P = [self.intr.fx, 0.0, self.intr.ppx, 0.0,
                         0.0, self.intr.fy, self.intr.ppy, 0.0,
                         0.0, 0.0, 1.0, 0.0]
		self.camera_info_msg.D=[0.0,0.0,0.0,0.0,0.0]
		self.camera_info_msg.header = Header()


	#camera_info_msg.header.stamp = rospy.Time.now()
		self.camera_info_msg.header.frame_id = camera_frame_id
		
	###############################################################################	
        
                
	
	########################choosing which sensor to align to################
	def align(self,dst="color"):
		if dst =="color":

			self.align_to = rs.stream.color
		else:
			self.align_to = rs.stream.depth	
		self.align = rs.align(self.align_to)
		return self.align
		
	###################################################################	#
		
		
		
	####################Returning aligned Frames#################3
		
	def get_frames(self):
		try:

        # Get frameset of color and depth
			self.frames = self.pipeline.wait_for_frames()	
			self.depth_frame_=self.frames.get_depth_frame()			
			# frames.get_depth_frame() is a 640x360 depth image				
			# Align the depth frame to color frame
			self.aligned_frames = self.align.process(self.frames)
			
			
			# Get aligned frames

			self.aligned_depth_frame = self.aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
			self.color_frame = self.aligned_frames.get_color_frame()
			self.depth_img=np.asanyarray(self.aligned_depth_frame.get_data())
			self.color_img=color_image = np.asanyarray(self.color_frame.get_data())
			return self.depth_img,self.color_img
		except:
				pass

	
	####################################################################
	
	
	
	######################Applying the mask to the depth Values#######################
	def pc(self,img,depth):


		
		depth_segmented=depth
		depth_segmented=np.where(img==0,0,depth_segmented)


		depth_segmented=(depth_segmented.astype(np.uint16))
		
		

		depth_segmented=self.convert_cv2_to_ros_msg(depth_segmented)

		depth_publisher.publish(depth_segmented)


		camera_info_publisher.publish(self.camera_info_msg)

	####################################################################################
	
		
		


	######################convert the depth image from a cv2 image to rosImage#############

	def convert_cv2_to_ros_msg(self, cv2_data, image_encoding='16UC1'):
		"""
		Convert from a cv2 image to a ROS Image message.
		"""
		return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)   
		
		
	############################################################################
	def cv2_to_ros(self,img,image_encoding='bgr8'):

		return self._cv_bridge.cv2_to_imgmsg(img, image_encoding)	
		
		
		
		
	##################Give your point cloud the desired frame id#####################
	
	
	def point_cloud_callback(self,msg):

		msg.header.frame_id = camera_frame_id

   		 # Publish the modified point cloud
    
		cloud_pub.publish(msg) 
		
		
	################################################################################
	
	################Sending XYZ of hand in space#####################################################

	def send_hand_cord(self,color_point):

        

		depth_point_ = rs.rs2_project_color_pixel_to_depth_pixel(self.depth_frame_.get_data(), self.depth_scale,2, -2,self.depth_intrin, self.intr, self.depth_to_color_extrin, self.color_to_depth_extrin,color_point)
		#print(depth_point_) 
  
		if depth_point_[0]<0 or depth_point_[1]<0: 
			print('camera is so near to object and depth cannot determine') 
		else:
			depth = self.depth_frame_.get_distance(round(depth_point_[0]) , round(depth_point_ [1]))
			dx,dy, dz = rs.rs2_deproject_pixel_to_point(self.depth_intrin, depth_point_ , depth) 
			hand_point=PoseStamped()
			hand_point.pose.position.x=dx
			hand_point.pose.position.y=dy
			hand_point.pose.position.z=dz
			return hand_point
       
	######################################################################################################	
	def set_pose_flag(self,flag):
		self.pose_flag=flag



	



	




class parts_seg:

 ##################intializing the two models#################
    def __init__(self):
        self.seg_model = YOLO("yolov8s-seg.pt")  # load a pretrained model
        self.pose_model = YOLO("yolov8s-pose.pt") 
        
 ######################running the segmentation model###############################       
    def predict_seg(self,color_image):
    
        try:
         self.results = self.seg_model.predict(color_image,save=False,show_labels=False,conf = 0.7,classes=[0,39])

         self.detection = self.results[0].plot()
         cv2.imshow('detect',self.detection)
         mask =self.results[0].masks.data.cpu().numpy()


         masked_img=mask[0,:,:]
         for i in range((mask.shape[0]-1)):    
          masked_img=np.logical_or(masked_img,mask[i+1,:,:])
					
         #print("aaa")
         self.s= (np.array(masked_img)*255).astype(np.uint8)
         #cv2.imshow("non_eroded_causes_noise",self.s)
         self.s = cv2.erode(self.s, kernel, iterations=1)
         #print(self.s)	 
	 
         #cv2.imshow("eroded removes error",self.s)
         
               
        except:
         pass
       
###########################running the pose estimation model############################################
    def predict_pose(self,img):
        try:
        
         self.pose_results = self.pose_model.predict(img,save=False,show_labels=False,conf = 0.5)
         self.pose_det= self.pose_results[0].plot()
         
         self.keypoints = (self.pose_results[0].keypoints)

        
         self.hand_points_canceller()

         

         cv2.imshow("mask1",self.s)
        except:
         pass
  
        

########################Canceling out hand points from the mask################################		
		

    def hand_points_canceller(self):
    
        #8 is right arm
        
        # 9 is left hand 
        #10 is right hand
            

        self.x_hand=int(self.keypoints.xy[0][9][0])
        self.y_hand=int(self.keypoints.xy[0][9][1])
        x_arm=int(self.keypoints.xy[0][7][0])
        y_arm=int(self.keypoints.xy[0][7][1])
        
        length=((self.x_hand-x_arm)**2+(y_arm-self.y_hand)**2)**0.5


        new_legnth=length+10
        

        slope=(y_arm-self.y_hand)/(x_arm-self.x_hand)

	##############solving for new point that is shifted 10 pixels above the hand##############33
        x, y = symbols('x y')        

        equation1 = Eq((y_arm-y)/(x_arm-x), slope)
        equation2 = Eq(((x-x_arm)**2+(y-y_arm)**2)**0.5,new_legnth)
        solution = solve((equation1, equation2), (x, y))
        
        #print("x,y solved")
        print(solution)# new x and y for a more hand centered point 
        
        #####################################################################################
        

	#################get the angle by which the hand is aligned ###############
        angle = np.arctan(slope) * 180 / np.pi
        #print(angle)
        if angle>0:
         x=solution[0][0]
         y=solution[0][1]
        else:
         angle=angle+360 
         x=solution[1][0]
         y=solution[1][1]         
        #############################################################################
        
        
         
        ##########################define box points around the centered point###########3333
        box_points = np.array([[x-100, y-100],[x+100, y-100] , [x+100, y+100], [x-100, y+100]], dtype=np.float32)                
        center = np.mean(box_points, axis=0)
        
        
        ##########get a rotation matrix by which the hand is rotated###########
        rotation_matrix = cv2.getRotationMatrix2D(tuple(center), -1*angle, 1)
	#################3apply this rotation to the box points that you got########33
        rotated_points = cv2.transform(np.array([box_points]), rotation_matrix)[0]
        #######################################################################################3
	
	
	
        rotated_points = rotated_points.astype(int)
        #print(rotated_points)
        color = (0, 0, 0)
        #cv2.putText(self.pose_det, "0", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imshow('pose',self.pose_det)
        #print(rotated_points)
       

        cv2.fillConvexPoly(self.s, rotated_points, color)
        #self.x_hand=int(x)
        #self.y_hand=int(y)
##################################################################################################        











	
	        
#######################create a camera class object and define necessary pubs and subs ######################################       



camera_1=rs_camera()    	
rospy.init_node('pose_estimator')
depth_publisher = rospy.Publisher('depth_pub', Image, queue_size=10)

camera_info_publisher=rospy.Publisher('rs_depth_info', CameraInfo, queue_size=10)

cloud_sub=rospy.Subscriber('/camera/depth/points', PointCloud2, camera_1.point_cloud_callback)

cloud_pub=rospy.Publisher('/camera/depth/points/with_frame', PointCloud2, queue_size=10)

pose_estimator=rospy.Publisher('pose_estimation', PoseStamped,queue_size=10)

pose__flag=rospy.Subscriber('pose_flag', Int16,camera_1.set_pose_flag)



rgb_pose_pub=rospy.Publisher('pose_result', Image, queue_size=10)
seg_mask_pub=rospy.Publisher('mask_pub', Image, queue_size=10)
seg_res=rospy.Publisher('seg_result', Image, queue_size=10)

	



#################################################################################







#####################make objects of the pose&segmentation class #############


seg=parts_seg()
camera_1.intiate_camera_info()
camera_1.align("color")


#########################################




###############Create a ros-cv bridge##################################




bridge = CvBridge()



###############################################################################

######################prepare a kernel of ur desire to erosion filter###############



kernel = np.ones((13, 13), np.uint8)


######################################################################################

while True:
    
    

	depth_img,img=camera_1.get_frames()


	try:
		seg.predict_seg(img)
		#print("start_pub")
		#seg_mask_pub.publish(bridge.cv2_to_imgmsg(seg.s, 'mono8'))
		#seg_res.publish(bridge.cv2_to_imgmsg(seg.detection, 'bgr8'))
		#print("pub done")

		
		if camera_1.pose_flag==1:
		
			seg.predict_pose(img)
			#rgb_pose_pub.publish(bridge.cv2_to_imgmsg(seg.pose_det, 'bgr8'))
			hand_point=camera_1.send_hand_cord([seg.x_hand,seg.y_hand])
			pose_estimator.publish(hand_point)
			
		#camera_1.pc(seg.s,depth_img)
		



	except:
		pass


	
	
	if cv2.waitKey(5) & 0xFF == 27:
		break

