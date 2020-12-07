#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math


class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.twist = Twist()

		# ============================================================================
		# EDIT THESE VALUES AS NECESSARY:
		# ============================================================================
		# Red Mask
		self.lower_red = np.array([  0, 220,  50]) 
		self.upper_red = np.array([  0, 255, 255])
		
		self.lower_yellow = np.array([  20, 100,  100]) 
		self.upper_yellow = np.array([  30, 255, 255])

		self.goal_radius = 85		# [pixels]
		self.goal_area   = 255 * math.pi * self.goal_radius ** 2		# [sum of pixel values]
		
		self.trackMethod = 'moments'		# Choose 'contours', 'moments', or 'Hough'.  'moments' is the default.
		
		self.maxSpeed = .5		# [m/s].  Maximum speed of our robot.
		# ============================================================================

		
		# Initialize our rotation direction.  -1 --> CW, 1 --> CCW
		self.rotationDir = -1
		self.calcRadius = 0
		self.turn = 0
	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')	
		
		h, w, d = image.shape
		
		# Define the range (rows) for our mask:
		# FIXME:  You'll probably want to adjust this.
		search_top = 0 
		search_bot = h
				
		(xr, yr) = (None, None)
		(xy1, yy1) = (None, None)
		(xy2, yy2) = (None, None)
		area = None
		radius = None
		
		if (self.trackMethod == 'contours'):
			# OPTION 1 -- USE CONTOURS.
			# See https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
			
			# resize the frame, blur it, and convert it to the HSV
			# color space.
			# frame = imutils.resize(frame, width=600)
			blurred = cv2.GaussianBlur(image, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		 
			# construct a mask for the color "red", then perform
			# a series of dilations and erosions to remove any small
			# blobs left in the mask
			red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
			red_mask[0:search_top, 0:w] = 0
			red_mask[search_bot:h, 0:w] = 0		
			red_mask = cv2.erode(red_mask, None, iterations=2)
			red_mask = cv2.dilate(red_mask, None, iterations=2)			
		
			# find contours in the mask and initialize the current
			# (x, y) center of the ball
			cntrs = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			
			# cntrs = cntrs[0] if imutils.is_cv2() else cntrs[1]
			cntrs = cntrs[1]			
						
			# only proceed if at least one contour was found
			if len(cntrs) > 0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				c = max(cntrs, key=cv2.contourArea)
				((cx, cy), r) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		 
				# only proceed if the radius meets a minimum size
				if r > 10:
					# draw the circle and centroid on the frame
					cv2.circle(image, (int(cx), int(cy)), int(r),
						(0, 255, 255), 2)
					cv2.circle(image, center, 5, (0, 0, 255), -1)
		
					radius = int(r)
					x = int(cx)
					y = int(cy)
					
			# Move the robot.
			# NOTE:  This method uses the RADIUS of the circle.
			self.moveMe(x, w, self.goal_radius, radius)
		
		
		elif (self.trackMethod == 'Hough'):
			# OPTION 2 -- USE HOUGH CIRCLES
			# See https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/

			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
			red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
			red_mask[0:search_top, 0:w] = 0
			red_mask[search_bot:h, 0:w] = 0		

			# detect circles in the image
			circles = cv2.HoughCircles(red_mask, cv2.HOUGH_GRADIENT, 4.5, 10)	# 40
			# print circles
			
			# Initialize a variable for the maximum radius we've found:
			max_r = 0

			# ensure at least some circles were found
			if circles is not None:
				# convert the (x, y) coordinates and radius of the circles to integers
				circles = np.round(circles[0, :]).astype("int")
			 
				# loop over the (x, y) coordinates and radius of the circles
				for (cx, cy, r) in circles:
					# draw the circle in the output image, then draw a rectangle
					# corresponding to the center of the circle
					cv2.circle(image, (cx, cy), r, (200, 255, 200), 2)
					cv2.rectangle(image, (cx - 5, cy - 5), (cx + 5, cy + 5), (0, 128, 255), -1)
					
					# We're going to keep track of center points of the biggest circle.
					# FIXME -- NOT SURE THAT THIS IS THE BEST IDEA.
					if (r > max_r):
						x = int(cx)
						y = int(cy)
						max_r = int(r)
				

			# Move the robot.
			# NOTE:  This method uses the RADIUS of the circle.
			self.moveMe(x, w, self.goal_radius, max_r)
			

		else:
			# OPTION 3 (default) -- USE MOMENTS

			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
			red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
			# left mask = 1, right mask = 2
			yellow_mask_left = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
			yellow_mask_right = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
			yellow_mask_left[0:h, w/2:w] = 0
			yellow_mask_right[0:h, 0:w/2] = 0
	
			M = cv2.moments(red_mask)
			if M['m00'] > 0:
				xr = int(M['m10']/M['m00'])
				yr = int(M['m01']/M['m00'])
				
				calcRadius   = int(math.sqrt(M['m00'] / (255 * math.pi)))
				self.calcRadius = calcRadius
				#print(calcRadius)
				cv2.circle(image, (xr, yr), calcRadius, (200, 255, 200), 2)
				cv2.rectangle(image, (xr - 5, yr - 5), (xr + 5, yr + 5), (0, 128, 255), -1)
	
				area = int(M['m00'])
				
			M_left = cv2.moments(yellow_mask_left)
			if M_left['m00'] > 0:
				xy1 = int(M_left['m10']/M_left['m00'])
				yy1 = int(M_left['m01']/M_left['m00'])
				cv2.circle(image, (xy1, yy1), 15, (100, 255, 100), 2)
			M_right = cv2.moments(yellow_mask_right)
			if M_right['m00'] > 0:
				xy2 = int(M_right['m10']/M_right['m00'])
				yy2 = int(M_right['m01']/M_right['m00'])
				cv2.circle(image, (xy2, yy2), 15, (100, 255, 100), 2)	
				
			# Move the robot.
			# NOTE:  This method uses the AREA of the circle.
			self.moveMe(xr, w, self.goal_area, area, xy1, xy2)
	
		
		if (xr is not None):
			# Print a blue circle on the image, showing the goal radius
			cv2.circle(image, (xr, yr), int(self.goal_radius), (255, 0, 0), 2)
			
		cv2.imshow("window", image)
		cv2.waitKey(3)


	def moveMe(self, xr, w, goal, observed, xy1, xy2):
	
		if (xr == None) and self.turn == 0: 
			# We lost sight of the ball.
	
			# Stop linear movement / Start rotating.
			self.twist.linear.x = 0.0 
			self.twist.angular.z = self.rotationDir * math.pi / 8	# FIXME -- Scaling by Pi/8 is arbitrary.
			self.twist.linear.y = 0
			print('finding ball')
		
						
		else:
			# Move towards ball until it is close enough (50 pixels radius)
			if self.calcRadius < 80 and self.turn == 0:
				# We found the ball.
				print('moving to ball')
				# Rotation
				errRotate = xr - w/2
				thetaRad = math.atan(errRotate/1024.0)		# FIXME -- Dividing by 1024 is arbitrary
				self.twist.angular.z = -thetaRad
				self.twist.linear.y = 0
				# Keep track of our direction of rotation.
				# We'll use this direction if we lose sight of the ball.
				if (thetaRad > 0):
					self.rotationDir = -1
				else:
					self.rotationDir =  1
				
				# Set our linear speed and direction
				# NOTE:  The "goal" value could be an AREA or a RADIUS.
				#        The "observed" value should be in the same units.
				errLinear = abs(goal - observed)
				rateLinear = errLinear / float(goal)
				self.twist.linear.x = self.maxSpeed * rateLinear 
				
				#print errRotate, rateLinear
				
			# When close enough, stops and rotates until ball is lined up with goal	
			else:
				
				if (xr is not None and xy1 is not None and xy2 is not None) or self.turn > 1:
					if self.turn == 0:
						# checking if center of ball and center of goal are lined up
						#if abs(xr-xy) < 15:
						if xr > xy1+40 and xr < xy2-40:
							self.twist.linear.x = 0
							self.twist.angular.z = 0
							self.twist.linear.y = 0
							print('kicking ball')
							self.turn = 1
					# until ball and goal are lined up then move forward to kick
					elif self.turn < 100:
						if xr is not None:
							print('following ball')
							errRotate = xr - w/2
							thetaRad = math.atan(errRotate/1024.0)		# FIXME -- Dividing by 1024 is arbitrary
							self.twist.angular.z = -thetaRad
							self.twist.linear.y = 0									
							self.twist.linear.x = self.maxSpeed
	
							self.turn += 1
						else:
							print('moving forward')
							self.twist.angular.z = 0
							self.twist.linear.y = 0									
							self.twist.linear.x = self.maxSpeed
							self.turn +=1
					else:
						#self.turn = 0
						print('done kicking ball')
						self.twist.linear.x = 0
						self.twist.angular.z = 0
						self.twist.linear.y = 0
				if self.turn == 0:
					error = w - xr
					thetaRad = math.atan(error/1024.0)
					if (error < w/3): #self.turn < 10:
						
						self.twist.linear.x = 0	
						self.twist.linear.y = 0
						self.twist.angular.z = -thetaRad/2

					elif (error > 2*w/3) :
						self.twist.angular.z = thetaRad/2
						self.twist.linear.x = 0	
						self.twist.linear.y = 0
					else:
						self.twist.linear.y = 0.2
						self.twist.linear.x = 0	
						self.twist.angular.z = 0
	
		# Publish the command
		self.cmd_vel_pub.publish(self.twist)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
