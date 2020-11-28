#!/usr/bin/env python
import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Basic:
	def __init__(self):

		self.target_robot = int(sys.argv[1])

		self.robot_position_x = np.empty(8, dtype=np.float32)
		self.robot_position_y = np.empty(8, dtype=np.float32)
		self.robot_orientation_z = np.empty(8, dtype=np.float32)
		self.robot_linear_vel_x = np.empty(8, dtype=np.float32)
		self.robot_linear_vel_y = np.empty(8, dtype=np.float32)
		self.robot_angular_vel_z = np.empty(8, dtype=np.float32)
		self.new_robot_position_x = np.empty(8, dtype=np.float32)
		self.new_robot_position_y = np.empty(8, dtype=np.float32)
		self.new_robot_orientation_z = np.empty(8, dtype=np.float32)

		self.initial_position_x = [-1,-0.5,0,-1,-0.5,0,-1,-0.5,0]
		self.initial_position_y = [1,1,1,0.5,0.5,0.5,0,0,0]

		self.robot1_move = rospy.Publisher('/robot1/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot2_move = rospy.Publisher('/robot2/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot3_move = rospy.Publisher('/robot3/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot4_move = rospy.Publisher('/robot4/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot5_move = rospy.Publisher('/robot5/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot6_move = rospy.Publisher('/robot6/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot7_move = rospy.Publisher('/robot7/cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.robot8_move = rospy.Publisher('/robot8/cmd_vel_mux/input/teleop', Twist, queue_size=1)

		self.robot1_odom = rospy.Subscriber("/robot1/odom", Odometry, self.robot1_odom_callback)
		self.robot2_odom = rospy.Subscriber("/robot2/odom", Odometry, self.robot2_odom_callback)
		self.robot3_odom = rospy.Subscriber("/robot3/odom", Odometry, self.robot3_odom_callback)
		self.robot4_odom = rospy.Subscriber("/robot4/odom", Odometry, self.robot4_odom_callback)
		self.robot5_odom = rospy.Subscriber("/robot5/odom", Odometry, self.robot5_odom_callback)
		self.robot6_odom = rospy.Subscriber("/robot6/odom", Odometry, self.robot6_odom_callback)
		self.robot7_odom = rospy.Subscriber("/robot7/odom", Odometry, self.robot7_odom_callback)
		self.robot8_odom = rospy.Subscriber("/robot8/odom", Odometry, self.robot8_odom_callback)
		
		self.linear_vel = Twist()
		self.linear_vel.linear.x = 0.5
		self.linear_neg_vel = Twist()
		self.linear_neg_vel.linear.x = -0.5
		self.left_turn = Twist()
		self.left_turn.angular.z = 0.5
		self.right_turn = Twist()
		self.right_turn.angular.z = -0.5
		self.stationary = Twist()

		self.board = np.array([[3,6,0],[2,5,8],[1,4,7]])
		self.commands=[]
		
		self.rate = rospy.Rate(10)
		self.rate.sleep()

		self.solve()
		self.move()

		print("Goal Reached!!!")
		print("Press Ctrl^C to Exit")

	def solve(self):
		
		# Phase-1
		index = zip(*np.where(self.board == 0))
		escort_index = index[0]

		index = zip(*np.where(self.board == self.target_robot))
		target_index = index[0]

		vertical_steps = target_index[0]-escort_index[0]
		horizontal_steps = escort_index[1]-target_index[1]

		for i in range(vertical_steps):
			self.escort_down()

		for i in range(horizontal_steps):
			self.escort_left()

		horizontal_steps-=1

		# Phase-2
		while (vertical_steps,horizontal_steps)!=(0,0):
			
			index = zip(*np.where(self.board == 0))
			escort_index = index[0]

			index = zip(*np.where(self.board == self.target_robot))
			target_index = index[0]

			if escort_index[0]==target_index[0]:
				if vertical_steps!=0:
					self.escort_up()
					self.escort_right()
					self.escort_down()
					vertical_steps-=1
				else:
					self.escort_down()
					self.escort_right()
					self.escort_right()
					self.escort_up()
					self.escort_left()
					horizontal_steps-=1

			elif escort_index[1]==target_index[1]:
				if horizontal_steps!=0:
					self.escort_right()
					self.escort_up()
					self.escort_left()
					horizontal_steps-=1
				else:
					self.escort_left()
					self.escort_up()
					self.escort_up()
					self.escort_right()
					self.escort_down()
					vertical_steps-=1

	def escort_down(self):
		index = zip(*np.where(self.board == 0))
		p = index[0]
		self.board[p[0]][p[1]],self.board[p[0]+1][p[1]]=self.board[p[0]+1][p[1]],self.board[p[0]][p[1]]
		robot_number = self.board[p[0]][p[1]]
		shelf_up = 1
		self.commands.append([robot_number,shelf_up])

	def escort_left(self):
		index = zip(*np.where(self.board == 0))
		p = index[0]
		self.board[p[0]][p[1]],self.board[p[0]][p[1]-1]=self.board[p[0]][p[1]-1],self.board[p[0]][p[1]]
		robot_number = self.board[p[0]][p[1]]
		shelf_right = 2
		self.commands.append([robot_number,shelf_right])

	def escort_up(self):
		index = zip(*np.where(self.board == 0))
		p = index[0]
		self.board[p[0]][p[1]],self.board[p[0]-1][p[1]]=self.board[p[0]-1][p[1]],self.board[p[0]][p[1]]
		robot_number = self.board[p[0]][p[1]]
		shelf_down = 3
		self.commands.append([robot_number,shelf_down])

	def escort_right(self):
		index = zip(*np.where(self.board == 0))
		p = index[0]
		self.board[p[0]][p[1]],self.board[p[0]][p[1]+1]=self.board[p[0]][p[1]+1],self.board[p[0]][p[1]]
		robot_number = self.board[p[0]][p[1]]
		shelf_left = 4
		self.commands.append([robot_number,shelf_left])


	def move(self):

		for x in range(len(self.commands)):
			M=self.commands[x]
			self.robot_number=M[0]
			i=M[1]
			if i==1:
				self.forward_move()
			elif i==2:
				self.right_move()
			elif i==3:
				self.backward_move()
			elif i==4:
				self.left_move()
		self.forward_move()
		self.forward_move()

	def forward_move(self):

		self.new_robot_position_x[self.robot_number-1] = self.robot_position_x[self.robot_number-1]+0.5
		while self.new_robot_position_x[self.robot_number-1] > self.robot_position_x[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.linear_vel)
			elif self.robot_number==2:
				self.robot2_move.publish(self.linear_vel)
			elif self.robot_number==3:
				self.robot3_move.publish(self.linear_vel)
			elif self.robot_number==4:
				self.robot4_move.publish(self.linear_vel)
			elif self.robot_number==5:
				self.robot5_move.publish(self.linear_vel)
			elif self.robot_number==6:
				self.robot6_move.publish(self.linear_vel)
			elif self.robot_number==7:
				self.robot7_move.publish(self.linear_vel)
			elif self.robot_number==8:
				self.robot8_move.publish(self.linear_vel)
			self.rate.sleep()

		self.robot1_move.publish(self.stationary)
		self.robot2_move.publish(self.stationary)
		self.robot3_move.publish(self.stationary)
		self.robot4_move.publish(self.stationary)
		self.robot5_move.publish(self.stationary)
		self.robot6_move.publish(self.stationary)
		self.robot7_move.publish(self.stationary)
		self.robot8_move.publish(self.stationary)

	def backward_move(self):

		self.new_robot_position_x[self.robot_number-1] = self.robot_position_x[self.robot_number-1]-0.5
		while self.new_robot_position_x[self.robot_number-1] < self.robot_position_x[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.linear_neg_vel)
			elif self.robot_number==2:
				self.robot2_move.publish(self.linear_neg_vel)
			elif self.robot_number==3:
				self.robot3_move.publish(self.linear_neg_vel)
			elif self.robot_number==4:
				self.robot4_move.publish(self.linear_neg_vel)
			elif self.robot_number==5:
				self.robot5_move.publish(self.linear_neg_vel)
			elif self.robot_number==6:
				self.robot6_move.publish(self.linear_neg_vel)
			elif self.robot_number==7:
				self.robot7_move.publish(self.linear_neg_vel)
			elif self.robot_number==8:
				self.robot8_move.publish(self.linear_neg_vel)
			self.rate.sleep()

		self.robot1_move.publish(self.stationary)
		self.robot2_move.publish(self.stationary)
		self.robot3_move.publish(self.stationary)
		self.robot4_move.publish(self.stationary)
		self.robot5_move.publish(self.stationary)
		self.robot6_move.publish(self.stationary)
		self.robot7_move.publish(self.stationary)
		self.robot8_move.publish(self.stationary)

	def right_move(self):

		self.new_robot_orientation_z[self.robot_number-1] = -math.pi/2
		while self.new_robot_orientation_z[self.robot_number-1] < self.robot_orientation_z[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.right_turn)
			elif self.robot_number==2:
				self.robot2_move.publish(self.right_turn)
			elif self.robot_number==3:
				self.robot3_move.publish(self.right_turn)
			elif self.robot_number==4:
				self.robot4_move.publish(self.right_turn)
			elif self.robot_number==5:
				self.robot5_move.publish(self.right_turn)
			elif self.robot_number==6:
				self.robot6_move.publish(self.right_turn)
			elif self.robot_number==7:
				self.robot7_move.publish(self.right_turn)
			elif self.robot_number==8:
				self.robot8_move.publish(self.right_turn)
			self.rate.sleep()

		self.new_robot_position_y[self.robot_number-1] = self.robot_position_y[self.robot_number-1]-0.5
		while self.new_robot_position_y[self.robot_number-1] < self.robot_position_y[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.linear_vel)
			elif self.robot_number==2:
				self.robot2_move.publish(self.linear_vel)
			elif self.robot_number==3:
				self.robot3_move.publish(self.linear_vel)
			elif self.robot_number==4:
				self.robot4_move.publish(self.linear_vel)
			elif self.robot_number==5:
				self.robot5_move.publish(self.linear_vel)
			elif self.robot_number==6:
				self.robot6_move.publish(self.linear_vel)
			elif self.robot_number==7:
				self.robot7_move.publish(self.linear_vel)
			elif self.robot_number==8:
				self.robot8_move.publish(self.linear_vel)
			self.rate.sleep()

		self.new_robot_orientation_z[self.robot_number-1] = 0
		while self.new_robot_orientation_z[self.robot_number-1] > self.robot_orientation_z[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.left_turn)
			elif self.robot_number==2:
				self.robot2_move.publish(self.left_turn)
			elif self.robot_number==3:
				self.robot3_move.publish(self.left_turn)
			elif self.robot_number==4:
				self.robot4_move.publish(self.left_turn)
			elif self.robot_number==5:
				self.robot5_move.publish(self.left_turn)
			elif self.robot_number==6:
				self.robot6_move.publish(self.left_turn)
			elif self.robot_number==7:
				self.robot7_move.publish(self.left_turn)
			elif self.robot_number==8:
				self.robot8_move.publish(self.left_turn)
			self.rate.sleep()
		
		self.robot1_move.publish(self.stationary)
		self.robot2_move.publish(self.stationary)
		self.robot3_move.publish(self.stationary)
		self.robot4_move.publish(self.stationary)
		self.robot5_move.publish(self.stationary)
		self.robot6_move.publish(self.stationary)
		self.robot7_move.publish(self.stationary)
		self.robot8_move.publish(self.stationary)

	def left_move(self):

		self.new_robot_orientation_z[self.robot_number-1] = math.pi/2
		while self.new_robot_orientation_z[self.robot_number-1] > self.robot_orientation_z[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.left_turn)
			elif self.robot_number==2:
				self.robot2_move.publish(self.left_turn)
			elif self.robot_number==3:
				self.robot3_move.publish(self.left_turn)
			elif self.robot_number==4:
				self.robot4_move.publish(self.left_turn)
			elif self.robot_number==5:
				self.robot5_move.publish(self.left_turn)
			elif self.robot_number==6:
				self.robot6_move.publish(self.left_turn)
			elif self.robot_number==7:
				self.robot7_move.publish(self.left_turn)
			elif self.robot_number==8:
				self.robot8_move.publish(self.left_turn)
			self.rate.sleep()

		self.new_robot_position_y[self.robot_number-1] = self.robot_position_y[self.robot_number-1]+0.5
		while self.new_robot_position_y[self.robot_number-1] > self.robot_position_y[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.linear_vel)
			elif self.robot_number==2:
				self.robot2_move.publish(self.linear_vel)
			elif self.robot_number==3:
				self.robot3_move.publish(self.linear_vel)
			elif self.robot_number==4:
				self.robot4_move.publish(self.linear_vel)
			elif self.robot_number==5:
				self.robot5_move.publish(self.linear_vel)
			elif self.robot_number==6:
				self.robot6_move.publish(self.linear_vel)
			elif self.robot_number==7:
				self.robot7_move.publish(self.linear_vel)
			elif self.robot_number==8:
				self.robot8_move.publish(self.linear_vel)
			self.rate.sleep()

		self.new_robot_orientation_z[self.robot_number-1] = 0
		while self.new_robot_orientation_z[self.robot_number-1] < self.robot_orientation_z[self.robot_number-1]:
			if self.robot_number==1:
				self.robot1_move.publish(self.right_turn)
			elif self.robot_number==2:
				self.robot2_move.publish(self.right_turn)
			elif self.robot_number==3:
				self.robot3_move.publish(self.right_turn)
			elif self.robot_number==4:
				self.robot4_move.publish(self.right_turn)
			elif self.robot_number==5:
				self.robot5_move.publish(self.right_turn)
			elif self.robot_number==6:
				self.robot6_move.publish(self.right_turn)
			elif self.robot_number==7:
				self.robot7_move.publish(self.right_turn)
			elif self.robot_number==8:
				self.robot8_move.publish(self.right_turn)
			self.rate.sleep()	
		
		self.robot1_move.publish(self.stationary)
		self.robot2_move.publish(self.stationary)
		self.robot3_move.publish(self.stationary)
		self.robot4_move.publish(self.stationary)
		self.robot5_move.publish(self.stationary)
		self.robot6_move.publish(self.stationary)
		self.robot7_move.publish(self.stationary)
		self.robot8_move.publish(self.stationary)
		

	def robot1_odom_callback(self, msg):
		self.robot_position_x[0] = msg.pose.pose.position.x + self.initial_position_x[0]
		self.robot_position_y[0] = msg.pose.pose.position.y + self.initial_position_y[0]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[0] = yaw
		self.robot_linear_vel_x[0] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[0] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[0] = msg.twist.twist.angular.z

	def robot2_odom_callback(self, msg):
		self.robot_position_x[1] = msg.pose.pose.position.x + self.initial_position_x[1]
		self.robot_position_y[1] = msg.pose.pose.position.y + self.initial_position_y[1]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[1] = yaw
		self.robot_linear_vel_x[1] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[1] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[1] = msg.twist.twist.angular.z

	def robot3_odom_callback(self, msg):
		self.robot_position_x[2] = msg.pose.pose.position.x + self.initial_position_x[2]
		self.robot_position_y[2] = msg.pose.pose.position.y + self.initial_position_y[2]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[2] = yaw
		self.robot_linear_vel_x[2] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[2] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[2] = msg.twist.twist.angular.z

	def robot4_odom_callback(self, msg):
		self.robot_position_x[3] = msg.pose.pose.position.x + self.initial_position_x[3]
		self.robot_position_y[3] = msg.pose.pose.position.y + self.initial_position_y[3]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[3] = yaw
		self.robot_linear_vel_x[3] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[3] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[3] = msg.twist.twist.angular.z

	def robot5_odom_callback(self, msg):
		self.robot_position_x[4] = msg.pose.pose.position.x + self.initial_position_x[4]
		self.robot_position_y[4] = msg.pose.pose.position.y + self.initial_position_y[4]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[4] = yaw
		self.robot_linear_vel_x[4] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[4] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[4] = msg.twist.twist.angular.z

	def robot6_odom_callback(self, msg):
		self.robot_position_x[5] = msg.pose.pose.position.x + self.initial_position_x[5]
		self.robot_position_y[5] = msg.pose.pose.position.y + self.initial_position_y[5]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[5] = yaw
		self.robot_linear_vel_x[5] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[5] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[5] = msg.twist.twist.angular.z

	def robot7_odom_callback(self, msg):
		self.robot_position_x[6] = msg.pose.pose.position.x + self.initial_position_x[6]
		self.robot_position_y[6] = msg.pose.pose.position.y + self.initial_position_y[6]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[6] = yaw
		self.robot_linear_vel_x[6] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[6] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[6] = msg.twist.twist.angular.z

	def robot8_odom_callback(self, msg):
		self.robot_position_x[7] = msg.pose.pose.position.x + self.initial_position_x[7]
		self.robot_position_y[7] = msg.pose.pose.position.y + self.initial_position_y[7]
		roll = pitch = yaw = 0.0
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.robot_orientation_z[7] = yaw
		self.robot_linear_vel_x[7] = msg.twist.twist.linear.x
		self.robot_linear_vel_y[7] = msg.twist.twist.linear.y
		self.robot_angular_vel_z[7] = msg.twist.twist.angular.z
		

rospy.init_node('basic')
basic = Basic()
rospy.spin()
