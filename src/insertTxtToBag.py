

import rosbag

import tf

from geometry_msgs.msg import TransformStamped

from tf2_msgs.msg import TFMessage

import numpy

from shutil import copyfile

from tf.transformations import quaternion_from_euler

import rospy

import sys

​

​

bagfile_name = sys.argv[1]  #parameter

copyfile(bagfile_name,"./final_bags/"+bagfile_name+"_final.bag")

final_bag = rosbag.Bag("./final_bags/"+bagfile_name+"_final.bag","a")

​

try:

	corrections = numpy.loadtxt(bagfile_name+"_edited.bag_correction.txt",dtype=str)

	#corrections = [t,x,y,z, o_x,o_y,o_z,o_w, parent_frame_id, child_frame_id]

​

	tf_tree = TFMessage()

	map2odom = TransformStamped()

	odom2basefoot = TransformStamped()

​

	basefoot2baselink = TransformStamped()

​

	basefoot2baselink.header.frame_id = "robot3/base_footprint"

	basefoot2baselink.child_frame_id = "robot3/base_link"

	basefoot2baselink.transform.translation.x = 0

	basefoot2baselink.transform.translation.y = 0

	basefoot2baselink.transform.translation.z = 0

	basefoot2baselink.transform.rotation.x = 0

	basefoot2baselink.transform.rotation.y = 0

	basefoot2baselink.transform.rotation.z = 0

	basefoot2baselink.transform.rotation.w = 1

	

	baselink2velo = TransformStamped()

	baselink2velo.header.frame_id = "robot3/base_link"

	baselink2velo.child_frame_id = "robot3/velodyne_link"

	baselink2velo.transform.translation.x = 1.1187

	baselink2velo.transform.translation.y = 0.1354

	baselink2velo.transform.translation.z = 1.75

	baselink2velo.transform.rotation.x = 0.00236069 #value given by quaternion_from_euler(0.017,0.01,1.594682713) function

	baselink2velo.transform.rotation.y = 0.00957453 #value given by quaternion_from_euler(0.017,0.01,1.594682713) function

	baselink2velo.transform.rotation.z = 0.71543678 #value given by quaternion_from_euler(0.017,0.01,1.594682713) function

	baselink2velo.transform.rotation.w = 0.69860788 #value given by quaternion_from_euler(0.017,0.01,1.594682713) function

	

​

	#final_bag.write('/tf',)

	best_index = 0

	for topic,msg,t in final_bag.read_messages(topics=['/robot3/control/odom']):

​

		# choosing the best correction for the odometry using the closest time(in the past)

		min_diff = 1000000000

		time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9

		for index in range(best_index,len(corrections)):

			current_diff = time - float(corrections[index][0])

			if current_diff > 0 and current_diff < min_diff:

				best_index = index

				min_diff = current_diff

			if current_diff < 0:

				break

​

		# applying selected correction in the tf

		#corrections = [t,x,y,z, o_x,o_y,o_z,o_w, parent_frame_id, child_frame_id]

		map2odom.header.stamp = msg.header.stamp # use the timestamp from the odom

		map2odom.header.frame_id = "map_laser2d"

		map2odom.child_frame_id = "robot3/odom"

		map2odom.transform.translation.x = float(corrections[index][1])

		map2odom.transform.translation.y = float(corrections[index][2])

		map2odom.transform.translation.z = float(corrections[index][3])

		map2odom.transform.rotation.x = float(corrections[index][4])

		map2odom.transform.rotation.y = float(corrections[index][5])

		map2odom.transform.rotation.z = float(corrections[index][6])

		map2odom.transform.rotation.w = float(corrections[index][7])

​

		#using odom topic to populat the tf transform

		odom2basefoot.header.stamp = msg.header.stamp

		odom2basefoot.header.frame_id = "robot3/odom"

		odom2basefoot.child_frame_id = "robot3/base_footprint"

		odom2basefoot.transform.translation.x = msg.pose.pose.position.x

		odom2basefoot.transform.translation.y = msg.pose.pose.position.y

		odom2basefoot.transform.translation.z = msg.pose.pose.position.z

		odom2basefoot.transform.rotation.x = msg.pose.pose.orientation.x

		odom2basefoot.transform.rotation.y = msg.pose.pose.orientation.y

		odom2basefoot.transform.rotation.z = msg.pose.pose.orientation.z

		odom2basefoot.transform.rotation.w = msg.pose.pose.orientation.w

​

		#the values on those transform are fixed. we only update the time

		basefoot2baselink.header.stamp = msg.header.stamp

​

		baselink2velo.header.stamp = msg.header.stamp

​

		# build the message and append to the rosbag file

		tf_tree.transforms = []

		tf_tree.transforms.append(map2odom)

		tf_tree.transforms.append(odom2basefoot)

		tf_tree.transforms.append(basefoot2baselink)

		tf_tree.transforms.append(baselink2velo)

​

		#print tf_tree

		final_bag.write('/tf',tf_tree,msg.header.stamp)

​

	final_bag.close()

​

except Exception as e:

	raise e

​

finally:

	final_bag.close()


