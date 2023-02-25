import message_filters
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ambf_msgs.msg import RigidBodyState
from os.path import join
import atexit
import time
import sys
import os

stereo_l_img = []
stereo_r_img = []
segment_l_img = []
segment_r_img = []
psm1_kin = []
psm2_kin = []


start_time = time.time()

def kin_img_cb(psm1, psm2, stereo_l, stereo_r, segment_l, segment_r):
	"""
	ros approximate time synchronizer callback, invoked when target messages are approximately synced
	"""
	sys.stdout.write('\r-- Time past: %02.1f' % float(time.time() - start_time))
        sys.stdout.flush()
	stereo_l_img.append(bridge.imgmsg_to_cv2(stereo_l,"bgr8"))
	stereo_r_img.append(bridge.imgmsg_to_cv2(stereo_r,"bgr8"))
	segment_l_img.append(bridge.imgmsg_to_cv2(segment_l,"bgr8"))
        segment_r_img.append(bridge.imgmsg_to_cv2(segment_r,"bgr8"))
        psm1_kin.append(np.array([psm1.joint_positions] + [psm1.joint_velocities]))
	psm2_kin.append(np.array([psm2.joint_positions] + [psm2.joint_velocities]))

def save_data_cb():
	"""
	data storage callback, invoked when script terminates
	"""
	if not os.path.exists('./data'):
		os.mkdir('./data')
        print("saving data...")
	np.save(join('./data', 'stereo-l.npy'), stereo_l_img)
	np.save(join('./data', 'stereo-r.npy'), stereo_r_img)
	np.save(join('./data', 'segment-l.npy'), segment_l_img)
	np.save(join('./data', 'segment-r.npy'), segment_r_img)
	np.save(join('./data', 'psm1_kin.npy'), psm1_kin)
	np.save(join('./data', 'psm2_kin.npy'), psm2_kin)
        print("done saving...")
	

if __name__ == '__main__':
	
	bridge = CvBridge()

	print("started collecting PSM 1&2 baselink, stereo image, and segmentation image ...")
	print("Press Ctrl-C to stop collecting...")
	
        # init ros node
	rospy.init_node('collect_ambf_data', anonymous=True)
	
	# create subscriber
	PSM_1 = message_filters.Subscriber("/ambf/env/psm1/baselink/State", RigidBodyState)
	PSM_2 = message_filters.Subscriber("/ambf/env/psm2/baselink/State", RigidBodyState)
	STEREO_L = message_filters.Subscriber("/ambf/env/cameras/cameraL/ImageData", Image)
        STEREO_R = message_filters.Subscriber("/ambf/env/cameras/cameraR/ImageData", Image)
        SEGMENT_L = message_filters.Subscriber("/ambf/env/cameras/segmentation_cameraL/ImageData", Image)
        SEGMENT_R = message_filters.Subscriber("/ambf/env/cameras/segmentation_cameraR/ImageData", Image)
	
	# create approximate time synchronizer
	ts = message_filters.ApproximateTimeSynchronizer([PSM_1, PSM_2, STEREO_L, STEREO_R, SEGMENT_L, SEGMENT_R], queue_size=10, slop=0.2, allow_headerless=False)
        ts.registerCallback(kin_img_cb)
	
	# spin ros until shutdown, i.e., Ctrl-C
	rospy.spin()

        # print msg container info
	print(len(stereo_l_img))
        print(len(stereo_r_img))
        print(len(segment_l_img))
        print(len(segment_r_img))
        print(len(psm1_kin))
        print(len(psm2_kin))

	# save data
        atexit.register(save_data_cb)
