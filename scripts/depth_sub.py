import rospy, math, random
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class DepthSub:
    def __init__(self, depth_topic="/ambf/env/cameras/cameraL/DepthData"):
        self.depth_sub = rospy.Subscriber(depth_topic, PointCloud2, self.depth_cb)

    def depth_cb(self, depth_msg):
        print('Receiving Depth Msg')
        gen = pc2.read_points(depth_msg, skip_nans=True, field_names=("x", "y", "z"))
        print(type(gen))
        print(len(list(gen)))
        # for p in pc2.read_points(depth_msg, field_names=("x", "y", "z"), skip_nans=True):
        #     print " x : %f  y: %f  z: %f" % (p[0], p[1], p[2])


rospy.init_node('depth_sub')
ds = DepthSub()
rospy.sleep(5.0)
