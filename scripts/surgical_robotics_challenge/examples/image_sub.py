import rospy
from sensor_msgs.msg import Image


class ImageSub:
    def __init__(self, image_topic="/ambf/env/cameras/cameraL/ImageData"):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_cb)

    def image_cb(self, image_msg):
        print('Receiving Image Msg')
        print(type(image_msg))


rospy.init_node('image_sub')
ds = ImageSub()
rospy.sleep(5.0)
