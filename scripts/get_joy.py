import rospy
from razer_hydra.msg import Hydra
# from sensor_msgs.msg import Joy
import numpy as np

# global axes_list, buttons_list
# axes_list = []
# buttons_list = []
#data_list = []
global x
x = []
def callback(data):
	x.append(data)
	pass
	# axes_list.append(data.axes)
	# buttons_list.append(data.buttons)
	# np.savez_compressed('./joypath3',A=axes_list,B=buttons_list)


def start():
	rospy.init_node('getdata',anonymous = True)
	# rospy.Subscriber('hydra_calib',Hydra,callback)
	rospy.Subscriber('hydra_calib',Hydra,callback,queue_size=1)

	rospy.spin()


if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass
