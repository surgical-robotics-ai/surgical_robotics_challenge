import rospy
from std_msgs.msg import Bool
import random

def publisher():
    pub = rospy.Publisher('communication_loss', Bool, queue_size=1)
    rospy.init_node('communication_loss_talker', anonymous=True)

    i = 0
    flag = False

    time_period = 5
    loss_period = 1.5

    hz = 200
    rate = rospy.Rate(hz) # 200hz

    print("TIME_period: " + str(time_period) + " sec")
    print("LOSS_period: "  + str(loss_period) + " sec")

    while not rospy.is_shutdown():
        i += 1

        if (i == int(time_period - loss_period) * hz):
            flag = True
            pub.publish(flag)

        if (i == int(time_period * hz)):
            flag = False
            pub.publish(flag)
            i = 0

            # Added for random generation loss
            time_period = 10 * random.random()
            loss_period = random.random() * 5
            while time_period <  loss_period:
                time_period = loss_period +1 

            print("TIME_period: " + str(time_period) + " sec")
            print("LOSS_period: "  + str(loss_period) + " sec")

        #pub.publish(flag)
        rate.sleep()

    # now = rospy.get_time()
    # flag = False
    # while not rospy.is_shutdown():
    #
    #     if (rospy.get_time() - now > 10):
    #         flag = True
    #
    #     if (rospy.get_time() - now > 12):
    #         flag = False
    #         now = rospy.get_time()
    #
    #     pub.publish(flag)
    #     rate.sleep()






if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
