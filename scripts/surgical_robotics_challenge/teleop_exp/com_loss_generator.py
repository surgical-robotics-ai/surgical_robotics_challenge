import rospy
from std_msgs.msg import Bool
import random
import time

def com_loss_publisher():
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
            loss_period = 3 * random.random()
            while time_period <  loss_period + 2:
                time_period = loss_period + 3 

            print("TIME_period: " + str(time_period) + " sec")
            print("LOSS_period: "  + str(loss_period) + " sec")

        rate.sleep()


if __name__ == '__main__':
    print("COMMUNICATION LOSS GENERATOR")
    print("Please enter your seed:")
    time.sleep(2)
    num_seed = input(":\n")
    
    print(f'Seed Number: {num_seed}')

    random.seed(num_seed)
    try:
        com_loss_publisher()
    except rospy.ROSInterruptException:
        pass
