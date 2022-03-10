from surgical_robotics_challenge.kinematics.psmFK import *
from PyKDL import Frame, Rotation, Vector
import rospy
from sensor_msgs.msg import JointState
import numpy as np

import matplotlib.pyplot as plt
from ambf_msgs.msg import RigidBodyState
from mpl_toolkits import mplot3d


class JointSub:
    def __init__(self, joint_topic="/MTMR/measured_js"):
        self.js_sub = rospy.Subscriber(joint_topic, JointState, self.js_cb)
        self._jointPosition = []
        self.cp = []

    def js_cb(self, js_msg):
        # print(js_msg.position)
        self._jointPosition.append(js_msg.position)

    def calculate_cp(self):
        print("Calculating cp...")
        joint_position = np.array(self._jointPosition).reshape((-1, 7))
        for i in range(joint_position.shape[0]):
            cp_homo = compute_FK(joint_position[i], 7)
            self.cp.append([cp_homo.item((0, 3)), cp_homo.item((1, 3)), cp_homo.item((2, 3))])
        return np.array(self.cp).reshape(-1, 3)

class PuzzleSub:
    def __init__(self, puzzle_topic="/ambf/icl/PuzzleRed1/State"):
        self.puzzle_sub = rospy.Subscriber(puzzle_topic, RigidBodyState, self.state_cb)
        self._statePosition = []
        self.cp = []

    def state_cb(self, state_msg):
        # print(js_msg.position)
        self._statePosition.append([state_msg.pose.position.x, state_msg.pose.position.y, state_msg.pose.position.z])


    def location(self):
        return np.array(self._statePosition).reshape(-1, 3)


if __name__ == "__main__":
    # subscribe and store joint angles
    # rospy.init_node('js_sub')
    # print("collecting data...")
    # ds = JointSub()
    # rospy.sleep(100.0)
    # cp = ds.calculate_cp()
    #
    # fig = plt.figure()
    # ax = plt.axes(projection="3d")
    #
    # ax.plot3D(cp[:, 0], cp[:, 1], cp[:, 2])
    #
    # plt.show()

    # subscribe and store puzzle position
    rospy.init_node('state_sub')
    print("collecting data...")
    pl1 = PuzzleSub("/ambf/icl/PuzzleRed1/State")
    pl2 = PuzzleSub("/ambf/icl/PuzzleRed4/State")
    pl3 = PuzzleSub("/ambf/icl/PuzzleRed5/State")

    rospy.sleep(100.0)

    puzzle_location1 = pl1.location()
    puzzle_location2 = pl2.location()
    puzzle_location3 = pl3.location()

    # Saving the data to csv file
    np.savetxt('../../Data/puzzleRed1.csv', puzzle_location1, delimiter=',')
    np.savetxt('../../Data/puzzleRed4.csv', puzzle_location2, delimiter=',')
    np.savetxt('../../Data/puzzleRed5.csv', puzzle_location3, delimiter=',')



    # Visualizing the plot
    fig = plt.figure()
    ax = plt.axes(projection="3d")

    ax.plot3D(puzzle_location1[:, 0], puzzle_location1[:, 1], puzzle_location1[:, 2])
    ax.plot3D(puzzle_location2[:, 0], puzzle_location2[:, 1], puzzle_location2[:, 2])
    ax.plot3D(puzzle_location3[:, 0], puzzle_location3[:, 1], puzzle_location3[:, 2])

    plt.show()
