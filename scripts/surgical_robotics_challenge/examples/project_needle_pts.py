"""
Script for projecting needle salient point in to the image plane

To use correctly this script make sure that:
   1) the /ambf/env/cameras/cameraL/ImageData topic is available
   2) the cameras can view the needle in the scene

Script tested in python 3.8 and ros Noetic

Juan Antonio Barragan 

"""

import json
import cv2
import numpy as np
from numpy.linalg import inv
from surgical_robotics_challenge.scene import Scene
from surgical_robotics_challenge.camera import Camera
from ambf_client import Client
import time
import tf_conversions.posemath as pm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.units_conversion import SimToSI


np.set_printoptions(precision=3, suppress=True)


class ImageSub:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_subs = rospy.Subscriber(
            "/ambf/env/cameras/cameraL/ImageData", Image, self.left_callback
        )

        self.left_frame = None
        self.left_ts = None

        # Wait a until subscribers and publishers are ready
        rospy.sleep(0.5)

    def left_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_frame = cv2_img
            self.left_ts = msg.header.stamp
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    # Connect to AMBF and setup image suscriber
    rospy.init_node("image_listener")
    saver = ImageSub()

    simulation_manager = SimulationManager("needle_projection_ex")
    time.sleep(0.5)
    # cam = simulation_manager.get_obj_handle("cameraL")
    scene = Scene(simulation_manager)  # Provides access to needle and entry/exit points
    ambf_cam_l = Camera(simulation_manager, "/ambf/env/cameras/cameraL")
    ambf_cam_frame = ECM(simulation_manager, "CameraFrame")

    assert ambf_cam_l is not None, "CameraL not found"
    print(ambf_cam_l)

    # Calculate opencv camera intrinsics
    fvg = 1.2
    width = 640
    height = 480
    f = height / (2 * np.tan(fvg / 2))

    intrinsic_params = np.zeros((3, 3))
    intrinsic_params[0, 0] = f
    intrinsic_params[1, 1] = f
    intrinsic_params[0, 2] = width / 2
    intrinsic_params[1, 2] = height / 2
    intrinsic_params[2, 2] = 1.0

    # Get pose for the needle and the camera
    print(f"Needle measured cp:\n {scene.needle_measured_cp()}")
    print(f"Camera Left frame cp: {ambf_cam_l.get_T_c_w()}")

    T_WN = pm.toMatrix(scene.needle_measured_cp())  # Needle to world
    T_FL = pm.toMatrix(ambf_cam_l.get_T_c_w())  # CamL to CamFrame
    T_WF = pm.toMatrix(ambf_cam_frame.get_T_c_w())  # CamFrame to world

    # Get image
    img = saver.left_frame

    # Calculate needle to left camera transformation
    T_WL = T_WF.dot(T_FL)
    T_LN = inv(T_WL).dot(T_WN)

    # Convert AMBF camera axis to Opencv Camera axis
    F = np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    T_LN_CV2 = F.dot(T_LN)

    # Project center of the needle with OpenCv
    rvecs, _ = cv2.Rodrigues(T_LN_CV2[:3, :3])
    tvecs = T_LN_CV2[:3, 3]

    # needle_salient points
    theta = np.linspace(np.pi / 3, np.pi, num=8).reshape((-1, 1))

    # Scale salient points to match unit conversion in simulation manager
    radius = 0.01018 / SimToSI.linear_factor
    needle_salient = radius * np.hstack((np.cos(theta), np.sin(theta), theta * 0))

    # Project points
    img_pt, _ = cv2.projectPoints(
        needle_salient,
        rvecs,
        tvecs,
        intrinsic_params,
        np.float32([0, 0, 0, 0, 0]),
    )

    # Print information
    print("intrinsic")
    print(intrinsic_params)
    print("T_WN. Transform from needle to world")
    print(T_WN)
    print("T_WC. Transform from camera to world")
    print(T_WL)
    print("T_CN. Transform from the needle to cam")
    print(T_LN)

    # Display image
    for i in range(img_pt.shape[0]):
        img = cv2.circle(img, (int(img_pt[i, 0, 0]), int(img_pt[i, 0, 1])), 3, (255, 0, 0), -1)

    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
