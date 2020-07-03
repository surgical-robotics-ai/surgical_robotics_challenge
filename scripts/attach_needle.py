from ambf_client import Client
from PyKDL import Vector, Rotation
import time


def attach_needle(client, needle_name, psm_name):
    needle = client.get_obj_handle(needle_name)
    # psm_name =
    tool_yaw_link = client.get_obj_handle(psm_name + '/toolyawlink')

    p = tool_yaw_link.get_pos()
    q = tool_yaw_link.get_rot()

    R_tINw = Rotation.Quaternion(q.x, q.y, q.z, q.w)

    # we need to move the needle based on the Pose of the toolyawlink.
    # The yawlink's negative Y direction faces the grasp
    # position. Therefore, lets add a small offset to the P of yawlink.
    y_offset = -0.09
    P_nINw = Vector(p.x, p.y, p.z) + R_tINw * Vector(0, y_offset, 0)

    # If you want to rotate the needle to a certain relative orientation
    # add another Rotation and multiply on the R.H.S of R_tINw in the
    # Equation below.
    R_nINw = R_tINw

    needle.set_pos(P_nINw[0], P_nINw[1], P_nINw[2])
    needle.set_rpy(R_nINw.GetRPY()[0], R_nINw.GetRPY()[1], R_nINw.GetRPY()[2])

    # Wait for the needle to get there
    time.sleep(3)

    # You should see the needle in the center of the two fingers.
    # If the gripper is not already closed, you shall have to manually
    # close it to grasp the needle. You should probably automate this in the testIK script.

    # Don't forget to release the pose command from the needle. We can
    # do so by calling:
    needle.set_force(0, 0, 0)
    needle.set_torque(0, 0, 0)


c = Client('test')
c.connect()
attach_needle(c, 'Needle', 'psm3')