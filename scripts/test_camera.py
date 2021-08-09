from camera import *
from ambf_client import Client
from PyKDL import Frame, Rotation, Vector, Twist
from obj_control_gui import ObjectGUI
import sys


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


c = Client('cam_test')
c.connect()
time.sleep(0.5)

cam = Camera(c, 'CameraFrame')

T_c_w = cam.measured_cp()

# for i in range(100):
#     print(i,"> ", cam.measured_cp().p)
#     T_c_w.p[0] = T_c_w.p[0] + 0.1
#     cam.move_cp(T_c_w)
#     time.sleep(1.0)

gui = ObjectGUI('Camera Velocity Control')
dt = 0.001
while True:
    try:
        gui.App.update()
        # print(i,"> ", cam.measured_cp().p)
        twist = Twist()
        twist.vel = Vector(gui.x, gui.y, gui.z)
        twist.rot = Vector(gui.ro, gui.pi, gui.ya)
        cam.move_cv(twist, dt)
        time.sleep(dt)
    except KeyboardInterrupt:
        print("Bye")
        sys.exit()
