from ecm_arm import ECM
import time
import sys
from ambf_client import Client
from jnt_control_gui import JointGUI


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


c = Client('ecm_test')
c.connect()
time.sleep(0.5)

ecm = ECM(c, 'CameraFrame')
gui = JointGUI("ECM JOINTS", 4, ["j0", "j1", "j2", "j3"], resolution=0.00001)
dt = 0.005
while True:
    try:
        gui.App.update()
        ecm.servo_jp(gui.jnt_cmds)
        time.sleep(dt)
    except KeyboardInterrupt:
        print("Bye")
        sys.exit()
