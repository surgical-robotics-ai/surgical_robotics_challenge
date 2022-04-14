import time

from surgical_robotics_challenge.utils.task3_init import NeedleInitialization
from surgical_robotics_challenge.psm_arm import PSM
from ambf_client import Client

c = Client('task_3_setup_test')
c.connect()
time.sleep(0.5)
w = c.get_world_handle()
time.sleep(0.2)
w.reset_bodies()
time.sleep(0.2)
psm = PSM(c, 'psm2')
time.sleep(0.5)
psm.servo_jp([-0.4, -0.22, 1.39, -1.64, -0.37, -0.11])
psm.set_jaw_angle(0.8)
time.sleep(2.0)

n = NeedleInitialization()
time.sleep(0.2)
n.lock_at_tip()

while not n.is_reached():
    time.sleep(0.1)

time.sleep(0.5)
for i in range(30):
    psm.set_jaw_angle(0.0) # Close the jaws to grasp the needle
    time.sleep(0.01)
time.sleep(1.0)
n.release() # Don't forget to release the needle
time.sleep(5.0)
psm.set_jaw_angle(0.8)
time.sleep(2.0)