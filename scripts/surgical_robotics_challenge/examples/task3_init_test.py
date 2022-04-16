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
psm2 = PSM(c, 'psm2')
time.sleep(0.5)
# First we shall move the PSM to its initial pose using joint commands OR pose command
psm2.servo_jp([-0.4, -0.22, 1.39, -1.64, -0.37, -0.11])
# Open the Jaws
psm2.set_jaw_angle(0.8)
# Sleep to achieve the target pose and jaw angle
time.sleep(1.0)
# Instantiate the needle initialization class
needle = NeedleInitialization()
# Sanity sleep
time.sleep(0.5)
# This method will automatically start moving the needle to be with the PSM2's jaws
needle.lock_at_tip()
# Wait for the needle to reach the target
while not needle.is_reached():
   time.sleep(0.1)
time.sleep(0.5)
for i in range(30):
   # Close the jaws to grasp the needle
   # Calling it repeatedly a few times so that the needle is forced
   # between the gripper tips and grasped properly
   psm2.set_jaw_angle(0.0)
   time.sleep(0.01)
time.sleep(0.5)
# Don't forget to release the needle control loop to move it freely.
needle.release()
time.sleep(2.0)
# Open the jaws to let go of the needle from grasp
psm2.set_jaw_angle(0.8)
time.sleep(2.0)