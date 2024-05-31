import time

from surgical_robotics_challenge.utils.task3_init import NeedleInitialization
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.kinematics.psmKinematics import ToolType

tool_id = ToolType.Default

simulation_manager = SimulationManager('task_3_setup_test')
time.sleep(0.5)
w = simulation_manager.get_world_handle()
time.sleep(0.2)
w.reset_bodies()
time.sleep(0.2)
psm2 = PSM(simulation_manager, 'psm2', tool_id=tool_id)
time.sleep(0.5)
# First we shall move the PSM to its initial pose using joint commands OR pose command
psm2.servo_jp([-0.4, -0.22, 0.139, -1.64, -0.37, -0.11])
# Open the Jaws
psm2.set_jaw_angle(0.8)
# Sleep to achieve the target pose and jaw angle
time.sleep(1.0)
# Instantiate the needle initialization class
needle = NeedleInitialization(simulation_manager)
psm2_tip = simulation_manager.get_obj_handle('psm2/toolyawlink')
# Sanity sleep
time.sleep(0.5)
# This method will automatically start moving the needle to be with the PSM2's jaws
needle.move_to(psm2_tip)
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