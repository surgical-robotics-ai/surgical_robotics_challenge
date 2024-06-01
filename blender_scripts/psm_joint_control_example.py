import surgical_robotics_challenge
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM

simulation_manager = SimulationManager("my_example_client")
psm1 = PSM(simulation_manager, 'psm1')


psm1.set_jaw_angle(0.0)

jp = [0., 0., 0.135, -0.2, -0.3, -0.2]
psm1.servo_jp([0., 0., 0.135, -0.2, -0.3, -0.2])
