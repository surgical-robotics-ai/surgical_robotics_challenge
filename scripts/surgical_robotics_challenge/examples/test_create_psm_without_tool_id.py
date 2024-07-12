
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM

sim = SimulationManager('sim_man')

tool_id_body = sim.get_obj_handle('psm1/tool_id')
psm1 = PSM(sim, 'psm1')
psm2 = PSM(sim, 'psm2')

print(tool_id_body.get_ros_name())
print(psm1.tool_id)
print(psm2.tool_id)
