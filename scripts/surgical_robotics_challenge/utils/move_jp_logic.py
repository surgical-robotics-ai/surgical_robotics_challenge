# Import da Vinci related classes
import rospy
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.scene import Scene
from surgical_robotics_challenge.simulation_manager import SimulationManager

from interp_logic import Interpolation
import numpy as np
import time

# inherited methods from PSM class
# inherited methods from Interpolation class to compute the path
class PSM_with_move_jp(Interpolation, PSM):

    def __init__(self, simul=None, n=None):
        PSM.__init__(self, simulation_manager=simul, name=n, add_joint_errors=True)
        Interpolation.__init__(self)

        self._cur_pos = list()
        self._nxt_pos = list()
        self._path = dict()

    def set_pos(self, jp_pos=np.zeros(6)):
        self._cur_pos = jp_pos

    def _make_list(self, old):
        ans = list()
        for rows in old:
            new = list()
            for element in rows:
                new.append(float(element))
            ans.append(new)
        return ans

    def _compute_path(self, nxt_pos, v0=np.zeros(6), vf=np.zeros(6), a0=np.zeros(6), af=np.zeros(6), t0=0.0, tf=1.0):
        self._nxt_pos = nxt_pos
        p0 = self._cur_pos
        pf = self._nxt_pos
        # Compute the Time Mat and Computation Params implicitly in the class
        super().compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)

    def _follow_trajectory(self, tf=1.0):

        rate = rospy.Rate(200)
        init_time = time.time()

        while not rospy.is_shutdown():
            cur_time = time.time()
            adj_time = cur_time - init_time
            if adj_time > tf:
                break
            val = super().get_interpolated_x(adj_time)
            self._path[adj_time] = val
            self.servo_jp(val)
            rate.sleep()

        self._cur_pos = self._nxt_pos

    def move_jp(self, jp_pos, tf=1.0):
        self._nxt_pos = jp_pos
        self._compute_path(self._nxt_pos, tf)

        self._follow_trajectory(tf)

    def get_path(self):
        return self._path

# testing when running in main
if __name__ == '__main__':

    simulation_manager = SimulationManager('my_example_client')
    psm1 = PSM_with_move_jp(simulation_manager, 'psm1')

    time.sleep(0.5)
    world_handle = simulation_manager.get_world_handle()
    scene = Scene(simulation_manager)
    time.sleep(0.5)
    print("Resetting the world")
    world_handle.reset()

    '''
    J1 ------------ J2
    |               |
    |               |
    |               |
    |               |
    J4 ------------ J3
    '''

    J1 = [0.5, 0.5, 0.1, 0, 0, 0]
    J2 = [-0.5, 0.5, 0.1, 0, 0, 0]
    J3 = [-0.5, -0.5, 0.1, 0, 0, 0]
    J4 = [0.5, -0.5, 0.1, 0, 0, 0]

    psm1.set_pos(J1)
    psm1.move_jp(J1)

    go_around = [J2, J3, J4, J1]

    for jp in go_around:
        psm1.move_jp(jp)
        time.sleep(0.5)
