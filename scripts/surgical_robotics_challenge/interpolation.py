import numpy as np


# simplified Interpolation class to compute the path for move_jp
# derived from Interpolation.interp_logic
class Interpolation_custom(object):
    def __init__(self):
        self._t0 = 0.0
        self._tf = 0.0

        self._coefficients = np.zeros([6, 1])
        self._T_mat = np.zeros([6, 6])
        self._boundary_conditions = np.zeros([6, 1])

        self._dimensions = 1
        self._t_array = []
        self._x = np.zeros([1, 1])
        # Define a custom margin of error for t0
        self._t0_moe = 0.01
        pass

    def get_interpolated_x(self, t):
        t = t - self._t0
        t_mat = np.column_stack((t ** 0, t ** 1, t ** 2, t ** 3, t ** 4, t ** 5))
        self._x = np.matmul(self._coefficients.transpose(), t_mat.transpose())
        return self._x

    def compute_interpolation_params(self, x0, xf, dx0, dxf, ddx0, ddxf, t0, tf):
        self._dimensions = x0.size
        self._t0 = t0
        self._tf = tf
        tf = tf - t0
        t0 = 0

        self._T_mat = np.mat([[1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
                              [0, 1, 2 * t0, 3 * (t0 ** 2), 4 * (t0 ** 3), 5 * (t0 ** 4)],
                              [0, 0, 2, 6 * t0, 12 * (t0 ** 2), 20 * (t0 ** 3)],
                              [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
                              [0, 1, 2 * tf, 3 * (tf ** 2), 4 * (tf ** 3), 5 * (tf ** 4)],
                              [0, 0, 2, 6 * tf, 12 * (tf ** 2), 20 * (tf ** 3)]])

        self._boundary_conditions = np.mat([x0, dx0, ddx0, xf, dxf, ddxf])
        self._coefficients = np.matmul(np.linalg.inv(self._T_mat), self._boundary_conditions)
