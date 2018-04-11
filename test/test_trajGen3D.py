from context import trajGen3D
import unittest
import numpy as np
import math

class TestGetHelixWaypoints(unittest.TestCase):
    def test_get_helix_waypoints_size(self):
        t = 0
        n = 8
        waypoints = trajGen3D.get_helix_waypoints(t, n)
        expected_shape = (8, 3)
        self.assertEqual(waypoints.shape, expected_shape)

    def test_get_helix_waypoints_value(self):
        n = 8
        start_t = 0
        T = np.linspace(start_t, 2*np.pi, n)
        expected = np.zeros((n,3))

        for i, t in enumerate(T):
            expected[i] = np.array([np.cos(t),np.sin(t),t])

        waypoints = trajGen3D.get_helix_waypoints(start_t, n)
        np.testing.assert_array_equal(waypoints, expected)

class TestGetPolyCC(unittest.TestCase):
    def test_get_poly_cc_n(self):
        cc = trajGen3D.get_poly_cc(4, 0, 0)
        expected = [1, 0, 0, 0]
        np.testing.assert_array_equal(cc, expected)

        cc = trajGen3D.get_poly_cc(8, 0, 0)
        expected = [1, 0, 0, 0, 0, 0, 0, 0]
        np.testing.assert_array_equal(cc, expected)

    def test_get_poly_cc_k(self):
        cc = trajGen3D.get_poly_cc(4, 1, 1)
        expected = [0, 1, 2, 3]
        np.testing.assert_array_equal(cc, expected)

        cc = trajGen3D.get_poly_cc(4, 2, 2)
        expected = [0, 0, 2, 12]
        np.testing.assert_array_equal(cc, expected)

        cc = trajGen3D.get_poly_cc(8, 7, 1)
        expected = [0, 0, 0, 0, 0, 0, 0, math.factorial(7)]
        np.testing.assert_array_equal(cc, expected)

        cc = trajGen3D.get_poly_cc(8, 8, 1)
        expected = np.zeros(8)
        np.testing.assert_array_equal(cc, expected)

    def test_get_poly_cc_t(self):
        cc = trajGen3D.get_poly_cc(4, 0, 1)
        expected = [1, 1, 1, 1]
        np.testing.assert_array_equal(cc, expected)

        cc = trajGen3D.get_poly_cc(8, 0, 2)
        expected = [1, 2, 4, 8, 16, 32, 64, 128]
        np.testing.assert_array_equal(cc, expected)

        cc = trajGen3D.get_poly_cc(8, 1, 1)
        expected = np.linspace(0, 7, 8)
        np.testing.assert_array_equal(cc, expected)

        t = 2
        cc = trajGen3D.get_poly_cc(8, 1, t)
        expected = [0, 1, 2*t, 3*np.power(t,2), 4*np.power(t,3), 5*np.power(t,4), 6*np.power(t,5), 7*np.power(t,6)]
        np.testing.assert_array_equal(cc, expected)

    def test_get_poly_cc_input(self):
        with self.assertRaises(AssertionError):
            trajGen3D.get_poly_cc(-1, -1, 0)

        with self.assertRaises(AssertionError):
            trajGen3D.get_poly_cc(0, -3, -3)

class TestMST(unittest.TestCase):
    def test_MST_output_size(self):
        t = 0
        n = 8
        waypoints = trajGen3D.get_helix_waypoints(t, n)
        C = trajGen3D.MST(waypoints[:,0], t)
        self.assertEqual(C.shape, (8*(n-1), 1))

class TestGenerateTrajectory(unittest.TestCase):
    def test_desired_state_size(self):
        t = 0
        n = 9
        v = 0.1 # m/s
        waypoints = trajGen3D.get_helix_waypoints(t, n)
        desired_state = trajGen3D.generate_trajectory(1.3, v, waypoints)
        self.assertEqual(desired_state.pos.shape, (3,))
        self.assertEqual(desired_state.vel.shape, (3,))
        self.assertEqual(desired_state.acc.shape, (3,))
        des_x_dot, des_y_dot, des_z_dot = desired_state.vel
        print "desired_state", desired_state

if __name__ == '__main__':
    unittest.main()
