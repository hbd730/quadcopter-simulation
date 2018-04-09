from context import trajGen3D
import unittest
import numpy as np

class TestTrajGen3DMehods(unittest.TestCase):
    def test_get_helix_waypoints_size(self):
        t = 0
        n = 8
        waypoints = trajGen3D.get_helix_waypoints(t, n)
        expected_shape = (8, 3)
        self.assertEqual(waypoints.shape, expected_shape)

if __name__ == '__main__':
    unittest.main()
