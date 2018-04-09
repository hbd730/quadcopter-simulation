from context import utils
from utils.quaternion import Quaternion
import unittest
import numpy as np

class TestQuaternionMehods(unittest.TestCase):
    def test_from_v_theta(self):
        x,y,z = np.eye(3)
        q = Quaternion.from_v_theta(x, np.pi/2)
        expected = [np.cos(np.pi/4), np.sin(np.pi/4), 0, 0]
        self.assertEqual(q, Quaternion(expected))

    def test_as_v_thta(self):
        x = [1.0,0.0,0.0]
        q = Quaternion.from_v_theta(x, np.pi/2)
        expected_v = np.array(x)
        expected_theta = np.pi/2
        actual_v, actual_theta = q.as_v_theta()
        print actual_v, actual_theta, expected_v
        self.assertTrue(np.array_equal(expected_v, actual_v))
        self.assertEqual(expected_theta, actual_theta)

if __name__ == '__main__':
    unittest.main()
