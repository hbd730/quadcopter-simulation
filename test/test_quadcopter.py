from context import model
from model.quadcopter import Quadcopter
from utils.quaternion import Quaternion
import numpy as np
import unittest

class TestQuadcopterMethod(unittest.TestCase):
    def test_world_frame(self):
        pos = (0,0,0)
        attitude = [0,0,0]
        quadcopter = Quadcopter(pos, attitude)
        frame = quadcopter.world_frame()
        # let's simply test the dimension of output matrix for now
        self.assertEqual((3,6), frame.shape)

    def test_state_dot(self):
        pos = (0,0,0)
        attitude = [0,0,np.pi/6]
        quadcopter = Quadcopter(pos, attitude)
        F = 1.831655
        M = np.array([[1.45879171444454e-05, -2.09414787558272e-05, 0.000133122686296450]]).T
        quadcopter.state =np.array([0.099008422415915,0.0990164877426797,0.0989554489338614,0.257998046804099,0.257989778737415,
                0.257530910432372,0.998488763307995,0.0165084622039285,-0.0182176180214827,-0.0491505705276828,
                0.00909049179030169,-0.0110849112574632,0.257885042514016])
        dt = 0.01

        s_dot = quadcopter.state_dot(quadcopter.state,dt,F,M)
        print "s_dot", s_dot

    def test_update(self):
        pos = (0,0,0)
        attitude = [0,0,0]
        quadcopter = Quadcopter(pos, attitude)
        M = np.array([[1,2,3]]).T
        dt = 0.01
        F = 0.0
        quadcopter.update(dt, F, M)

    def test_attitude(self):
        pos = (0,0,0)
        attitude = [0,0,np.pi/2]
        quadcopter = Quadcopter(pos, attitude)
        attitude = quadcopter.attitude()
        print "attitude", attitude

if __name__ == '__main__':
    unittest.main()
