from context import model
from control import pid_controller as pid
from control import lqr_controller as lqr
import trajGen
from model.quadcopter import Quadcopter
import numpy as np
import unittest

class TestController(unittest.TestCase):
    def test_pid(self):
        pos = (0,0,0)
        attitude = [0,0,np.pi/6]
        quad = Quadcopter(pos, attitude)
        time = 0.0
        des_state = trajGen.genLine(time)
        F, M = pid.run(quad, des_state)
        print "des_state", des_state
        print "F", F
        print "M", M

    def test_lqr(self):
        pos = (0,0,0)
        attitude = [0,0,np.pi/6]
        quad = Quadcopter(pos, attitude)
        time = 0.0
        des_state = trajGen.genLine(time)
        F, M = lqr.run(quad, des_state)
        print "des_state", des_state
        print "F", F
        print "M", M
        
if __name__ == '__main__':
    unittest.main()
