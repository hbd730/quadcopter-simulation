import numpy as np
from context import trajGen
import unittest

class TestGenLine(unittest.TestCase):
    time = 0
    output = []
    while time < 5:
        desired_state = trajGen.genLine(time)
        x,y,z = desired_state.pos
        output.append((x,y,z))
        time += 0.1
    with open('traj.test','w') as fp:
            fp.write('\n'.join('%s %s %s' % item for item in output))

if __name__ == '__main__':
    unittest.main()
