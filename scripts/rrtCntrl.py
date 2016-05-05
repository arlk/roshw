from __future__ import division
import numpy as np
from rrtPlanner import *
import matplotlib.pyplot as plt

init_posn = np.array([0,0])
goal_posn = np.array([5,5])
myPlanner = RRT(init_posn,0.05,1)
myPlanner.generate_tree(goal_posn, 0.15)
print myPlanner.path

plt.scatter(myPlanner.path[:,0],-myPlanner.path[:,1])
plt.show()
