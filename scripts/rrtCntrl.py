import numpy as np
from rrtPlanner import RRT

init_posn = np.array([0,0])
goal_posn = np.array([5,5])
myPlanner = RRT(init_posn,0.1,1)
myPlanner.generate_tree(goal_posn)
print myPlanner.path
