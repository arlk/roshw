from __future__ import division
import numpy as np

class obstacles(object):
    def __init__(self):
        self.wall = np.array([[-1,6],[-1,-1],[6,-1],[6,6]])
        self.obsOrigin = np.array([[2.5,2.5],[5,1],[3,5],[0,3]])
        self.width = np.array([1,0.75,0.5,0.25])
        self.vertices = []

    def calcVertices(self):
        clockwise = np.array([[-1, -1],[-1, 1],[1, 1],[1, -1]])
        for i,org in enumerate(self.obsOrigin):
            sides = org + clockwise*self.width[i]/2.
            self.vertices.append(sides)
        self.vertices.append(self.wall)
        self.vertices = np.asarray(self.vertices)
