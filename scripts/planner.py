from __future__ import division
import numpy as np 
import math

class potentialPlanner(object):
    def __init__(self):
        self.minDistance = np.inf
        self.perpVec = np.zeros(2)
        self.vertex = np.zeros(2)
        self.F = np.zeros(2)

    def _calcRepl(self, gain):
        self.F = gain*self.perpVec/(self.minDistance+1e-3)

    def _calcAttr(self, gain):
        self.F = gain*self.perpVec*self.minDistance
        # print "Force:", self.F

    def _polgonRepulsion(self, robot, obs, wall):
        repulsionGain = -0.3
        self.minDistance = np.inf
        self.perpVec = np.zeros(2)
        self.vertex = np.zeros(2)

        for pt in robot:
            for i in range(obs.shape[0]):
                v_parallel = obs[i] - obs[i-1]
                v_perp = np.flipud(v_parallel)
                v_perp[1] = -v_perp[1]
                v_perp /= np.linalg.norm(v_perp)
                r0 = obs[i-1] - pt
                r1 = obs[i] - pt
                r0_mag = np.linalg.norm(r0)
                r1_mag = np.linalg.norm(r1)

                distance_r0 = np.dot(v_perp,r0)
                distance_r1 = np.dot(v_perp,r1)
                ang0 = math.acos(np.dot(-r0/r0_mag, -v_parallel/np.linalg.norm(v_parallel)))
                ang1 = math.acos(np.dot(-r1/r1_mag, v_parallel/np.linalg.norm(v_parallel)))

                if wall != True:
                    if ang0 > math.pi/2:
                        distance = r0_mag
                        v_perp = r0/r0_mag
                    elif ang1 > math.pi/2::
                        distance = r1_mag
                        v_perp = r1/r1_mag 
                    else:
                        distance = distance_r0
                else:
                    distance = distance_r0


                if distance_r0==0 or distance_r1==0:
                    if r0_mag < r1_mag:
                        distance = r0_mag
                    else:
                        distance = r1_mag

                distance = math.fabs(distance)

                if distance < self.minDistance:
                    self.minDistance = distance 
                    self.perpVec = v_perp
                    self.vertex = pt

        # print self.minDistance, obs 

        self._calcRepl(repulsionGain)

    def _jacobian(self, angle, posn):
        localVertex = self.vertex - posn
        Ji = np.zeros((3,2))
        Ji[0,0]=1
        Ji[1,1]=1
        # Ji[2,0] = -self.vertex[0]*math.sin(angle)-self.vertex[1]*math.cos(angle)
        # Ji[2,1] = self.vertex[0]*math.cos(angle)-self.vertex[1]*math.sin(angle)
        Ji[2,0] = -localVertex[1]
        Ji[2,1] = localVertex[0]
        return np.dot(Ji,self.F)

    def _goalAttraction(self, robotVer, goal):
        attractionGain = -.25 # -2
        dist = np.linalg.norm(robotVer - goal, axis = 1)
        i = np.argmin(dist)
        self.minDistance = dist[i]
        self.perpVec = (robotVer[i] - goal)/self.minDistance 
        self.vertex = robotVer[i]

        self._calcAttr(attractionGain)

    def planTraj(self, robotPts, angle, posn, obstacles, goal):
        U = np.zeros(3)

        for j,obs in enumerate(obstacles):
            if j == 4:
                wallbool = 1
            else:
                wallbool = 0
            self._polgonRepulsion(robotPts, obs, wallbool)
            U += self._jacobian(angle,posn)

        self._goalAttraction(robotPts,goal)
        U += self._jacobian(angle,posn)
        return U





    