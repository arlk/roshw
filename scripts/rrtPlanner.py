from __future__ import division
from collisionDetect import Obstacles
import numpy as np

from pygame.locals import *
import pygame

class Graph(object):
    def __init__(self, vals):
        self.degree = [1]
        self.nodes = [vals]
        self.adjacency = [-1]

    def _update_degree(self, neighbor):
        self.degree.append(1)
        self.degree[neighbor] += 1

    def _update_adjacency(self, neighbor):
        self.adjacency.append(neighbor)

    def _update_nodes(self,vals):
        self.nodes.append(vals)

    def add_node(self, vals, neighbor):
        self._update_degree(neighbor)
        self._update_adjacency(neighbor)
        self._update_nodes(vals)

class RRT(object):
    def __init__(self, init_config, step_dist, part_num):
        self.init = init_config
        self.tree = Graph(init_config)
        self.epsilon = step_dist
        self.section = part_num
        self.world = Obstacles(part_num)
        self.path = None
        self.cntrl = [np.zeros(2)]

    def _distance(self, c1, c2):
        if self.section == 1:
            return np.linalg.norm(c1-c2)
        else: # ARM configuration space
            c1 = np.asarray(c1)
            c2 = np.asarray(c2)
            dist3 = np.minimum(np.absolute(c1-c2),2*np.pi-np.absolute(c1-c2))
            w = np.array([3,2,1])
            return np.dot(w,dist3)

    def _generate_random_node(self):
        if self.section == 1:
            return np.random.rand(2)*self.world.room_dimensions + self.world.room_offset
        else:
            jointRangeDeg = np.array([155,297,205])
            jointRangeOffsetDeg = np.array([12.5,-2.5,0.0])-jointRangeDeg/2
            return np.random.rand(3)*(jointRangeDeg*(np.pi/180))-(jointRangeOffsetDeg*(np.pi/180))

    def _nearest_neighbor(self, rand):
        nn = self.tree.nodes[0]
        nn_arg = 0
        for i,n in enumerate(self.tree.nodes):
            if self._distance(n, rand) < self._distance(nn, rand):
                nn = n
                nn_arg = i
        return nn, nn_arg

    def _step_from_to(self, c1, c2):
        if self.section == 1:
            if self._distance(c1, c2) < self.epsilon:
                return c2, (c2-c1)/np.linalg.norm(c2-c1)
            else:
                return c1 + self.epsilon*(c2-c1)/np.linalg.norm(c2-c1), self.epsilon*(c2-c1)/np.linalg.norm(c2-c1)
        else:                           #  ^^^  Check c2-c1
            if self._distance(c1,c2) < self.epsilon:
                return c2, torusVect(c1,c2)/distance(c1,c2)
            else:
                return c1 + self.epsilon*torusVect(c1,c2)/distance(c1,c2), self.epsilon*torusVect(c1,c2)/distance(c1,c2)
        def torusVect(c1,c2):
            c1 = np.array(c1)
            c2 = np.array(c2)
            return np.minimum(np.absolute(c1-c2),2*np.pi-np.absolute(c1-c2))

    def _find_shortest_path(self):
        k = len(self.tree.adjacency)-1
        short_path = [k]
        print "k: ", short_path
        while short_path[-1] is not -1:
            short_path.append(self.tree.adjacency[k])
            k = self.tree.adjacency[k]
        short_path.reverse()
        short_path.pop(0)
        nodes_array = np.asarray(self.tree.nodes)
        self.path = nodes_array[short_path]
        cntrl_array = np.asarray(self.cntrl)
        self.cntrl = cntrl_array[short_path]

    def generate_tree(self, goal, clearance):
        #constants
        Xi = 0.0
        Yi = 0.0
        Xg = 5.0
        Yg = 5.0
        XDIM = 8.0
        YDIM = 8.0
        RESOLUTION = 0.01
        XDIMPIX = int (XDIM / RESOLUTION)
        YDIMPIX = int (YDIM / RESOLUTION)
        WINSIZE = [XDIMPIX, YDIMPIX]
        EPSILON = 0.1 / RESOLUTION
        pygame.init()
        screen = pygame.display.set_mode(WINSIZE)
        pygame.display.set_caption('RRT     Jones/Lakshmanan     ECE550 F16')
        white = 255, 240, 200
        black = 20, 20, 40
        green = 10, 250, 10
        red = 250, 10, 10
        blue = 10, 10, 250
        screen.fill(black)
        new_node = self.init
        while self._distance(new_node, goal) > clearance:
            rand = self._generate_random_node()
            if not self.world.is_colliding([rand[0], rand[1], 0]):
                nn, nn_arg = self._nearest_neighbor(rand)
                new_node, u = self._step_from_to(nn, rand)

                pygame.draw.line(screen,white,list((nn-self.world.room_offset)*100.0),list((new_node-self.world.room_offset)*100.0))
                pygame.draw.circle(screen, green, map(int,list((np.array([0,0])-self.world.room_offset)*100)),5, 5)
                pygame.draw.circle(screen, red, map(int,list((np.array([5,5])-self.world.room_offset)*100)),5, 5)
                pygame.draw.line(screen,blue,list((rand-self.world.room_offset)*100.0),list((rand-self.world.room_offset)*100.0))
                pygame.display.update()

                if not self.world.is_colliding([new_node[0], new_node[1], 0]):
                    self.tree.add_node(new_node, nn_arg)
                    self.cntrl.append(u)
        self._find_shortest_path()
        self.path = np.append(self.path, [goal], axis = 0)
