from __future__ import division
from collisionDetect import Obstacles
import numpy as np

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
        self.tree = Graph(init)
        self.epsilon = step_dist
        self.section = part_num
        self.world = Obstacles(part_num)
        self.path = None

    def _distance(c1, c2):
        if self.section == 1:
            return np.linalg.norm(c1-c2)
        else
            return None #To do

    def _generate_random_node():
        if self.section == 1:
            return np.random.rand(2)*self.world.room_dimensions + self.world.room_offset
        else:
            return None #To do

    def _nearest_neighbor(rand):
        nn = self.world.nodes[0]
        for n in self.world.nodes:
            if distance(n, rand) < distance(nn, rand):
                nn = n
        return nn

    def _step_from_to(c1, c2):
        if self.section == 1:
            if _distance(c1, c2) < self.epsilon:
                return c2
            else:
                return c1 + self.epsilon*c2
        else
            return None #To do

    def _find_shortest_path():
        k = len(self.tree.adjacency)-1
        short_path = [k]
        while short_path[-1] is not -1:
            short_path.append(self.tree.adjacency[k])
            k = self.tree.adjacency[k]
        short_path.reverse()
        nodes_array = np.asarray(self.tree.nodes)
        self.path = nodes_array[short_path]

    def generate_tree(goal):
        new_node = self.init
        while _distance(new_node, goal) > self.epsilon:
            rand = _generate_random_node()
            nn, nn_arg = _nearest_neighbor(rand)
            new_node = _step_from_to(nn, rand)
            self.tree.add_node(new_node, nn_arg)
        _find_shortest_path(goal)
        self.path = np.append(self.path, [goal], axis = 0)
