from __future__ import division
from fcl import fcl, transform

class Obstacles(object):
    def __init__(self, part_num):
        """ Room Parameters """
        self.room_dimensions = [7.50, 7.50]
        self.room_offset = [-1.25, -1.25]
        """ Walls """
        self.wall_1 = fcl.CollisionObject(fcl.Box(8.0, 0.5, 0.5),transform.Transform(transform.Quaternion(), [2.50, -1.25, 0.25]))
        self.wall_2 = fcl.CollisionObject(fcl.Box(0.5, 8.0, 0.5),transform.Transform(transform.Quaternion(), [6.25, 2.50, 0.25]))
        self.wall_3 = fcl.CollisionObject(fcl.Box(8.0, 0.5, 0.5),transform.Transform(transform.Quaternion(), [2.50, 6.25, 0.25]))
        self.wall_4 = fcl.CollisionObject(fcl.Box(0.5, 8.0, 0.5),transform.Transform(transform.Quaternion(), [-1.25, 2.50, 0.25]))
        """ Obstacles """
        self.obs_1 = fcl.CollisionObject(fcl.Box(2.0, 2.0, 1.0),transform.Transform(transform.Quaternion(), [2.50, 2.50, 0.50]))
        self.obs_2 = fcl.CollisionObject(fcl.Box(0.75, 0.75, 0.75),transform.Transform(transform.Quaternion(), [5.0, 1.0, 0.375]))
        self.obs_3 = fcl.CollisionObject(fcl.Box(0.75, 0.75, 0.75),transform.Transform(transform.Quaternion(), [3.0, 5.0, 0.375]))
        self.obs_4 = fcl.CollisionObject(fcl.Box(0.5, 0.5, 0.5),transform.Transform(transform.Quaternion(), [0.0, 3.0, 0.25]))
        """ Table Obstacle """
        self.obs_5_1 = fcl.CollisionObject(fcl.Box(0.4, 0.4, 0.2),transform.Transform(transform.Quaternion(), [0.6, 0.0, 0.1]))
        self.obs_5_2 = fcl.CollisionObject(fcl.Box(0.5, 0.5, 0.15),transform.Transform(transform.Quaternion(), [0.5, 0.1, 0.275]))
        self.obs_5_3 = fcl.CollisionObject(fcl.Box(0.5, 0.5, 0.15),transform.Transform(transform.Quaternion(), [0.5, -0.1, 0.275]))
        self.obs_5_4 = fcl.CollisionObject(fcl.Box(0.05, 0.25, 0.05),transform.Transform(transform.Quaternion(), [0.5, 0.0, 0.375]))
        """ Part 1 YouBot """
        self.youbot_Box = fcl.Box(0.5701, 0.357, 0.6)
        """ YouBot """
        self.section = part_num
        self.youbot = fcl.CollisionObject(self.youbot_Box,transform.Transform(transform.Quaternion(), [0.0, 0.0, 0.0]))
        """ Obstacles as a list """
        self.obs = [self.wall_1, self.wall_2, self.wall_3, self.wall_4,
                        self.obs_1, self.obs_2, self.obs_3, self.obs_4,
                        self.obs_5_1, self.obs_5_2, self.obs_5_3, self.obs_5_4]
        self.res = fcl.CollisionResult()

    def _check_collision(self, obstacle, robot):
        if self.section == 1:
            ret, res = fcl.collide(obstacle, robot, fcl.CollisionRequest())
        else:
            ret = False # Section 2
        return ret

    def _create_youbot_object(self, config):
        if self.section == 1:
            self.youbot = fcl.CollisionObject(self.youbot_Box,transform.Transform(transform.Quaternion(), config))
        else:
            pass # Section 2

    def is_colliding(self, config):
        self._create_youbot_object(config)
        collide = False
        for obj in self.obs:
            if self._check_collision(obj, self.youbot):
                collide = True
        return collide
