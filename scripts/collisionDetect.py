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
        self.obs_5_2 = fcl.CollisionObject(fcl.Box(0.05, 0.05, 0.15),transform.Transform(transform.Quaternion(), [0.5, 0.1, 0.275]))
        self.obs_5_3 = fcl.CollisionObject(fcl.Box(0.05, 0.05, 0.15),transform.Transform(transform.Quaternion(), [0.5, -0.1, 0.275]))
        self.obs_5_4 = fcl.CollisionObject(fcl.Box(0.05, 0.25, 0.05),transform.Transform(transform.Quaternion(), [0.5, 0.0, 0.375]))
        """ Part 1 YouBot """
        self.youbot_Box = fcl.Box(0.7, 0.5, 0.6)
        """ Part 2 YouBot """
        self.youbot_base = fcl.Box(0.5701, 0.3570, 0.1060)
        self.youbot_arm_1 = fcl.Box(0.1697, 0.1699, 0.1060)
        self.youbot_arm_2 = fcl.Box(0.22, 0.0745, 0.0823)
        self.youbot_arm_3 = fcl.Box(0.1920, 0.0591, 0.0750)
        self.youbot_arm_4 = fcl.Box(0.2401, 0.0581, 0.0987)
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
            for bot_part in robot:
                ret, res = fcl.collide(obstacle, bot_part, fcl.CollisionRequest())
                if ret = False:
                    break
        return ret

    def _multiply_quaternion(self, q1, q2):
        w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
        x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
        y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
        z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
        return [w, x, y, z]


    def _create_youbot_object(self, config):
        if self.section == 1:
            self.youbot = fcl.CollisionObject(self.youbot_Box, transform.Transform(transform.Quaternion(), config))
        else:
            self.youbot = []
            self.yobot.append(fcl.CollisionObject(self.youbot_base, transform.Transform(transform.Quaternion(), [-0.0014, 0.0, 0.0956])))
            self.yobot.append(fcl.CollisionObject(self.youbot_arm_1, transform.Transform(transform.Quaternion(), [0.167, 0.0, 0.086])))

            quat_arm_1 = [math.cos(config[0]/2), 0, -math.sin(config[0]/2), 0]
            pos_arm_1 = [0.167+0.074*math.cos(config[0]), -0.0412, 0.086+0.074*math.sin(config[0])]
            self.yobot.append(fcl.CollisionObject(self.youbot_arm_2, transform.Transform(quat_arm_1, pos_arm_1)))

            quat_arm_2 = [math.cos(config[1]/2), 0, -math.sin(config[1]/2), 0]
            quat_arm_2 = self._multiply_quaternion(quat_arm_1, quat_arm_2)
            pos_arm_2 = pos_arm_1 + [0.0662*math.cos(config[1]), 0.0370 + 0.0412, 0.0662*math.sin(config[1])]
            self.yobot.append(fcl.CollisionObject(self.youbot_arm_3, transform.Transform(quat_arm_2, pos_arm_2)))

            quat_arm_3 = [math.cos(config[2]/2), 0, -math.sin(config[2]/2), 0]
            quat_arm_3 = self._multiply_quaternion(quat_arm_2, quat_arm_3)
            pos_arm_3 = pos_arm_2 + [0.0869*math.cos(config[2]), -0.0370 , 0.0869*math.sin(config[2])]
            self.yobot.append(fcl.CollisionObject(self.youbot_arm_4, transform.Transform(quat_arm_3, pos_arm_3)))

    def is_colliding(self, config):
        self._create_youbot_object(config)
        collide = False
        for obj in self.obs:
            if self._check_collision(obj, self.youbot):
                collide = True
        return collide
