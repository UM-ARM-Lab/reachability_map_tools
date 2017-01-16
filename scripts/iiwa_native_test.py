#! /usr/bin/python

import time
import openravepy as rave
import IPython

import math
import numpy as np


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

if __name__ == "__main__":
    env = rave.Environment()  # create openrave environment

    env.SetViewer('qtcoin')  # attach viewer (optional)
    env.GetViewer().SetCamera([
        [ 0.,  0.,  1., -3.6],
        [-1.,  0.,  0.,  0.],
        [ 0., -1.,  0.,  1.1],
        [ 0.,  0.,  0.,  1.]
    ])

    # robot_single_arm = env.ReadRobotURI('/home/dmcconachie/Dropbox/catkin_ws/src/personal_robotics_lab/arm_ordata/data/robots/iiwa7.robot.xml')
    robot_single_arm = env.ReadRobotURI('robots/iiwa7.robot.xml')
    # robot_single_arm = env.ReadRobotURI('robots/iiwa14.robot.xml')
    env.Add(robot_single_arm, True)

    #robot_dual_arm = env.ReadRobotURI('iiwa14-dual.robot.xml')
    #env.Add(robot_dual_arm, True)

    # joint = range(6, 7)
    # val = np.ones(len(joint))
    # while True:
    #     with env:
    #         robot_orxml.SetActiveDOFs(joint)
    #         robot_orxml.SetActiveDOFValues(val)
    #         robot_orxml.GetController().SetDesired(robot_orxml.GetDOFValues())
    #
    #         robot_urdf.SetActiveDOFs(joint)
    #         robot_urdf.SetActiveDOFValues(val)
    #         robot_urdf.GetController().SetDesired(robot_urdf.GetDOFValues())
    #
    #     waitrobot(robot_orxml)
    #     waitrobot(robot_urdf)
    #
    #     IPython.embed()
    #     val = 1.0 - val

    #kinematic_reachability = rave.databases.kinematicreachability.ReachabilityModel(robot_single_arm, iktype=rave.IkParameterization.Type.Transform6D)
    #kinematic_reachability.load()

    # ikmodel = rave.databases.inversekinematics.InverseKinematicsModel(robot=robot_orxml, iktype=rave.IkParameterization.Type.Transform6D)
    #
    # if not ikmodel.load():
    #     ikmodel.autogenerate()  # autogenerate if one doesn't exist
    # lmodel = rave.databases.linkstatistics.LinkStatisticsModel(robot_orxml)
    # if not lmodel.load():
    #     lmodel.autogenerate()
    # lmodel.setRobotWeights()

    IPython.embed()
