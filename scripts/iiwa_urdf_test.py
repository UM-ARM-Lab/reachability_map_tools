#! /usr/bin/python

import time
import openravepy as rave
import or_plugin
import IPython
import numpy as np

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

if __name__ == "__main__":
    env = rave.Environment()  # create openrave environment
    urdf_module = rave.RaveCreateModule(env, 'urdf')

    # attach viewer (optional)
    env.SetViewer('qtcoin')
    # env.SetViewer('InteractiveMarker')
    # env.SetViewer('RViz')
    # env.GetViewer().SetCamera([
    #     [ 0.,  0.,  1., -3.6],
    #     [-1.,  0.,  0.,  0.],
    #     [ 0., -1.,  0.,  1.1],
    #     [ 0.,  0.,  0.,  1.]
    # ])

    with env:
        name = urdf_module.SendCommand('LoadURI /home/dmcconachie/Dropbox/catkin_ws/src/personal_robotics_lab/arm_ordata/data/robots/iiwa7.urdf')
        robot_urdf_kinbody = env.GetKinBody(name)

    robot_single_arm = env.ReadRobotURI('/home/dmcconachie/Dropbox/catkin_ws/src/personal_robotics_lab/arm_ordata/data/robots/iiwa7.robot.xml')
    env.Add(robot_single_arm, True)

    # while True:
        # pass

    np.set_printoptions(precision=5)

    for i in range(0,8):
        print '\n\nLink Num: ', i

        link_name = 'iiwa_link_' + str(i)
        print 'URDF:\n', np.round(robot_urdf_kinbody.GetLink(link_name).GetTransform(), 5)
        print 'Native:\n', np.round(robot_single_arm.GetLink(link_name).GetTransform(), 5)

    IPython.embed()
