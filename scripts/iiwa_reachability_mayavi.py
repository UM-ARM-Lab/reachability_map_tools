import IPython
import openravepy as rave

if __name__ == "__main__":
    showrobot = True
    contours = [0.01, 0.1, 0.2, 0.5, 0.8, 0.9, 0.99]
    opacity = None
    figureid = 1

    try:
        mlab = __import__('enthought.mayavi.mlab', fromlist=['mlab'])
    except ImportError:
        mlab = __import__('mayavi.mlab', fromlist=['mlab'])

    mlab.figure(figureid, fgcolor=(0, 0, 0), bgcolor=(1, 1, 1), size=(1024, 768))
    mlab.clf()

    reachability3d = loaddatahere #minimum(self._GetValue(self.reachability3d), 1.0)
    reachability3d[0, 0, 0] = 1  # have at least one point be at the maximum

    offset = array((0, 0, 0))
    src = mlab.pipeline.scalar_field(reachability3d)

    for i, c in enumerate(contours):
        mlab.pipeline.iso_surface(src, contours=[c], opacity=min(1, 0.7 * c if opacity is None else opacity[i]))
    # mlab.pipeline.volume(mlab.pipeline.scalar_field(reachability3d*100))

    if showrobot:
        env = rave.Environment()  # create openrave environment
        robot_dual_arm = env.ReadRobotURI('robots/iiwa14-dual.robot.xml')
        env.Add(robot_dual_arm, True)

        with robot_dual_arm:
            IPython.embed()
            # Tbase = robot_dual_arm
            # Tbaseinv = linalg.inv(Tbase)
            # self.robot.SetTransform(dot(Tbaseinv, self.robot.GetTransform()))
            # baseanchor = self.getOrderedArmJoints()[0].GetAnchor()
            # trimesh = self.env.Triangulate(self.robot)
            # v = self.pointscale[0] * (trimesh.vertices - tile(baseanchor, (len(trimesh.vertices), 1))) + \
            # self.pointscale[1]
            # mlab.triangular_mesh(v[:, 0] - offset[0], v[:, 1] - offset[1], v[:, 2] - offset[2], trimesh.indices, color=(0.5, 0.5, 0.5))
    mlab.show()

    IPython.embed()