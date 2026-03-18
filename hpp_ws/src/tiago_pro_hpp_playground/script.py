from pinocchio import neutral, SE3
from pyhpp.manipulation import (Device, urdf)
from pyhpp_viser import Viewer

robot = Device("tiago_pro")

urdf_filename = "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf"
srdf_filename = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/tiago_pro_dummy.srdf"

urdf.loadModel(robot, 0, "tiago_pro", "anchor", urdf_filename, srdf_filename, SE3.Identity())

# Get neutral configuration of robot
q = neutral(robot.model())

v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v(q)

# add ground as an obstacle
urdf_filename = "package://hpp_tutorial/urdf/ground.urdf"
srdf_filename = "package://hpp_tutorial/srdf/ground.srdf"
urdf.loadModel(robot, 0, "ground", "anchor", urdf_filename, srdf_filename, SE3.Identity())
v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v(q)

# adding an object to the scene
urdf_filename = "package://hpp_tutorial/urdf/box.urdf"
srdf_filename = "package://hpp_tutorial/srdf/box.srdf"
urdf.loadModel(robot, 0, "box", "freeflyer", urdf_filename, srdf_filename, SE3.Identity())
q = neutral(robot.model())

q[37:40] = [.6, -0.1, 0.025]
v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v(q)

# adding bounds to the object translation
robot.setJointBounds("box/root_joint", [-1.5, 1.5,
    -1.5, 1.5,
    -0.2, 1.5,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf")])