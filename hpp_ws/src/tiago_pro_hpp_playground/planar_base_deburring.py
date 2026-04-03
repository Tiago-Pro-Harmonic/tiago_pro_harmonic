# ============================================================
# TIAGo Pro — HPP deburring demo (planar base)
# ============================================================
#
# Task   : approach a vertical plate then insert the gripper
#          (simulates a deburring / drilling operation)
# Robot  : TIAGo Pro with planar mobile base
# Object : vertical square plate (fixed in environment)
#
# Planning pipeline:
#   p1 : q_init → qpg  (approach, transition f_01)
#   p2 : qpg   → qg   (insertion, transition f_12)
#   p3 : qg    → qpg  (retraction, reverse of p2)
#
# Active DOFs: base (3) + right arm (7) + plate (7) = 17
# ============================================================

import numpy as np
from pinocchio import SE3, neutral
import pinocchio as pin
from pyhpp.manipulation import (
    Device,
    Graph,
    Problem,
    TransitionPlanner,
    urdf,
)
from pyhpp.core import RandomShortcut
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp_viser import Viewer


SRDF = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/tiago_pro.srdf"

# ── 1. Load TIAGo Pro (mobile planar base) ───────────────────────────────────
robot = Device("tiago_pro")
urdf.loadModel(
    robot, 0, "tiago_pro", "planar",
    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf",
    SRDF,
    SE3.Identity(),
)

# ── 2. Load ground (visual reference) ────────────────────────────────────────
urdf.loadModel(
    robot, 0, "ground", "anchor",
    "package://hpp_tutorial/urdf/ground.urdf",
    "package://hpp_tutorial/srdf/ground.srdf",
    SE3.Identity(),
)

# ── 3. Load plate (vertical wall — fixed in environment) ─────────────────────
PLATE_X = 1.20   # m in front of robot
PLATE_Y = -0.20  # m to the right (toward right arm)
PLATE_Z =  1.00  # m height

urdf.loadModel(
    robot, 0, "plate", "freeflyer",
    "/home/user/devel/hpp_ws/src/hpp_tutorial/urdf/square-plate.urdf",
    "",
    SE3.Identity(),
)

# ── Robot info ───────────────────────────────────────────────────────────────
model = robot.model()
print(f"\n{'='*65}")
print(f"  Full model (TIAGo Pro planar + ground + plate)")
print(f"{'='*65}")
print(f"  nq (config space dim)  : {model.nq}")
print(f"  nv (velocity space dim): {model.nv}")

def _idx(joint_name):
    return model.joints[model.getJointId(joint_name)].idx_q

_base_idx      = _idx("tiago_pro/root_joint")
_torso_idx     = _idx("tiago_pro/torso_lift_joint")
_left_arm_idx  = _idx("tiago_pro/arm_left_1_joint")
_right_arm_idx = _idx("tiago_pro/arm_right_1_joint")
_plate_idx     = _idx("plate/root_joint")

print(f"→ Base     : q[{_base_idx}:{_base_idx+4}]  (x, y, cos θ, sin θ)")
print(f"→ Torso    : q[{_torso_idx}]")
print(f"→ Left arm : q[{_left_arm_idx}:{_left_arm_idx+7}]")
print(f"→ Right arm: q[{_right_arm_idx}:{_right_arm_idx+7}]")
print(f"→ Plate    : q[{_plate_idx}:{_plate_idx+7}]  (x, y, z, qx, qy, qz, qw)")
print(f"{'='*65}\n")

# ── 4. Joint bounds ──────────────────────────────────────────────────────────
robot.setJointBounds("tiago_pro/root_joint", [
    -2.0, 2.0, -2.0, 2.0,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])
robot.setJointBounds("plate/root_joint", [
    PLATE_X - 0.01, PLATE_X + 0.01,
    PLATE_Y - 0.01, PLATE_Y + 0.01,
    PLATE_Z - 0.01, PLATE_Z + 0.01,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])

# ── 5. Add handle to the plate via Python ────────────────────────────────────
# Handle Z-axis points toward -X (toward the robot).
# approachingDirection=[0,0,1] in handle frame = approach along -X in world.
# Clearance = 0.1 m → pregrasp positions gripper 10 cm from handle.
R = np.array([[ 0, 0, 1],
              [ 0, 1, 0],
              [-1, 0, 0]])
T = np.array([0.02, 0, 0])
pose = SE3(rotation=R, translation=T)
robot.addHandle("plate/base_link", "plate/hole", pose, 0.1, 6 * [True])
robot.handles()["plate/hole"].approachingDirection = np.array([0, 0, 1])

# ── 6. Constraint graph ──────────────────────────────────────────────────────
problem = Problem(robot)
graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)

graph.maxIterations(40)
graph.errorThreshold(1e-3)

factory.setGrippers(["tiago_pro/gripper"])
factory.setObjects(["plate"], [["plate/hole"]], [[]])  # no surface: plate is fixed
factory.generate()

# ── 7. Lock passive joints ───────────────────────────────────────────────────
model = robot.model()
_cts = ComparisonTypes()
_cts[:] = [ComparisonType.EqualToZero]

def _lock(joint_name, value):
    j = model.joints[model.getJointId(joint_name)]
    if j.nq == 2 and j.nv == 1:
        val = np.array([np.cos(value), np.sin(value)])
    else:
        val = np.array([value])
    return LockedJoint(robot, joint_name, val, _cts)

print(f"{len(robot.geomModel().collisionPairs)} collision pairs active.")

LEFT_ARM_TUCK  = [0.2, -1.83, -0.47, -2.35, 0.0, -1.2, 0.0]
RIGHT_ARM_TUCK = [0.0,  0.0,   0.0,   0.0,  0.0,  0.0, 0.0]

locked = []
locked.append(_lock("tiago_pro/torso_lift_joint", 0.0))
for w in ["wheel_front_left_joint", "wheel_front_right_joint",
          "wheel_rear_left_joint",  "wheel_rear_right_joint"]:
    locked.append(_lock(f"tiago_pro/{w}", 0.0))
for i, val in enumerate(LEFT_ARM_TUCK):
    locked.append(_lock(f"tiago_pro/arm_left_{i+1}_joint", val))
for name in ["gripper_left_finger_joint",
             "gripper_left_inner_finger_left_joint",
             "gripper_left_fingertip_left_joint",
             "gripper_left_inner_finger_right_joint",
             "gripper_left_fingertip_right_joint",
             "gripper_left_outer_finger_right_joint",
             "gripper_right_finger_joint",
             "gripper_right_inner_finger_left_joint",
             "gripper_right_fingertip_left_joint",
             "gripper_right_inner_finger_right_joint",
             "gripper_right_fingertip_right_joint",
             "gripper_right_outer_finger_right_joint"]:
    locked.append(_lock(f"tiago_pro/{name}", 0.0))
locked.append(_lock("tiago_pro/head_1_joint", 0.0))
locked.append(_lock("tiago_pro/head_2_joint", 0.0))

graph.addNumericalConstraintsToGraph(locked)
print(f"{len(locked)} joints locked (torso + wheels + left arm + grippers + head).")

graph.initialize()

print(f"\nConstraint graph states:")
for name in graph.getStateNames():
    print(f"  '{name}'")
print(f"\nTransitions:")
for name in graph.getTransitionNames():
    print(f"  '{name}'")

# ── 8. Transitions ───────────────────────────────────────────────────────────
transition_approach = graph.getTransition("tiago_pro/gripper > plate/hole | f_01")
transition_insert   = graph.getTransition("tiago_pro/gripper > plate/hole | f_12")

# ── 9. Security margins ──────────────────────────────────────────────────────
# 5 cm margin between robot and plate during approach (f_01).
# On insertion (f_12), disable all robot-plate collision to allow penetration.
sm = SecurityMargins(problem, factory, ["tiago_pro", "plate"], robot)
sm.setSecurityMarginBetween("tiago_pro", "plate", 0.05)
sm.apply()

for jname in robot.getJointNames():
    if "tiago_pro" in jname:
        graph.setSecurityMarginForTransition(
            transition_insert, jname, "plate/root_joint", float("-inf")
        )
graph.initialize()
graph.initialize()

# ── 10. Initial configuration ────────────────────────────────────────────────
q_init = neutral(model).copy()
q_init[_left_arm_idx:_left_arm_idx+7]   = LEFT_ARM_TUCK
q_init[_right_arm_idx:_right_arm_idx+7] = RIGHT_ARM_TUCK
q_init[_plate_idx:_plate_idx+7] = [PLATE_X, PLATE_Y, PLATE_Z, 0., 0., 0., 1.]

# ── 11. Viewer ───────────────────────────────────────────────────────────────
v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v(q_init)

# ── 12. Validate q_init ──────────────────────────────────────────────────────
_geom  = robot.geomModel()
_data  = model.createData()
_gdata = _geom.createData()
pin.computeCollisions(model, _data, _geom, _gdata, q_init, True)
_collisions = [
    f"  {_geom.geometryObjects[_geom.collisionPairs[i].first].name}"
    f" vs {_geom.geometryObjects[_geom.collisionPairs[i].second].name}"
    for i, res in enumerate(_gdata.collisionResults) if res.isCollision()
]
if _collisions:
    print(f"Collisions in q_init ({len(_collisions)}):")
    for c in _collisions:
        print(c)
    raise SystemExit("Fix collisions before planning.")
print("No collision in q_init. ✓")

# ── 13. Generate waypoint configurations ─────────────────────────────────────
shooter = problem.configurationShooter()

res, qpg, err = graph.generateTargetConfig(transition_approach, q_init, q_init)
print(f"qpg: res={res}, err={err:.2e}")
if not res:
    raise SystemExit("Failed to generate qpg.")

res, qg, err = graph.generateTargetConfig(transition_insert, qpg, qpg)
print(f"qg:  res={res}, err={err:.2e}")
if not res:
    raise SystemExit("Failed to generate qg.")

v(qpg)
v(qg)

# ── 14. Plan p1 : q_init → qpg (approach) ────────────────────────────────────
problem.constraintGraph(graph)
planner = TransitionPlanner(problem)
planner.maxIterations(1000)

planner.setTransition(transition_approach)
q_goal = np.zeros((1, robot.configSize()), order='F')
q_goal[0, :] = qpg
p1 = planner.planPath(q_init, q_goal, True)
print("p1 found (q_init → qpg).")

try:
    optimizer = RandomShortcut(problem)
    p1_opt = optimizer.optimize(p1)
    tr, tr_opt = p1.timeRange(), p1_opt.timeRange()
    print(f"p1 optimised: {tr_opt.second - tr_opt.first:.2f} s "
          f"(was {tr.second - tr.first:.2f} s)")
    v.loadPath(p1_opt)
except Exception as e:
    print(f"p1 optimisation failed: {e}")
    v.loadPath(p1)

# ── 15. Plan p2 : qpg → qg (insertion) ───────────────────────────────────────
planner.setTransition(transition_insert)
q_goal[0, :] = qg
p2 = planner.planPath(qpg, q_goal, True)
print("p2 found (qpg → qg).")
v.loadPath(p2)

# ── 16. p3 : retraction (reverse of p2) ──────────────────────────────────────
p3 = p2.reverse()
print("p3 ready (retraction).")
v.loadPath(p3)
