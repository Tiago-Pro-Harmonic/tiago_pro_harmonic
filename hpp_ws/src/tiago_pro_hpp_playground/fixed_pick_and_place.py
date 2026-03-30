# ============================================================
# TIAGo Pro — HPP pick-and-place demo
# ============================================================
#
# Goal: plan a trajectory for TIAGo Pro's right arm to pick
# a box from a table and place it at a different position.
#
# HPP workflow:
#   1. Load all models into a single Device
#   2. Build a constraint graph (states + transitions)
#   3. Define init/goal configurations
#   4. Run the manipulation planner
#
# Scene geometry:
#   - Robot: TIAGo Pro at origin, fixed base ("anchor" in HPP)
#   - Table: internal URDF places it at x=0.8m; offset by
#            TABLE_OFFSET_X and TABLE_OFFSET_Z at load time.
#   - Box:   5cm cube placed at the near edge of the table so
#            the gripper can approach from the front without
#            colliding with the table top.
# ============================================================

from pyhpp.manipulation import Device, urdf
from pyhpp_viser import Viewer
import numpy as np
import pinocchio as pin
from pyhpp.manipulation import Graph, Problem, ManipulationPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint


# ── 1. Load TIAGo Pro ────────────────────────────────────────────────────────
# "anchor" = fixed base. The custom SRDF only declares the right-arm gripper.
robot = Device("tiago_pro")
urdf.loadModel(
    robot, 0, "tiago_pro", "anchor",
    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf",
    "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/tiago_pro_dummy.srdf",
    pin.SE3.Identity(),
)

# ── 2. Load table (fixed obstacle) ───────────────────────────────────────────
# The table URDF places the body at x=0.8m internally.
# TABLE_OFFSET_X shifts it toward the robot; TABLE_OFFSET_Z lowers it.
# Result: table center at x = 0.8 + TABLE_OFFSET_X, surface at z = 0.730 + TABLE_OFFSET_Z.
TABLE_OFFSET_X =  0.2   # shift toward robot (m)
TABLE_OFFSET_Z = -0.1   # lower the table (m)
urdf.loadModel(
    robot, 0, "table", "anchor",
    "package://hpp_tutorial/urdf/table.urdf",
    "package://hpp_tutorial/srdf/table.srdf",
    pin.SE3(np.eye(3), np.array([TABLE_OFFSET_X, 0, TABLE_OFFSET_Z])),
)

# ── 3. Load ground (visual reference) ────────────────────────────────────────
# 2×2 m slab at z=0. Used as a visual reference only; contact surface is the table.
urdf.loadModel(
    robot, 0, "ground", "anchor",
    "package://hpp_tutorial/urdf/ground.urdf",
    "package://hpp_tutorial/srdf/ground.srdf",
    pin.SE3.Identity(),
)

# ── 4. Load box (manipulable object) ─────────────────────────────────────────
# "freeflyer" = 6-DOF floating joint (x, y, z, qx, qy, qz, qw) = 7 values in q.
# box.srdf defines:
#   - handle  : grasp frame (front approach along +x, 90° rotation around Y)
#   - surface : bottom face of the box (contact with table)
urdf.loadModel(
    robot, 0, "box", "freeflyer",
    "package://hpp_tutorial/urdf/box.urdf",
    "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/box.srdf",
    pin.SE3.Identity(),
)

# ── Robot info ────────────────────────────────────────────────────────────────
model = robot.model()
print(f"\n{'='*65}")
print(f"  Full model (TIAGo Pro + table + ground + box)")
print(f"{'='*65}")
print(f"  nq (config space dim)  : {model.nq}")
print(f"  nv (velocity space dim): {model.nv}")
print(f"  Number of joints       : {model.njoints}")
print(f"\n  Joint index map (joint name → idx_q):")
print(f"  {'id':>3}  {'name':<45}  {'idx_q':>5}  {'nq':>3}")
print(f"  {'-'*62}")
for jid in range(model.njoints):
    j = model.joints[jid]
    name = model.names[jid]
    if j.nq > 0:
        print(f"  {jid:3d}  {name:<45}  {j.idx_q:5d}  {j.nq:3d}")
print(f"{'='*65}\n")

_box_jid = model.getJointId("box/root_joint")
_box_idx = model.joints[_box_jid].idx_q
print(f"→ Box freeflyer: q[{_box_idx}:{_box_idx+7}]  (x, y, z, qx, qy, qz, qw)")

# ── 5. Box joint bounds ───────────────────────────────────────────────────────
# Required so the planner can sample valid box configurations.
robot.setJointBounds("box/root_joint", [
    -1.5,  1.5,                      # x
    -1.5,  1.5,                      # y
    -0.2,  1.5,                      # z
    -float("Inf"), float("Inf"),     # quaternion (unbounded)
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])

# ── 6. Disable adjacent collision pairs ──────────────────────────────────────
# Pinocchio generates collision pairs between all geometry objects.
# Links connected by a joint (parent-child or grandparent-child) produce false
# positives → remove pairs whose joints share an ancestor up to depth 2.
_geom = robot.geomModel()
_pairs_to_remove = []
for pair_id in range(len(_geom.collisionPairs)):
    cp = _geom.collisionPairs[pair_id]
    j1 = _geom.geometryObjects[cp.first].parentJoint
    j2 = _geom.geometryObjects[cp.second].parentJoint
    ancestors_j1 = {j1, model.parents[j1], model.parents[model.parents[j1]]}
    ancestors_j2 = {j2, model.parents[j2], model.parents[model.parents[j2]]}
    if ancestors_j1 & ancestors_j2:
        _pairs_to_remove.append(pair_id)
for pair_id in reversed(_pairs_to_remove):
    _geom.removeCollisionPair(_geom.collisionPairs[pair_id])
print(f"{len(_pairs_to_remove)} adjacent pairs disabled, "
      f"{len(_geom.collisionPairs)} pairs remaining.")

# Arm tuck configurations (used both for locking and for q_init/q_goal)
LEFT_ARM_TUCK  = [0.2, -1.83, -0.47, -2.35,  0.0, -1.2, 0.0]
RIGHT_ARM_TUCK = [-0.2, -1.83,  0.47, -2.35,  0.0, -1.2, 0.0]

# ── 7. Constraint graph ───────────────────────────────────────────────────────
# The constraint graph encodes all manipulation states and transitions:
#
#   State "free"    : box resting on the table, arm free
#   State "grasped" : arm holding the box (gripper aligned with handle)
#   Transitions     : grasp / place + pregrasp / preplace waypoints
#
# The factory auto-generates the graph from the gripper/object/contact spec.
problem = Problem(robot)
graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)

graph.maxIterations(40)
graph.errorThreshold(1e-3)

factory.setGrippers(["tiago_pro/gripper"])
factory.setObjects(["box"], [["box/handle"]], [["box/surface"]])
factory.environmentContacts(["table/pancake_table_table_top"])

factory.generate()

# ── Lock passive joints ───────────────────────────────────────────────────────
# Without locking, the planner explores finger/head/left-arm configurations too,
# inflating the search space from ~44 to ~15 active DOFs.
# Active DOFs after locking: right arm (7) + torso (1) + box (7) = 15.
_cts = ComparisonTypes()
_cts[:] = [ComparisonType.EqualToZero]

def _lock(joint_name, value):
    return LockedJoint(robot, joint_name, np.array([value]), _cts)

locked = []

# Left arm (locked at tuck pose)
for i, val in enumerate(LEFT_ARM_TUCK):
    locked.append(_lock(f"tiago_pro/arm_left_{i+1}_joint", val))

# Left gripper fingers
for name in ["gripper_left_finger_joint",
             "gripper_left_inner_finger_left_joint",
             "gripper_left_fingertip_left_joint",
             "gripper_left_inner_finger_right_joint",
             "gripper_left_fingertip_right_joint",
             "gripper_left_outer_finger_right_joint"]:
    locked.append(_lock(f"tiago_pro/{name}", 0.0))

# Right gripper fingers (open = 0)
for name in ["gripper_right_finger_joint",
             "gripper_right_inner_finger_left_joint",
             "gripper_right_fingertip_left_joint",
             "gripper_right_inner_finger_right_joint",
             "gripper_right_fingertip_right_joint",
             "gripper_right_outer_finger_right_joint"]:
    locked.append(_lock(f"tiago_pro/{name}", 0.0))

# Head
locked.append(_lock("tiago_pro/head_1_joint", 0.0))
locked.append(_lock("tiago_pro/head_2_joint", 0.0))

graph.addNumericalConstraintsToGraph(locked)
print(f"{len(locked)} joints locked (left arm + grippers + head).")

graph.initialize()

print(f"\nConstraint graph states:")
for name in graph.getStateNames():
    print(f"  '{name}'")
print(f"\nTransitions:")
for name in graph.getTransitionNames():
    print(f"  '{name}'")

# ── 8. Initial and goal configurations ───────────────────────────────────────
# Table geometry (from table.urdf + offsets):
#   table surface z = 0.730 + TABLE_OFFSET_Z
#   table x range  = [0.8 + TABLE_OFFSET_X - 0.3,  0.8 + TABLE_OFFSET_X + 0.3]
#
# The box is placed at the near edge of the table so the gripper body
# stays outside the table top during the front approach.
TABLE_Z         = 0.730 + TABLE_OFFSET_Z
BOX_HALF        = 0.025
BOX_Z           = TABLE_Z + BOX_HALF
TABLE_NEAR_EDGE = 0.8 + TABLE_OFFSET_X - 0.3
BOX_X           = TABLE_NEAR_EDGE + BOX_HALF + 0.02   # 2 cm inside the near edge

q_init = pin.neutral(robot.model()).copy()
q_init[9:16]                = LEFT_ARM_TUCK
q_init[22:29]               = RIGHT_ARM_TUCK
q_init[_box_idx:_box_idx+7] = [BOX_X, -0.15, BOX_Z, 0., 0., 0., 1.]  # box on the left

q_goal = pin.neutral(robot.model()).copy()
q_goal[9:16]                = LEFT_ARM_TUCK
q_goal[22:29]               = RIGHT_ARM_TUCK
q_goal[_box_idx:_box_idx+7] = [BOX_X,  0.15, BOX_Z, 0., 0., 0., 1.]  # box on the right

# ── 9. Viewer + q_init validation ────────────────────────────────────────────
v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v(q_init)

# Collision check
_data  = robot.model().createData()
_gdata = robot.geomModel().createData()
pin.computeCollisions(robot.model(), _data, robot.geomModel(), _gdata, q_init, True)
_collisions = [
    f"  {_geom.geometryObjects[_geom.collisionPairs[i].first].name}"
    f" vs {_geom.geometryObjects[_geom.collisionPairs[i].second].name}"
    for i, res in enumerate(_gdata.collisionResults) if res.isCollision()
]
if _collisions:
    print(f"\nCollisions detected in q_init ({len(_collisions)}) — fix before planning:")
    for c in _collisions:
        print(c)
    raise SystemExit("Fix collisions before running the planner.")
else:
    print("No collision in q_init. ✓")

# Constraint graph state check — q_init must be classified as 'free'
print("\nConstraint error per state (q_init):")
for name in graph.getStateNames():
    try:
        state = graph.getState(name)
        err   = graph.getConfigErrorForState(state, q_init)
        label = "✓" if err[1] else "✗"
        print(f"  {label} '{name}': valid={err[1]}, err_max={max(abs(err[0])):.2e}")
    except Exception as e:
        print(f"  ? '{name}': {e}")

# ── 10. Grasp IK test ─────────────────────────────────────────────────────────
# generateTargetConfig finds a configuration where the gripper grasps the handle.
# Returns (success, config, residual).
grasp_edge = graph.getTransition("tiago_pro/gripper > box/handle | f")
success, q_grasp, residual = graph.generateTargetConfig(grasp_edge, q_init, q_init)
print(f"\nGrasp IK: success={success}, residual={residual:.2e}")
if success:
    print(f"  Torso      : {q_grasp[8]:.3f}")
    print(f"  Left arm   : {np.array(q_grasp[9:16]).round(3)}")
    print(f"  Right arm  : {np.array(q_grasp[22:29]).round(3)}")
    v(q_grasp)
else:
    print("  IK failed — gripper in collision or joint limits exceeded.")
    v(q_init)

# ── 11. Solve ─────────────────────────────────────────────────────────────────
problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
problem.constraintGraph(graph)

manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(10000)

try:
    p = manipulationPlanner.solve()
    print("\nPath found!")
    v.loadPath(p)
except RuntimeError as e:
    print(f"\nPlanner failed: {e}")
    v(q_init)
