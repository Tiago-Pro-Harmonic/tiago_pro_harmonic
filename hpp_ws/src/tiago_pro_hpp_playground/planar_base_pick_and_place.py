# ============================================================
# TIAGo Pro — HPP pick-and-place with mobile planar base (v2)
# ============================================================
#
# Same as planar_base_pick_and_place.py but uses tiago_pro.srdf
# instead of tiago_pro_dummy.srdf.
#
# tiago_pro.srdf adds a proper <gripper name="gripper"> definition
# for the right arm with a 15 cm TCP offset along arm_right_tool_link:
#   <position xyz="0 0 0.15" wxyz="1 0 0 0" />
#
# Active DOFs after locking:
#   base (3: x, y, θ) + right arm (7) + box (7) = 17
# ============================================================

from pyhpp.manipulation import Device, urdf
from pyhpp_viser import Viewer
import numpy as np
import pinocchio as pin
from pyhpp.manipulation import Graph, Problem, ManipulationPlanner
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut


SRDF = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/tiago_pro.srdf"

# ── 1. Load TIAGo Pro (mobile base) ──────────────────────────────────────────
robot = Device("tiago_pro")
urdf.loadModel(
    robot, 0, "tiago_pro", "planar",
    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf",
    SRDF,
    pin.SE3.Identity(),
)

# ── 2. Load table (fixed obstacle) ───────────────────────────────────────────
TABLE_OFFSET_X =  0.5   # push table further from robot (m)
TABLE_OFFSET_Z = -0.1   # lower the table (m)
urdf.loadModel(
    robot, 0, "table", "anchor",
    "package://hpp_tutorial/urdf/table.urdf",
    "package://hpp_tutorial/srdf/table.srdf",
    pin.SE3(np.eye(3), np.array([TABLE_OFFSET_X, 0, TABLE_OFFSET_Z])),
)

# ── 3. Load ground (visual reference) ────────────────────────────────────────
urdf.loadModel(
    robot, 0, "ground", "anchor",
    "package://hpp_tutorial/urdf/ground.urdf",
    "package://hpp_tutorial/srdf/ground.srdf",
    pin.SE3.Identity(),
)

# ── 4. Load box (manipulable object) ─────────────────────────────────────────
urdf.loadModel(
    robot, 0, "box", "freeflyer",
    "package://hpp_tutorial/urdf/box.urdf",
    "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/box.srdf",
    pin.SE3.Identity(),
)

# ── Robot info ────────────────────────────────────────────────────────────────
model = robot.model()
print(f"\n{'='*65}")
print(f"  Full model (TIAGo Pro planar + table + ground + box)")
print(f"{'='*65}")
print(f"  nq (config space dim)  : {model.nq}")
print(f"  nv (velocity space dim): {model.nv}")
print(f"  Number of joints       : {model.njoints}")
print(f"\n  Joint index map (joint name → idx_q):")
print(f"  {'id':>3}  {'name':<50}  {'idx_q':>5}  {'nq':>3}")
print(f"  {'-'*67}")
for jid in range(model.njoints):
    j = model.joints[jid]
    name = model.names[jid]
    if j.nq > 0:
        print(f"  {jid:3d}  {name:<50}  {j.idx_q:5d}  {j.nq:3d}")
print(f"{'='*65}\n")

def _idx(joint_name):
    jid = model.getJointId(joint_name)
    return model.joints[jid].idx_q

_base_idx       = _idx("tiago_pro/root_joint")
_torso_idx      = _idx("tiago_pro/torso_lift_joint")
_left_arm_idx   = _idx("tiago_pro/arm_left_1_joint")
_right_arm_idx  = _idx("tiago_pro/arm_right_1_joint")
_box_idx        = _idx("box/root_joint")

print(f"→ Base     : q[{_base_idx}:{_base_idx+4}]  (x, y, cos θ, sin θ)")
print(f"→ Torso    : q[{_torso_idx}]")
print(f"→ Left arm : q[{_left_arm_idx}:{_left_arm_idx+7}]")
print(f"→ Right arm: q[{_right_arm_idx}:{_right_arm_idx+7}]")
print(f"→ Box      : q[{_box_idx}:{_box_idx+7}]  (x, y, z, qx, qy, qz, qw)")

# ── 5. Joint bounds ───────────────────────────────────────────────────────────
robot.setJointBounds("tiago_pro/root_joint", [
    -2.0,  2.0,
    -2.0,  2.0,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])
robot.setJointBounds("box/root_joint", [
    -1.5,  3.0,
    -1.5,  1.5,
    -0.2,  1.5,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])

# ── 6. Collision pairs ───────────────────────────────────────────────────────
# Intra-robot collisions are handled by tiago_pro.srdf (<disable_collisions>).
# Cross-model contacts (table vs box, ground vs robot, etc.) span separate
# SRDF files and must be disabled here explicitly, one by one as needed.
_geom = robot.geomModel()

def _disable(name1, name2):
    """Remove all collision pairs whose names contain name1 and name2."""
    ids1 = [i for i, o in enumerate(_geom.geometryObjects) if name1 in o.name]
    ids2 = [i for i, o in enumerate(_geom.geometryObjects) if name2 in o.name]
    removed = 0
    for i in ids1:
        for j in ids2:
            lo, hi = min(i, j), max(i, j)
            cp = pin.CollisionPair(lo, hi)
            idx = _geom.findCollisionPair(cp)
            if idx < len(_geom.collisionPairs):
                _geom.removeCollisionPair(_geom.collisionPairs[idx])
                removed += 1
    print(f"  disabled {removed} pair(s): '{name1}' vs '{name2}'")

# Box resting on table — expected contact, not a collision
_disable("pancake_table_table_top_link", "box/base_link")

print(f"{len(_geom.collisionPairs)} collision pairs active.")

LEFT_ARM_TUCK  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RIGHT_ARM_TUCK = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ── 7. Constraint graph ───────────────────────────────────────────────────────
problem = Problem(robot)
graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)

graph.maxIterations(40)
graph.errorThreshold(1e-3)

# "tiago_pro/gripper" matches the <gripper name="gripper"> in tiago_pro.srdf
# (prefixed with the robot name used in loadModel)
factory.setGrippers(["tiago_pro/gripper"])
factory.setObjects(["box"], [["box/handle"]], [["box/surface"]])
factory.environmentContacts(["table/pancake_table_table_top"])

factory.generate()

# ── Lock passive joints ───────────────────────────────────────────────────────
_cts = ComparisonTypes()
_cts[:] = [ComparisonType.EqualToZero]

def _lock(joint_name, value):
    jid = model.getJointId(joint_name)
    j   = model.joints[jid]
    if j.nq == 2 and j.nv == 1:
        locked_val = np.array([np.cos(value), np.sin(value)])
    else:
        locked_val = np.array([value])
    return LockedJoint(robot, joint_name, locked_val, _cts)

locked = []
locked.append(_lock("tiago_pro/torso_lift_joint", 0.0))
for wheel in ["wheel_front_left_joint", "wheel_front_right_joint",
              "wheel_rear_left_joint",  "wheel_rear_right_joint"]:
    locked.append(_lock(f"tiago_pro/{wheel}", 0.0))
for i, val in enumerate(LEFT_ARM_TUCK):
    locked.append(_lock(f"tiago_pro/arm_left_{i+1}_joint", val))
for name in ["gripper_left_finger_joint",
             "gripper_left_inner_finger_left_joint",
             "gripper_left_fingertip_left_joint",
             "gripper_left_inner_finger_right_joint",
             "gripper_left_fingertip_right_joint",
             "gripper_left_outer_finger_right_joint"]:
    locked.append(_lock(f"tiago_pro/{name}", 0.0))
for name in ["gripper_right_finger_joint",
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

# ── 8. Initial and goal configurations ───────────────────────────────────────
TABLE_Z         = 0.730 + TABLE_OFFSET_Z
BOX_HALF        = 0.025
BOX_Z           = TABLE_Z + BOX_HALF
TABLE_NEAR_EDGE = 0.8 + TABLE_OFFSET_X - 0.3
BOX_X           = TABLE_NEAR_EDGE + BOX_HALF + 0.02

q_init = pin.neutral(robot.model()).copy()
q_init[_left_arm_idx:_left_arm_idx+7]   = LEFT_ARM_TUCK
q_init[_right_arm_idx:_right_arm_idx+7] = RIGHT_ARM_TUCK
q_init[_box_idx:_box_idx+7]             = [BOX_X, -0.15, BOX_Z, 0., 0., 0., 1.]

q_goal = pin.neutral(robot.model()).copy()
q_goal[_left_arm_idx:_left_arm_idx+7]   = LEFT_ARM_TUCK
q_goal[_right_arm_idx:_right_arm_idx+7] = RIGHT_ARM_TUCK
q_goal[_box_idx:_box_idx+7]             = [BOX_X,  0.15, BOX_Z, 0., 0., 0., 1.]

# ── 9. Viewer + q_init validation ────────────────────────────────────────────
v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v(q_init)

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
grasp_edge = graph.getTransition("tiago_pro/gripper > box/handle | f")
success, q_grasp, residual = graph.generateTargetConfig(grasp_edge, q_init, q_init)
print(f"\nGrasp IK: success={success}, residual={residual:.2e}")
if success:
    print(f"  Base      : x={q_grasp[_base_idx]:.3f}, y={q_grasp[_base_idx+1]:.3f}")
    print(f"  Right arm : {np.array(q_grasp[_right_arm_idx:_right_arm_idx+7]).round(3)}")
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
    tr = p.timeRange()
    print(f"\nPath found!  Duration: {tr.second - tr.first:.2f} s")
    v.loadPath(p)
except RuntimeError as e:
    print(f"\nPlanner failed: {e}")
    v(q_init)
    raise SystemExit

# ── 12. Path optimisation ─────────────────────────────────────────────────────

print("\nOptimising path with RandomShortcut …")
try:
    optimizer = RandomShortcut(problem)
    p_opt = optimizer.optimize(p)
    tr_opt = p_opt.timeRange()
    print(f"Optimised!  Duration: {tr_opt.second - tr_opt.first:.2f} s  "
          f"(was {tr.second - tr.first:.2f} s)")
    v.loadPath(p_opt)
except Exception as e:
    print(f"Optimiser failed: {e}")
    print("Keeping original path.")
