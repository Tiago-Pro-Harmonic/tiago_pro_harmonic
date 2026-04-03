# ============================================================
# TIAGo Pro — HPP deburring v2
# ============================================================
# Run with: python -i planar_base_deburring_v2.py
#
# Step 1 : load robot + environment and display.
# ============================================================

import numpy as np
from pinocchio import SE3, neutral
from pyhpp.manipulation import Device, Graph, Problem, TransitionPlanner, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory
from pyhpp.manipulation.security_margins import SecurityMargins
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import RandomShortcut
from pyhpp_viser import Viewer

SRDF        = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/tiago_pro.srdf"
PYLONE_URDF = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/pylone/pylone.urdf"
PYLONE_SRDF = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/pylone/pylone.srdf"
TABLE_URDF  = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/table/table.urdf"
TABLE_SRDF  = "/home/user/devel/hpp_ws/src/tiago_pro_hpp_playground/table/table.srdf"

# ── 1. Load TIAGo Pro (mobile planar base) ───────────────────────────────────
robot = Device("tiago_pro")
urdf.loadModel(
    robot, 0, "tiago_pro", "planar",
    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf",
    SRDF,
    SE3.Identity(),
)

# ── 2. Load ground ───────────────────────────────────────────────────────────
urdf.loadModel(
    robot, 0, "ground", "anchor",
    "package://hpp_tutorial/urdf/ground.urdf",
    "package://hpp_tutorial/srdf/ground.srdf",
    SE3.Identity(),
)

# ── 3. Load table (fixed obstacle) ──────────────────────────────────────────
TABLE_X =  1.5
TABLE_Y =  0.0
TABLE_Z =  0.0

urdf.loadModel(
    robot, 0, "table", "anchor",
    TABLE_URDF,
    TABLE_SRDF,
    SE3(np.eye(3), np.array([TABLE_X, TABLE_Y, TABLE_Z])),
)

# ── 4. Load pylone (fixed in environment) ────────────────────────────────────
PYLONE_X = TABLE_X
PYLONE_Y = TABLE_Y
PYLONE_Z = TABLE_Z + 0.73 + 0.3

urdf.loadModel(
    robot, 0, "pylone", "freeflyer",
    PYLONE_URDF,
    PYLONE_SRDF,
    SE3.Identity(),
)

# ── Robot info ────────────────────────────────────────────────────────────────
model = robot.model()
print(f"\n{'='*65}")
print(f"  Full model (TIAGo Pro planar + ground + pylone)")
print(f"{'='*65}")
print(f"  nq (config space dim)  : {model.nq}")
print(f"  nv (velocity space dim): {model.nv}")

def _idx(name):
    return model.joints[model.getJointId(name)].idx_q

_base_idx      = _idx("tiago_pro/root_joint")
_torso_idx     = _idx("tiago_pro/torso_lift_joint")
_left_arm_idx  = _idx("tiago_pro/arm_left_1_joint")
_right_arm_idx = _idx("tiago_pro/arm_right_1_joint")
_pylone_idx    = _idx("pylone/root_joint")

print(f"→ Base     : q[{_base_idx}:{_base_idx+4}]  (x, y, cos θ, sin θ)")
print(f"→ Torso    : q[{_torso_idx}]")
print(f"→ Left arm : q[{_left_arm_idx}:{_left_arm_idx+7}]")
print(f"→ Right arm: q[{_right_arm_idx}:{_right_arm_idx+7}]")
print(f"{'='*65}")
print(f"{len(robot.geomModel().collisionPairs)} collision pairs active.")

print(f"{'='*65}\n")

# ── 4. Joint bounds ──────────────────────────────────────────────────────────
robot.setJointBounds("tiago_pro/root_joint", [
    -3.0, 3.0, -3.0, 3.0,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])
robot.setJointBounds("pylone/root_joint", [
    PYLONE_X - 0.01, PYLONE_X + 0.01,
    PYLONE_Y - 0.01, PYLONE_Y + 0.01,
    PYLONE_Z - 0.01, PYLONE_Z + 0.01,
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
    -float("Inf"), float("Inf"),
])

# ── 5. Constraint graph ───────────────────────────────────────────────────────
HANDLE = "pylone/hole_tiago_25"

# Add handle programmatically with correct frame for TIAGo:
# - position: (0, -0.213, 0) in pylone frame (center of right face, z=0)
# - rotation: Rx(-90°) so that handle Z = world +Y (into the hole)
# - approachingDirection [0,0,-1] = approach from outside (world -Y side)
_R_handle = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
_T_handle  = np.array([0.0, -0.213, 0.0])
robot.addHandle(
    "pylone/pylone_link", HANDLE,
    SE3(_R_handle, _T_handle),
    0.05,          # clearance
    6 * [True],    # all DOFs constrained
)
robot.handles()[HANDLE].approachingDirection = np.array([0, 0, 1])

problem = Problem(robot)
graph = Graph("robot", robot, problem)
factory = ConstraintGraphFactory(graph)

graph.maxIterations(40)
graph.errorThreshold(1e-3)

factory.setGrippers(["tiago_pro/gripper"])
factory.setObjects(["pylone"], [[HANDLE]], [[]])
factory.generate()

# ── 6. Lock passive joints ───────────────────────────────────────────────────
_cts = ComparisonTypes()
_cts[:] = [ComparisonType.EqualToZero]

def _lock(joint_name, value):
    j = model.joints[model.getJointId(joint_name)]
    if j.nq == 2 and j.nv == 1:
        val = np.array([np.cos(value), np.sin(value)])
    else:
        val = np.array([value])
    return LockedJoint(robot, joint_name, val, _cts)

LEFT_ARM_TUCK  = [0.2, -1.83, -0.47, -2.35, 0.0, -1.2, 0.0]
RIGHT_ARM_TUCK = [-0.2, -1.83,  0.47, -2.35,  0.0, -1.2, 0.0]

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
print(f"{len(locked)} joints locked.")

# ── 7. Transitions & security margins ────────────────────────────────────────
transition_approach = graph.getTransition(
    f"tiago_pro/gripper > {HANDLE} | f_01"
)
transition_insert = graph.getTransition(
    f"tiago_pro/gripper > {HANDLE} | f_12"
)

sm = SecurityMargins(problem, factory, ["tiago_pro", "pylone"], robot)
sm.setSecurityMarginBetween("tiago_pro", "pylone", 0.05)
sm.setSecurityMarginBetween("tiago_pro", "table", 0.05)
sm.apply()

# On insertion (f_12), disable robot-pylone collision to allow penetration.
for jname in model.names:
    if "tiago_pro" in jname:
        graph.setSecurityMarginForTransition(
            transition_insert, jname, "pylone/root_joint", float("-inf")
        )

graph.initialize()

# ── 8. Initial configuration ─────────────────────────────────────────────────
q_init = neutral(model).copy()
q_init[_left_arm_idx:_left_arm_idx+7]   = LEFT_ARM_TUCK
q_init[_right_arm_idx:_right_arm_idx+7] = RIGHT_ARM_TUCK
q_init[_pylone_idx:_pylone_idx+7] = [PYLONE_X, PYLONE_Y, PYLONE_Z, 0., 0., 0., 1.]

# ── 9. Viewer ────────────────────────────────────────────────────────────────
v = Viewer(robot)
v.initViewer(open=False, loadModel=True)
v.setProblem(problem)
v.setGraph(graph)
v(q_init)

def play(path, n=50, dt=0.05):
    """Visualise un chemin HPP avec n segments, dt secondes entre chaque config."""
    import time
    t0 = path.timeRange().first
    tf = path.timeRange().second
    for i in range(n):
        t = t0 + i * (tf - t0) / (n - 1)
        q = path.eval(t)[0]
        v(q)
        time.sleep(dt)

# ── 10. Generate pregrasp configuration (collision-free) ─────────────────────
shooter = problem.configurationShooter()
qpg = None
for i in range(50):
    q = shooter.shoot()
    res, q_candidate, err = graph.generateTargetConfig(transition_approach, q_init, q)
    if not res:
        continue
    pv = transition_approach.pathValidation()
    res, report = pv.validateConfiguration(q_candidate)
    if not res:
        continue
    qpg = q_candidate
    print(f"qpg found at iteration {i}, err={err:.2e}")
    break

if qpg is None:
    raise SystemExit("Failed to find a collision-free qpg in 50 attempts.")

v(qpg)

# ── 11. Generate grasp configuration ─────────────────────────────────────────
res, qg, err = graph.generateTargetConfig(transition_insert, qpg, qpg)
print(f"qg:  res={res}, err={err:.2e}")
if not res:
    raise SystemExit("Failed to generate qg.")
v(qg)

# ── 12. Plan p1 : q_init → qpg (approach) ────────────────────────────────────
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

# ── 13. Plan p2 : qpg → qg (insertion) ───────────────────────────────────────
planner.setTransition(transition_insert)
q_goal[0, :] = qg
p2 = planner.planPath(qpg, q_goal, True)
print("p2 found (qpg → qg).")
v.loadPath(p2)

# ── 14. p3 : retraction (reverse of p2) ──────────────────────────────────────
p3 = p2.reverse()
print("p3 ready (retraction).")
v.loadPath(p3)

print("\nReady. Use v(q) to display a configuration.")
