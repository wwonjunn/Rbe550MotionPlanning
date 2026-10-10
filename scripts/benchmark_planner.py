import math
import os
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint

JOINTS = ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
          'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint']
HOME   = [0.0, 0.4, 1.0, 0.6, 0.0, 0.0]
GOAL_A = [1.2, 0.8, 0.6, 0.4, 0.0, 0.0]
PLANNER_ID = 'RRTstarkConfigDefault'
PLANNING_TIME = 1.0
RUNS = 10
OUT = os.path.expanduser('~/projects/rbe550FinalProj/results/benchmarks/rrtstar_baseline.csv')


def path_length(points):
    """Joint-space path length: sum of distances between consecutive waypoints."""
    return sum(math.dist(a.positions, b.positions) for a, b in zip(points, points[1:]))


def make_request():
    req = GetMotionPlan.Request()
    r = req.motion_plan_request
    r.group_name = 'arm'
    r.pipeline_id = 'ompl'
    r.planner_id = PLANNER_ID
    r.allowed_planning_time = PLANNING_TIME
    r.num_planning_attempts = 1

    # Fixed start: home
    r.start_state.joint_state.name = JOINTS
    r.start_state.joint_state.position = HOME
    r.max_velocity_scaling_factor = 1.0
    r.max_acceleration_scaling_factor = 1.0
    # Fixed goal: goal_a, as tight joint constraints
    goal = Constraints()
    for name, pos in zip(JOINTS, GOAL_A):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        goal.joint_constraints.append(jc)
    r.goal_constraints = [goal]
    return req


def main():
    rclpy.init()
    node = Node('planner_benchmark')
    client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
    if not client.wait_for_service(timeout_sec=10.0):
        print('move_group planning service not available. Is moveit_sim running?')
        return

    rows = []
    for run in range(1, RUNS + 1):
        future = client.call_async(make_request())
        rclpy.spin_until_future_complete(node, future)
        res = future.result().motion_plan_response

        if res.error_code.val != 1:   # 1 = SUCCESS
            row = f'{run},FAILED,error_code={res.error_code.val}'
        else:
            length = path_length(res.trajectory.joint_trajectory.points)
            row = f'{run},{res.planning_time:.4f},{length:.4f}'
        rows.append(row)
        print(row, flush=True)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write(f'# planner: {PLANNER_ID}, budget: {PLANNING_TIME} s, attempts: 1\n')
        f.write(f'# start (home): {HOME}\n')
        f.write(f'# goal (goal_a): {GOAL_A}\n')
        f.write('# box: 0.1 x 0.1 x 0.3 at (0.31, 0.21, 0.25)\n')
        f.write('run,planning_time_s,path_length_rad\n')
        f.write('\n'.join(rows) + '\n')
    print(f'Saved to {OUT}')

    rclpy.shutdown()


main()