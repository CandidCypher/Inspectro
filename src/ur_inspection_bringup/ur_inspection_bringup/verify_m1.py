"""Milestone 1 acceptance checks against a running moveit_mock.launch.py.

    ros2 run ur_inspection_bringup verify_m1

Checks:
  1. TF: tool0 -> zivid_optical_frame exists (camera is in the URDF).
  2. The current (home) state is collision-free with the camera attached.
  3. The camera is collision-checked: some arm configuration puts the camera/bracket in
     collision with the arm, and MoveIt reports it.
  4. Plan + execute a joint-space goal with OMPL on mock hardware, and the arm arrives there.
"""

import math
import random
import sys
import time

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

GROUP = "ur_manipulator"
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
CAMERA_LINKS = ("zivid_",)
# Candidate goals (camera pointing roughly down, as for inspecting parts on a conveyor).
GOALS = [
    [0.8, -1.2, 1.2, -1.57, -1.57, 0.0],
    [-0.8, -1.4, 1.5, -1.67, -1.57, 0.0],
    [0.0, -1.0, 1.0, -1.57, -1.57, 0.5],
]


class VerifyM1(Node):
    def __init__(self):
        super().__init__("verify_m1")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.executor_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.results = []

    def _on_js(self, msg):
        self.joint_state = msg

    # ---------- helpers ----------
    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def record(self, name, ok, detail=""):
        self.results.append((name, ok))
        self.get_logger().info(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")

    def state_of(self, positions):
        rs = RobotState()
        rs.joint_state.name = list(JOINTS)
        rs.joint_state.position = [float(p) for p in positions]
        return rs

    def check_state(self, positions):
        req = GetStateValidity.Request()
        req.group_name = GROUP
        req.robot_state = self.state_of(positions)
        return self.call(self.validity, req)

    def current_positions(self):
        js = self.joint_state
        lookup = dict(zip(js.name, js.position))
        return [lookup[j] for j in JOINTS]

    # ---------- checks ----------
    def check_tf(self):
        ok = self.spin_until(
            lambda: self.tf_buffer.can_transform("tool0", "zivid_optical_frame", Time()), 10.0
        )
        detail = ""
        if ok:
            t = self.tf_buffer.lookup_transform("tool0", "zivid_optical_frame", Time()).transform
            detail = f"xyz=({t.translation.x:.3f}, {t.translation.y:.3f}, {t.translation.z:.3f})"
        self.record("TF tool0 -> zivid_optical_frame", ok, detail)

    def check_home_valid(self):
        res = self.check_state(self.current_positions())
        pairs = {(c.contact_body_1, c.contact_body_2) for c in res.contacts}
        self.record("Current/home state collision-free", res.valid, "" if res.valid else str(pairs))

    def check_camera_collision_detected(self, samples=3000):
        rng = random.Random(42)
        for _ in range(samples):
            q = [rng.uniform(-math.pi, math.pi) for _ in JOINTS]
            res = self.check_state(q)
            if res.valid:
                continue
            for c in res.contacts:
                bodies = (c.contact_body_1, c.contact_body_2)
                if any(b.startswith(CAMERA_LINKS) for b in bodies):
                    self.record(
                        "Camera/bracket is collision-checked against the arm",
                        True,
                        f"{bodies[0]} <-> {bodies[1]}",
                    )
                    return
        self.record("Camera/bracket is collision-checked against the arm", False,
                    f"no camera contact found in {samples} samples")

    def check_plan_and_execute(self):
        goal = next((g for g in GOALS if self.check_state(g).valid), None)
        if goal is None:
            self.record("Plan & execute", False, "no collision-free goal among candidates")
            return
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.pipeline_id = "ompl"
        mpr.num_planning_attempts = 5
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = 0.5
        mpr.max_acceleration_scaling_factor = 0.5
        mpr.start_state.is_diff = True
        mpr.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(joint_name=j, position=p, tolerance_above=0.01,
                                    tolerance_below=0.01, weight=1.0)
                    for j, p in zip(JOINTS, goal)
                ]
            )
        ]
        req.motion_plan_request = mpr
        res = self.call(self.planner, req, timeout=15.0)
        code = res.motion_plan_response.error_code.val if res else None
        if code != 1:
            self.record("Plan (OMPL)", False, f"error_code={code}")
            return
        traj = res.motion_plan_response.trajectory
        n = len(traj.joint_trajectory.points)
        self.record("Plan (OMPL)", True, f"{n} waypoints")

        if not self.executor_ac.wait_for_server(timeout_sec=10.0):
            self.record("Execute on mock hardware", False, "/execute_trajectory not available")
            return
        send = self.executor_ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj))
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.record("Execute on mock hardware", False, "goal rejected")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        exec_code = result_future.result().result.error_code.val if result_future.result() else None
        self.spin_until(lambda: False, 1.0)  # let /joint_states catch up
        err = max(abs(a - b) for a, b in zip(self.current_positions(), goal))
        self.record("Execute on mock hardware", exec_code == 1 and err < 0.02,
                    f"error_code={exec_code}, max joint error={err:.4f} rad")

    def run(self):
        for name, client in (("/check_state_validity", self.validity),
                             ("/plan_kinematic_path", self.planner)):
            if not client.wait_for_service(timeout_sec=30.0):
                self.record(f"Service {name}", False, "not available - is moveit_mock.launch.py running?")
                return False
        if not self.spin_until(lambda: self.joint_state is not None, 10.0):
            self.record("/joint_states", False, "no messages")
            return False
        self.check_tf()
        self.check_home_valid()
        self.check_camera_collision_detected()
        self.check_plan_and_execute()
        return all(ok for _, ok in self.results)


def main():
    rclpy.init()
    node = VerifyM1()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("\nMILESTONE 1:", "ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
