#!/usr/bin/env python3
"""
fake_position_controller.py

A minimal fake position controller for testing the spring controller
study stack without real hardware.

Accepts JointTrajectory commands and smoothly interpolates between
positions, publishing the interpolated state on /joint_states so the
rest of the stack (equilibrium_mover, virtual_spring_node) has
realistic joint state data to work with.

Subscriptions
-------------
~/joint_trajectory  (trajectory_msgs/JointTrajectory)
    Target joint positions. Only the final point is used — intermediate
    waypoints are ignored, as this is a test node, not a real controller.

Publications
------------
/joint_states  (sensor_msgs/JointState)
    Current simulated joint positions and velocities, published at
    publish_rate_hz.

Parameters
----------
urdf_path          : str   -- path to URDF (used to read joint names and limits)
publish_rate_hz    : float -- joint state publish rate (default 50.0)
interpolation_sec  : float -- time to move from current to target position (default 5.0)
initial_position   : list  -- starting joint angles in radians (default all zeros)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import pinocchio as pin


class FakePositionController(Node):

    def __init__(self):
        super().__init__("fake_position_controller")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("interpolation_sec", 5.0)
        self.declare_parameter("initial_position", [0.0, 0.0])

        urdf_path = self.get_parameter("urdf_path").value
        if not urdf_path:
            self.get_logger().fatal("urdf_path must be set.")
            raise RuntimeError("urdf_path not set")

        self._rate_hz         = self.get_parameter("publish_rate_hz").value
        self._interp_sec      = self.get_parameter("interpolation_sec").value
        initial               = self.get_parameter("initial_position").value

        # Load joint names from URDF via pinocchio
        model = pin.buildModelFromUrdf(urdf_path)
        self._joint_names = [
            model.names[i] for i in range(1, model.njoints)
        ]
        n_dof = model.nv
        self.get_logger().info(
            f"Fake controller ready. {n_dof} DOF. "
            f"Joints: {self._joint_names}"
        )

        # State
        if len(initial) != n_dof:
            self.get_logger().warn(
                f"initial_position has {len(initial)} values but arm has "
                f"{n_dof} DOF — using zeros."
            )
            initial = [0.0] * n_dof

        self._q_current  = np.array(initial, dtype=float)
        self._q_target   = self._q_current.copy()
        self._q_start    = self._q_current.copy()
        self._qdot       = np.zeros(n_dof)

        # Interpolation tracking
        self._interp_elapsed = self._interp_sec  # start as "already arrived"

        # Publisher
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Subscriber
        self._traj_sub = self.create_subscription(
            JointTrajectory,
            "~/joint_trajectory",
            self._trajectory_cb,
            10,
        )

        # Timer
        dt = 1.0 / self._rate_hz
        self._timer = self.create_timer(dt, self._tick)
        self._dt = dt

        self.get_logger().info(
            f"Publishing /joint_states at {self._rate_hz} Hz. "
            f"Interpolation time: {self._interp_sec}s. "
            f"Waiting for commands on ~/joint_trajectory."
        )

    # ------------------------------------------------------------------
    # Trajectory callback
    # ------------------------------------------------------------------

    def _trajectory_cb(self, msg: JointTrajectory) -> None:
        """
        Accept a new target. Uses the last point in the trajectory
        (the goal position) and ignores intermediate waypoints.
        """
        if not msg.points:
            self.get_logger().warn("Received empty JointTrajectory — ignoring.")
            return

        # Map incoming joint names to our joint order
        incoming_names = list(msg.name)
        final_point = msg.points[-1]

        try:
            order = [incoming_names.index(n) for n in self._joint_names]
        except ValueError as e:
            self.get_logger().error(
                f"Joint name mismatch in trajectory: {e}. "
                f"Expected: {self._joint_names}, got: {incoming_names}"
            )
            return

        target = np.array(final_point.positions)[order]

        self.get_logger().info(
            f"New target received: {np.round(target, 3)} rad. "
            f"Interpolating over {self._interp_sec}s."
        )

        self._q_start         = self._q_current.copy()
        self._q_target        = target
        self._interp_elapsed  = 0.0

    # ------------------------------------------------------------------
    # Timer tick — interpolate and publish
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """Advance interpolation and publish joint state."""

        if self._interp_elapsed < self._interp_sec:
            self._interp_elapsed += self._dt
            t = min(self._interp_elapsed / self._interp_sec, 1.0)

            # Smooth step (ease in / ease out) so motion doesn't feel jerky
            t_smooth = t * t * (3.0 - 2.0 * t)

            q_prev = self._q_current.copy()
            self._q_current = (
                self._q_start + t_smooth * (self._q_target - self._q_start)
            )
            # Approximate velocity from finite difference
            self._qdot = (self._q_current - q_prev) / self._dt
        else:
            self._q_current = self._q_target.copy()
            self._qdot      = np.zeros_like(self._q_current)

        self._publish_joint_state()

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self._joint_names
        msg.position     = self._q_current.tolist()
        msg.velocity     = self._qdot.tolist()
        msg.effort       = [0.0] * len(self._joint_names)
        self._js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
