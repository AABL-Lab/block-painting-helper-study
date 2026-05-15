#!/usr/bin/env python3
"""
SMACH state machine for block-painting-helper arm demo
kat.allen@tufts.edu

States:
    Wait                    -> RetrievingObject        (on /button String: payload is color to fetch)
    RetrievingObject        -> WaitingForObject         (GoToLocation service accepted)
    WaitingForObject        -> NavigatingHome           (on /button message)
    NavigatingHome          -> LocatingObjectAndPeople  (GoToLocation service accepted)
    LocatingObjectAndPeople -> PickAndPlace             (always; pose may be None if perception failed)
    PickAndPlace            -> Grasping                 (arm at grasp position)
    Grasping                -> MoveToWorkspace          (on /button: manual stand-in for gripper close)
    MoveToWorkspace         -> SpringController         (arm in workspace)
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Empty
import math
import smach
import smach_ros
from controller_manager_msgs.srv import SwitchController
from bph_interfaces.srv import GoToLocation, GetTargetPose, MoveToPose
from bph_pickmeup.bph_pickmeup_client import BphPickmeupClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker # to see nav2's goal

# ---------------------------------------------------------------------------
# _Latch — thread-safe single-message capture
# ---------------------------------------------------------------------------
class _Latch:
    def __init__(self):
        self._event = threading.Event()
        self.value = None

    def reset(self):
        self._event.clear()
        self.value = None

    def callback(self, msg):
        self.value = msg.data if hasattr(msg, "data") else msg
        self._event.set()

    def wait(self, timeout=None):
        self._event.wait(timeout=timeout)
        return self.value


# ---------------------------------------------------------------------------
# RobotFetchNode — owns all ROS2 infrastructure
# ---------------------------------------------------------------------------
class RobotFetchNode(Node):
    def __init__(self):
        super().__init__("robot_fetch_smach")
        self._goal_marker_pub = self.create_publisher(Marker, "/nav_goal_marker", 10)
        self._nav_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)
        self.declare_parameter("supply_closet_x", 0.105)
        self.declare_parameter("supply_closet_y", -1.5)
        self.declare_parameter("supply_closet_yaw", 0.0)

        self.requested_object_color = "red"

        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.get_logger().info("RobotFetchNode initialised")

        self._nav_recovery_pub = self.create_publisher(String, '/nav_recovery_prompt', 10)
        from action_msgs.srv import CancelGoal as CancelGoalSrv

        self._nav_cancel_pub = self.create_publisher(Empty, '/navigate_to_cancel', 10)
        
        
    @property
    def home_location(self):
        return (
            self.get_parameter("home_x").value,
            self.get_parameter("home_y").value,
            self.get_parameter("home_yaw").value,
        )

    @property
    def supply_closet_location(self):
        return (
            self.get_parameter("supply_closet_x").value,
            self.get_parameter("supply_closet_y").value,
            self.get_parameter("supply_closet_yaw").value,
        )
    def publish_nav_goal_marker(self, x, y, yaw):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.ns = "nav_goal"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        m.scale.x = 0.5   # arrow length
        m.scale.y = 0.05  # arrow width
        m.scale.z = 0.05  # arrow height
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime.sec = 0  # 0 = persistent until replaced
        self._goal_marker_pub.publish(m)
    
    def switch_controllers(self, activate: list, deactivate: list):
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = 2
        event = threading.Event()
        result = [None]

        def cb(f):
            result[0] = f.result()
            event.set()

        self.switch_client.call_async(req).add_done_callback(cb)
        event.wait()
        return result[0]

    def build_and_run_sm(self):
        pickmeup = BphPickmeupClient(self)

        sm = smach.StateMachine(outcomes=["task_complete"])
        sm.userdata.target_pose = None

        with sm:
            smach.StateMachine.add(
                "WAIT", Wait(self, pickmeup),
                transitions={"button_pressed": "RETRIEVING_OBJECT"},
            )
            smach.StateMachine.add(
                "RETRIEVING_OBJECT", RetrievingObject(self),
                transitions={"at_supply_closet": "WAITING_FOR_OBJECT", "nav_error": "WAIT"},
            )
            smach.StateMachine.add(
                "WAITING_FOR_OBJECT", WaitingForObject(self),
                transitions={"object_loaded": "NAVIGATING_HOME"},
            )
            smach.StateMachine.add(
                "NAVIGATING_HOME", NavigatingHome(self),
                transitions={"at_home": "LOCATING_OBJECT_AND_PEOPLE", "nav_error": "WAIT"},
            )
            smach.StateMachine.add(
                "LOCATING_OBJECT_AND_PEOPLE", LocatingObjectAndPeople(self),
                transitions={"proceed": "PICK_AND_PLACE", "try again":"LOCATING_OBJECT_AND_PEOPLE"},
            )
            smach.StateMachine.add(
                "PICK_AND_PLACE", PickAndPlace(self, BphPickmeupClient(self)),
                transitions={"arm_at_grasp_position": "GRASPING", "failed": "WAIT"},
            )
            smach.StateMachine.add(
                "GRASPING", Grasping(self),
                transitions={"grasp_confirmed": "MOVE_TO_WORKSPACE"},
            )
            smach.StateMachine.add(
                "MOVE_TO_WORKSPACE", MoveToWorkspace(self, BphPickmeupClient(self)),
                transitions={"in_workspace": "SPRING_CONTROLLER", "failed": "WAIT"},
            )
            smach.StateMachine.add(
                "SPRING_CONTROLLER", SpringController(self),
                transitions={"done": "WAIT"},
            )

        # Create a publisher so we can check the state easily
        self.state_pub = self.create_publisher(String, '/smach_state', 10)
        self.create_timer(1.0, self._publish_state_cb)
        self.sm = sm 

            
        sis = smach_ros.IntrospectionServer("robot_fetch_smach", sm, "/SM_ROOT")
        sis.start()
        outcome = sm.execute()
        self.get_logger().info("State machine finished: %s" % outcome)
        sis.stop()
        
    def _publish_state_cb(self):
        if not hasattr(self, 'sm'):
            return
        active = self.sm.get_active_states()
        msg = String()
        msg.data = str(active)
        self.state_pub.publish(msg)


    def cancel_navigation(self):
        self._nav_cancel_pub.publish(Empty())
        self.get_logger().info("[Nav] Published cancel to /navigate_to_cancel.")
# ---------------------------------------------------------------------------
# _FetchState — base class with shared helpers
# ---------------------------------------------------------------------------
class _FetchState(smach.State):
    EFFORT_CONTROLLER   = "forward_effort_controller"
    POSITION_CONTROLLER = "scaled_joint_trajectory_controller"

    def __init__(self, node: RobotFetchNode, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys or [],
            output_keys=output_keys or [],
        )
        self._node = node


    def _prompt_nav_recovery(self, destination_name: str) -> str:
        """
        Publish a recovery prompt and block until the user responds.
        
        Listens on two topics simultaneously — whichever fires first wins:
        /nav_recovery_choice  (std_msgs/String)
            'retry'  — call Nav2 again from scratch
            'skip'   — proceed as if navigation succeeded
            'abort'  — fall back to WAIT state
        /initialpose  (geometry_msgs/PoseWithCovarianceStamped)
            RViz "2D Pose Estimate" button — AMCL relocalization,
            then automatically retries navigation.
        
        Returns 'retry', 'skip', or 'abort'.
        """
        # Tell the web UI which destination failed
        prompt_msg = String()
        prompt_msg.data = f"nav_failed:{destination_name}"
        self._node._nav_recovery_pub.publish(prompt_msg)
        
        # Mirror options in the terminal
        self._node.get_logger().warn(
            f"\n{'='*60}\n"
            f"[Nav Recovery] Navigation to '{destination_name}' failed.\n"
            "Options:\n"
            "  ros2 topic pub --once /nav_recovery_choice std_msgs/String \"data: 'retry'\"\n"
            "      → Try Nav2 again\n"
            "  ros2 topic pub --once /nav_recovery_choice std_msgs/String \"data: 'skip'\"\n"
            "      → Proceed as if robot arrived (useful if it's close enough)\n"
            "  ros2 topic pub --once /nav_recovery_choice std_msgs/String \"data: 'abort'\"\n"
            "      → Return to Wait state\n"
            "  OR click '2D Pose Estimate' in RViz to relocalize → auto-retry\n"
            f"{'='*60}"
        )

        result: list[str | None] = [None]
        done = threading.Event()
        subs = []

        def on_choice(msg: String):
            choice = msg.data.strip().lower()
            if choice in ("retry", "skip", "abort") and not done.is_set():
                self._node.get_logger().info(f"[Nav Recovery] /nav_recovery_choice: '{choice}'")
                result[0] = choice
                done.set()
                
        def on_initialpose(msg: PoseWithCovarianceStamped):
            if not done.is_set():
                self._node.get_logger().info(
                    "[Nav Recovery] /initialpose received — AMCL updated, will retry navigation..."
                )
                result[0] = "retry"
                done.set()
                
        subs.append(self._node.create_subscription(
            String, "/nav_recovery_choice", on_choice, 10
        ))
        subs.append(self._node.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", on_initialpose, 10
        ))
        
        done.wait()  # block until either subscription fires
    
        for sub in subs:
            self._node.destroy_subscription(sub)

        choice = result[0]
        if choice in ("skip", "abort"):          # <-- add this block
            self._node.cancel_navigation()
            
        return choice

        
    def _call_service(self, client, request, timeout=10.0):
        event = threading.Event()
        result = [None]
        def cb(f):
            try:
                result[0] = f.result()
            except Exception as e:
                self._node.get_logger().error(f"Service call future exception: {e}")
            event.set()
        client.call_async(request).add_done_callback(cb)
        event.wait(timeout=timeout)
        if result[0] is None:
            self._node.get_logger().error("Service call timed out!")
        return result[0]
        

    def _wait_for_button(self):
        """Block until a String message arrives on /requestedmaterial from the UI; return its payload."""
        latch = _Latch()
        sub = self._node.create_subscription(String, "/requestedmaterial", latch.callback, 10)
        value = latch.wait()
        self._node.destroy_subscription(sub)
        return value


# ---------------------------------------------------------------------------
# State: Wait
# ---------------------------------------------------------------------------
class Wait(_FetchState):
    """Idle. The /button message payload is the color to fetch (e.g. 'red')."""

    def __init__(self, node: RobotFetchNode, pickmeup_client:BphPickmeupClient):
        super().__init__(node, outcomes=["button_pressed"])
        self._pickmeup = pickmeup_client
    def execute(self, userdata):
        # Step 1: ensure position controller is active before any arm move
        self._node.get_logger().info("[Wait] Switching to position controller...")
        self._node.switch_controllers(
            activate=[self.POSITION_CONTROLLER],
            deactivate=[self.EFFORT_CONTROLLER],
        )
        
        # Step 2: move to workspace — only proceed to effort if this succeeds
        self._node.get_logger().info("[Wait] Moving arm to workspace...")
        success, code = self._pickmeup.send_goal(position_name="workspace")
        if success:
            self._node.get_logger().info("[Wait] Activating effort controller...")
            self._node.switch_controllers(
                activate=[self.EFFORT_CONTROLLER],
                deactivate=[self.POSITION_CONTROLLER],
            )
        else:
            self._node.get_logger().warn(
                "[Wait] Arm failed to reach workspace (code=%d) — "
                "staying in position controller, NOT activating effort." % code
            )

        # Step 3: idle until UI sends a color
        self._node.get_logger().info("[Wait] Waiting for /requestedmaterial...")
        color = self._wait_for_button()
        if color:
            self._node.requested_object_color = color
            self._node.get_logger().info("[Wait] Requested color: %s" % color)
        return "button_pressed"

# ---------------------------------------------------------------------------
# State: RetrievingObject
# ---------------------------------------------------------------------------
class RetrievingObject(_FetchState):
    """Navigate to the supply closet."""
    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["at_supply_closet", "nav_error"])
        self._nav_client = node.create_client(GoToLocation, "navigate_to")

      
    def execute(self, userdata):
        self._node.get_logger().info("[RetrievingObject] Switching to position controller...")
        self._node.switch_controllers(
            activate=[self.POSITION_CONTROLLER],
            deactivate=[self.EFFORT_CONTROLLER],
        )

        self._node.publish_nav_goal_marker(*self._node.supply_closet_location)
        while True:
            self._node.get_logger().info("[RetrievingObject] Navigating to supply closet...")
            req = GoToLocation.Request()
            req.x, req.y, req.yaw = self._node.supply_closet_location
            resp = self._call_service(self._nav_client, req)
            
            if not (resp and resp.accepted):
                # Service rejected immediately (Nav2 not ready, bad goal, etc.)
                choice = self._prompt_nav_recovery("supply_closet")
                if choice == "retry":
                    continue
                return "at_supply_closet" if choice == "skip" else "nav_error"
            
            # Wait for Nav2 to actually finish
            latch = _Latch()
            sub = self._node.create_subscription(String, "/navigation_status", latch.callback, 10)
            status = latch.wait(timeout=120.0)
            self._node.destroy_subscription(sub)
            
            if status and status.startswith("SUCCEEDED"):
                return "at_supply_closet"

            # Nav2 accepted but then failed mid-route
            choice = self._prompt_nav_recovery("supply_closet")
            if choice == "retry":
                continue
            return "at_supply_closet" if choice == "skip" else "nav_error"
    

# ---------------------------------------------------------------------------
# State: WaitingForObject
# ---------------------------------------------------------------------------
class WaitingForObject(_FetchState):
    """Wait at the supply closet for a human to load the object (button confirm)."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["object_loaded"])

    def execute(self, userdata):
        self._node.get_logger().info(
            "[WaitingForObject] Waiting for /button to confirm object loaded..."
        )
        self._wait_for_button()
        return "object_loaded"


# ---------------------------------------------------------------------------
# State: NavigatingHome
# ---------------------------------------------------------------------------
class NavigatingHome(_FetchState):
    """Return to home base."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["at_home", "nav_error"])
        self._nav_client = node.create_client(GoToLocation, "navigate_to")


    def execute(self, userdata):
        self._node.publish_nav_goal_marker(*self._node.home_location)
        while True:
            self._node.get_logger().info("[NavigatingHome] Navigating to home...")
            req = GoToLocation.Request()
            req.x, req.y, req.yaw = self._node.home_location
            resp = self._call_service(self._nav_client, req)
            
            if not (resp and resp.accepted):
                choice = self._prompt_nav_recovery("home")
                if choice == "retry":
                    continue
                return "at_home" if choice == "skip" else "nav_error"
            
            latch = _Latch()
            sub = self._node.create_subscription(String, "/navigation_status", latch.callback, 10)
            status = latch.wait(timeout=120.0)
            self._node.destroy_subscription(sub)
            
            if status and status.startswith("SUCCEEDED"):
                return "at_home"

            choice = self._prompt_nav_recovery("home")
            if choice == "retry":
                continue
            return "at_home" if choice == "skip" else "nav_error"
        
# ---------------------------------------------------------------------------
# State: LocatingObjectAndPeople
# ---------------------------------------------------------------------------
class LocatingObjectAndPeople(_FetchState):
    """
    Call color_picker for the target pose.
    Passes the pose (or None on failure) to PickAndPlace via SMACH userdata.
    Tries three times and then goes to PickAndPlace which handles the fallback to default grasp location
    """

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["proceed", "try again"], output_keys=["target_pose"])
        self._perception_client = node.create_client(
            GetTargetPose, "/color_picker/get_target_pose"
        )
        self.perceptionrepeats = 0 # initalize
        
    def execute(self, userdata):
        self._node.get_logger().info(
            "[LocatingObjectAndPeople] Requesting pose for color '%s'..."
            % self._node.requested_object_color
        )
        req = GetTargetPose.Request()
        req.color = self._node.requested_object_color
        resp = self._call_service(self._perception_client, req)
        if resp and resp.success:
            self._node.get_logger().info(
                "[LocatingObjectAndPeople] Pose found: %s" % resp.message
            )
            userdata.target_pose = resp.pose
            self.perceptionrepeats = 0 
            return "proceed"
        
        elif self.perceptionrepeats < 3:
            self.perceptionrepeats = self.perceptionrepeats+1
            return "try again"
        else:
            self._node.get_logger().warn(
                "[LocatingObjectAndPeople] Perception failed (%s) — will use pre_grasp fallback"
                % (resp.message if resp else "no response")
            )
            userdata.target_pose = None
            self.perceptionrepeats = 0
            return "proceed"


# ---------------------------------------------------------------------------
# State: PickAndPlace
# ---------------------------------------------------------------------------
class PickAndPlace(_FetchState):
    """
    Move the arm to the grasp position.
    If color_picker returned a pose, use the MoveToPose service (Cartesian move)
    If perception failed, fall back to the named 'pre_grasp' position.
    """

    def __init__(self, node: RobotFetchNode, pickmeup_client: BphPickmeupClient):
        super().__init__(node, outcomes=["arm_at_grasp_position", "failed"],
                         input_keys=["target_pose"])
        self._pickmeup = pickmeup_client
        self._move_to_pose_client = node.create_client(MoveToPose, "/bph_pickmeup/move_to_pose")

    def execute(self, userdata):
        pose = userdata.target_pose
        if pose is not None:
            self._node.get_logger().info(
                "[PickAndPlace] Moving arm to detected object pose..."
            )
            self._node.get_logger().info(
                "[PickAndPlace] Target pose: position=(%.3f, %.3f, %.3f) frame=%s"
                % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                   pose.header.frame_id if hasattr(pose, 'header') else 'unknown')
    )
            req = MoveToPose.Request()
            req.target_pose = pose
            resp = self._call_service(self._move_to_pose_client, req)
            if resp and resp.success:
                return "arm_at_grasp_position"
            self._node.get_logger().warn(
                "[PickAndPlace] Cartesian move failed (%s) — falling back to pre_grasp"
                % (resp.message if resp else "no response")
            )

        self._node.get_logger().info("[PickAndPlace] Moving arm to pre_grasp position...")
        success, code = self._pickmeup.send_goal(position_name="pre_grasp")
        if success:
            return "arm_at_grasp_position"
        self._node.get_logger().warn("[PickAndPlace] Arm move failed, code=%d" % code)
        return "failed"


# ---------------------------------------------------------------------------
# State: Grasping
# ---------------------------------------------------------------------------
class Grasping(_FetchState):
    """No gripper — /button press is a manual stand-in for gripper close confirmation."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["grasp_confirmed"])

    def execute(self, userdata):
        self._node.get_logger().info("[Grasping] Press /button to confirm grasp...")
        self._wait_for_button()
        return "grasp_confirmed"


# ---------------------------------------------------------------------------
# State: MoveToWorkspace
# ---------------------------------------------------------------------------
class MoveToWorkspace(_FetchState):
    """Carry the grasped object to the workspace position."""

    def __init__(self, node: RobotFetchNode, pickmeup_client: BphPickmeupClient):
        super().__init__(node, outcomes=["in_workspace", "failed"])
        self._pickmeup = pickmeup_client

    def execute(self, userdata):
        self._node.get_logger().info("[MoveToWorkspace] Moving arm to workspace...")
        self._node.get_logger().info("[MoveToWorkspace] Calling send_goal now...")
        success, code = self._pickmeup.send_goal(position_name="workspace")
        self._node.get_logger().info(f"[MoveToWorkspace] send_goal returned: success={success}, code={code}")
        if success:
            return "in_workspace"
        self._node.get_logger().warn("[MoveToWorkspace] Arm move failed, code=%d" % code)
        return "failed"


# ---------------------------------------------------------------------------
# State: SpringController
# ---------------------------------------------------------------------------
class SpringController(_FetchState):
    """Switch to effort controller and run spring/impedance control."""

    def __init__(self, node: RobotFetchNode):
        super().__init__(node, outcomes=["done"])

    def execute(self, userdata):
        self._node.get_logger().info("[SpringController] Switching to effort controller...")
        self._node.switch_controllers(
            activate=[self.EFFORT_CONTROLLER],
            deactivate=[self.POSITION_CONTROLLER],
        )
        self._node.get_logger().info(
            "[SpringController] spring controller running"
        )

        return "done"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = RobotFetchNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.build_and_run_sm()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
