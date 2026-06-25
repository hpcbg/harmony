import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import String
import json
import os
import utils
from ament_index_python.packages import get_package_share_directory
import time

from custom_interfaces.action import Move


class EduBotPackBottle(Node):

    def __init__(self):
        super().__init__('xarm_pack_bottle')

        from robot_api.sts_robot_api import StsRobotAPI

        self.gripper_open = 1000
        self.gripper_close = 300
        self.pick_offset_mm = [140, 0, 10]

        self.arm = StsRobotAPI(port="/dev/ttyACM0")
        self.arm.connect()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.arm.clean_error()
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_position(self.gripper_open)
        self.arm.set_servo_angle(
            angle=[0, 0, 0, 0, 0, 0], wait=True, speed=50, mvacc=10)

        self.status_pub = self.create_publisher(
            String, '/xarm_pack_bottle/robot_status', 10)

        self.busy = False

        # Map each action topic suffix to its run_* method
        _actions = {
            'pick':         self.run_pick,
            'fill':         self.run_fill,
            'move_to_cap':  self.run_move_to_cap,
            'cap':          self.run_cap,
            'handover':     self.run_handover,
        }
        self._action_servers = {
            name: ActionServer(
                self, Move,
                f'/xarm_pack_bottle/{name}',
                execute_callback=self._make_execute_cb(name, fn),
                goal_callback=self.goal_cb,
                cancel_callback=self.cancel_cb,
            )
            for name, fn in _actions.items()
        }

        self.publish_status('IDLE')
        self.get_logger().info("XArm Action Server ready")

    # Action servers callbacks

    def _make_execute_cb(self, label, run_fn):
        async def execute_cb(goal_handle):
            self.busy = True
            self.publish_status('BUSY')
            result = Move.Result()
            try:
                self.get_logger().info(f"Executing {label}")
                run_fn(goal_handle)
                result.success = True
                result.message = f"{label.replace('_', ' ').title()} completed"
                self.publish_status('OK')
                goal_handle.succeed()
            except Exception as e:
                self.get_logger().error(str(e))
                result.success = False
                result.message = str(e)
                self.publish_status('ERROR')
                goal_handle.abort()
            finally:
                self.busy = False
            return result
        return execute_cb

    def goal_cb(self, goal_request):
        if self.busy:
            self.get_logger().warn("Rejecting goal: robot is BUSY")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested (not supported)")
        return CancelResponse.REJECT

    # Helpers

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def send_move_feedback(self, goal_handle, text):
        self.get_logger().info(text)
        fb = Move.Feedback()
        fb.state = text
        goal_handle.publish_feedback(fb)

    def move(self, x, y, z, r=180, p=0, yw=0):
        self.arm.set_position(x, y, z, r, p, yw,
                              wait=True, speed=900, mvacc=500)
        self.get_logger().info(
            f"Move to ({x}, {y}, {z}) with orientation ({r}, {p}, {yw})")

    # Robot Movement Routines

    def run_pick(self, goal_handle):
        d = json.loads(goal_handle.request.pose_json)
        pick_pose = d
        self.get_logger().info(
            f"Pick pose: {pick_pose}")

        self.send_move_feedback(goal_handle, "Pick: Move to home")
        self.arm.set_servo_angle(
            angle=[0, 0, 0, 0, 0, 0], wait=True, speed=50, mvacc=10)
        self.arm.set_gripper_position(self.gripper_open, wait=True)

        self.send_move_feedback(goal_handle, "Pick: Approach pick position")
        self.arm.set_position(pick_pose['x'] + self.pick_offset_mm[0], pick_pose['y'] + self.pick_offset_mm[1], self.pick_offset_mm[2] + 50,
                              90 - pick_pose['rotation'], 0, 180, wait=True, speed=100, mvacc=50)

        self.send_move_feedback(goal_handle, "Pick: Move to pick position")
        self.arm.set_position(pick_pose['x'] + self.pick_offset_mm[0], pick_pose['y'] + self.pick_offset_mm[1], self.pick_offset_mm[2],
                              90 - pick_pose['rotation'], 0, 180, wait=True, speed=100, mvacc=50)

        time.sleep(2)
        self.send_move_feedback(goal_handle, "Pick: Close gripper")
        self.arm.set_gripper_position(self.gripper_close, wait=True)
        time.sleep(2)

        self.send_move_feedback(goal_handle, "Pick: Return to approach height")
        self.arm.set_position(pick_pose['x'] + self.pick_offset_mm[0], pick_pose['y'] + self.pick_offset_mm[1], self.pick_offset_mm[2] + 50,
                              pick_pose['rotation'], 0, 180, wait=True, speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Pick: Set after pick orientation")
        self.arm.set_servo_angle(angle=[15.6, 0, -74.86, -0.13, 76.26, 60.62],
                                 wait=True, speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[75, 0, -74.86, -0.13, 76.26, 60.62],
                                 wait=True, speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[75, 0, -70, 0, 0, 0],
                                 wait=True, speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[75, 25, -70, 0, 0, 0], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[75, 25, -70, 0, 0, 0], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Pick: Approach after pick position")
        self.arm.set_servo_angle(angle=[75, 25, -70, 0, -65, 0], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Pick: Move to after pick position")
        self.arm.set_servo_angle(angle=[52, 52, -87, -63, -70, 36], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(goal_handle, "Pick: Open gripper")
        self.arm.set_gripper_position(self.gripper_open, wait=True)
        time.sleep(2)

        self.send_move_feedback(
            goal_handle, "Pick: Return to approach height")
        self.arm.set_servo_angle(angle=[52.7, 48.8, -88.5, -62.8, -68.4, 37.5, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[53.0, 45.4, -90.0, -63.2, -67.3, 39.2, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[53.5, 41.6, -91.3, -63.8, -65.9, 41.5, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[54.1, 37.8, -92.4, -64.4, -64.3, 43.9, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[54.7, 34.1, -93.3, -65.1, -62.7, 46.5, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[55.5, 30.1, -94.1, -66.0, -60.8, 49.4, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[56.4, 26.5, -94.7, -67.0, -58.9, 52.3, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[57.5, 22.3, -95.1, -68.4, -56.5, 56.0, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[58.7, 18.8, -95.3, -69.8, -54.3, 59.4, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[60.3, 14.8, -95.3, -71.6, -51.6, 63.4, -32.4], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Pick: Set bottom grip pick orientation")
        self.arm.set_servo_angle(angle=[62.0, 11.3, -95.0, -73.6, -49.0, 67.4, -32.4], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[71, 0, -50, 0, -75, 0], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[0, 0, 0, 0, 120, 72], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[0, 75, 0, 0, 120, 72], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[72, 75, 10, 0, 110, 72], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Pick: Move to bottom grip pick height")
        self.arm.set_servo_angle(angle=[72, 77, 10, 0, 110, 72], wait=True,
                                 speed=100, mvacc=50)

        time.sleep(2)
        self.send_move_feedback(goal_handle, "Pick: Close gripper")
        self.arm.set_gripper_position(self.gripper_close, wait=True)
        time.sleep(2)

        self.send_move_feedback(
            goal_handle, "Pick: Return to approach height")
        self.arm.set_servo_angle(angle=[72, 60, 10, 0, 110, 72], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Pick: Set bottom grip move orientation")
        self.arm.set_servo_angle(angle=[72, 60, 10, 0, -62, 72], wait=True,
                                 speed=100, mvacc=50)
        self.arm.set_servo_angle(angle=[72, 70, 10, 0, -62, 72], wait=True,
                                 speed=100, mvacc=50)

    def run_fill(self, goal_handle):
        self.send_move_feedback(
            goal_handle, "Fill: Move to fill approach position")
        self.arm.set_servo_angle(angle=[0, 70, 10, 0, -62, 72], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(goal_handle, "Fill: Move to fill position")
        self.arm.set_servo_angle(angle=[-30, 70, 10, 0, -62, 72], wait=True,
                                 speed=100, mvacc=50)

    def run_move_to_cap(self, goal_handle):
        self.send_move_feedback(
            goal_handle, "Cap: Return to fill approach position")
        self.arm.set_servo_angle(angle=[0, 70, 10, 0, -62, 72], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(goal_handle, "Cap: Move to cap position")
        self.arm.set_servo_angle(angle=[90, 70, 10, 0, -62, 72], wait=True,
                                 speed=100, mvacc=50)

        self.send_move_feedback(
            goal_handle, "Cap: Set cap joint to min degree")
        self.arm.set_servo_angle(angle=[90, 70, 10, 0, -62, -179], wait=True,
                                 speed=3000, mvacc=500)

    def run_cap(self, goal_handle):
        self.send_move_feedback(
            goal_handle, "Cap: Set cap joint to max degree")
        self.arm.set_servo_angle(angle=[90, 70, 10, 0, -62, 179], wait=True,
                                 speed=2000, mvacc=500)

    def run_handover(self, goal_handle):
        self.send_move_feedback(goal_handle, "Handover: Open gripper")

        self.arm.set_gripper_position(self.gripper_open, wait=True)
        time.sleep(3)

        self.send_move_feedback(goal_handle, "Handover: Move to home")
        self.arm.set_servo_angle(
            angle=[0, 0, 0, 0, 0, 0], wait=True, speed=100, mvacc=50)

        self.send_move_feedback(goal_handle, "Handover: Completed")
        time.sleep(3)


def main(args=None):
    rclpy.init(args=args)
    node = EduBotPackBottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down XArm ROS bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()
