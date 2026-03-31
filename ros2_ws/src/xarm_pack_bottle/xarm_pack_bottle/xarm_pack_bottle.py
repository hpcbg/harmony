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


class XArmPackBottle(Node):

    def __init__(self):
        super().__init__('xarm_pack_bottle')

        pkg = get_package_share_directory('xarm_pack_bottle')
        default_config_path = os.path.join(
            pkg, 'config', 'xarm_pack_bottle.json')

        self.declare_parameter('config_path', default_config_path)

        config_path = self.get_parameter('config_path').value
        self.get_logger().info(f"Loading config from: {config_path}")

        self.CONFIG = utils.json_config.load(config_path)

        if self.CONFIG['EMULATE_ROBOT']:
            from utils.xarm_emulator import XArmAPI
        else:
            from xarm.wrapper import XArmAPI

        self.arm = XArmAPI(self.CONFIG['ROBOT_IP'])
        self.arm.connect()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.arm.clean_error()
        self.arm.set_gripper_enable(True)

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
        pick_pose = d['pick_pose']

        self.send_move_feedback(goal_handle, "Pick: Move to home")
        self.arm.set_servo_angle(
            angle=self.CONFIG['HOME_POS_JOINTS_DEG'], wait=True)
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_OPEN_POS'], wait=True)
        time.sleep(3)

        self.send_move_feedback(goal_handle, "Pick: Approach pick position")
        self.move(pick_pose['x'], pick_pose['y'], self.CONFIG['APPROACH_HEIGHT_MM'],
                  pick_pose['roll_degrees'], pick_pose['pitch_degrees'], pick_pose['yaw_degrees'])

        self.send_move_feedback(goal_handle, "Pick: Move to pick position")
        self.move(pick_pose['x'], pick_pose['y'], pick_pose['z'],
                  pick_pose['roll_degrees'], pick_pose['pitch_degrees'], pick_pose['yaw_degrees'])

        time.sleep(2)
        self.send_move_feedback(goal_handle, "Pick: Close gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_CLOSE_POS'], wait=True)
        time.sleep(2)

        self.send_move_feedback(goal_handle, "Pick: Return to approach height")
        self.move(pick_pose['x'], pick_pose['y'], self.CONFIG['APPROACH_HEIGHT_MM'],
                  pick_pose['roll_degrees'], pick_pose['pitch_degrees'], pick_pose['yaw_degrees'])

        self.send_move_feedback(
            goal_handle, "Pick: Set after pick orientation")
        self.move(pick_pose['x'], pick_pose['y'], self.CONFIG['APPROACH_HEIGHT_MM'],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][0], self.CONFIG['AFTER_PICK_RPY_DEG'][1],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Pick: Approach after pick position")
        self.move(self.CONFIG['AFTER_PICK_POS_MM'][0], self.CONFIG['AFTER_PICK_POS_MM'][1],
                  self.CONFIG['APPROACH_HEIGHT_MM'],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][0], self.CONFIG['AFTER_PICK_RPY_DEG'][1],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Pick: Move to after pick position")
        self.move(self.CONFIG['AFTER_PICK_POS_MM'][0], self.CONFIG['AFTER_PICK_POS_MM'][1],
                  self.CONFIG['AFTER_PICK_POS_MM'][2],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][0], self.CONFIG['AFTER_PICK_RPY_DEG'][1],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][2])

        self.send_move_feedback(goal_handle, "Pick: Open gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_OPEN_POS'], wait=True)
        time.sleep(2)

        self.send_move_feedback(
            goal_handle, "Pick: Return to approach height")
        self.move(self.CONFIG['AFTER_PICK_POS_MM'][0], self.CONFIG['AFTER_PICK_POS_MM'][1],
                  self.CONFIG['APPROACH_HEIGHT_MM'],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][0], self.CONFIG['AFTER_PICK_RPY_DEG'][1],
                  self.CONFIG['AFTER_PICK_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Pick: Set bottom grip pick orientation")
        self.move(self.CONFIG['AFTER_PICK_POS_MM'][0], self.CONFIG['AFTER_PICK_POS_MM'][1],
                  self.CONFIG['APPROACH_HEIGHT_MM'],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][0], self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][1],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Pick: Move to bottom grip approach height")
        self.move(self.CONFIG['BOTTOM_PICK_POS_MM'][0], self.CONFIG['BOTTOM_PICK_POS_MM'][1],
                  self.CONFIG['APRROACH_HEIGHT_MM'],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][0], self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][1],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Pick: Move to bottom grip pick height")
        self.move(self.CONFIG['BOTTOM_PICK_POS_MM'][0], self.CONFIG['BOTTOM_PICK_POS_MM'][1],
                  self.CONFIG['BOTTOM_GRIP_HEIGHT_MM'],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][0], self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][1],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][2])

        time.sleep(2)
        self.send_move_feedback(goal_handle, "Pick: Close gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_CLOSE_POS'], wait=True)
        time.sleep(2)

        self.send_move_feedback(
            goal_handle, "Pick: Return to approach height")
        self.move(self.CONFIG['BOTTOM_PICK_POS_MM'][0], self.CONFIG['BOTTOM_PICK_POS_MM'][1],
                  self.CONFIG['APPROACH_HEIGHT_MM'],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][0], self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][1],
                  self.CONFIG['BOTTOM_GRIP_PICK_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Pick: Set bottom grip move orientation")
        self.arm.set_servo_angle(
            angle=self.CONFIG['BOTTOM_GRIP_MOVE_JOINTS_DEG'], wait=True)

        time.sleep(3)

    def run_fill(self, goal_handle):

        self.send_move_feedback(
            goal_handle, "Fill: Move to fill approach position")
        self.arm.set_servo_angle(
            angle=self.CONFIG['FILL_APPROACH_JOINTS_DEG'], wait=True, speed=400, mvacc=200)

        self.send_move_feedback(goal_handle, "Fill: Move to fill position")
        self.arm.set_servo_angle(
            angle=self.CONFIG['FILL_POS_JOINTS_DEG'], wait=True, speed=400, mvacc=200)

        time.sleep(3)

    def run_move_to_cap(self, goal_handle):
        self.send_move_feedback(
            goal_handle, "Cap: Return to fill approach position")
        self.arm.set_servo_angle(
            angle=self.CONFIG['FILL_APPROACH_JOINTS_DEG'], wait=True, speed=700, mvacc=300)

        self.send_move_feedback(goal_handle, "Cap: Move to cap position")
        self.move(self.CONFIG['CAP_POS_MM'][0], self.CONFIG['CAP_POS_MM'][1],
                  self.CONFIG['CAP_POS_MM'][2],
                  self.CONFIG['BOTTOM_GRIP_MOVE_RPY_DEG'][0], self.CONFIG['BOTTOM_GRIP_MOVE_RPY_DEG'][1],
                  self.CONFIG['BOTTOM_GRIP_MOVE_RPY_DEG'][2])

        self.send_move_feedback(
            goal_handle, "Cap: Set cap joint to min degree")
        self.arm.set_servo_angle(
            servo_id=self.CONFIG['CAP_JOINT_ID'], angle=self.CONFIG['CAP_JOINT_MIN_MAX_DEG'][0], wait=True,  speed=900, mvacc=500)

        time.sleep(3)

    def run_cap(self, goal_handle):
        self.send_move_feedback(
            goal_handle, "Cap: Set cap joint to max degree")
        self.arm.set_servo_angle(servo_id=self.CONFIG['CAP_JOINT_ID'],
                                 angle=self.CONFIG['CAP_JOINT_MIN_MAX_DEG'][1], speed=900, mvacc=700, wait=True)

        time.sleep(3)

    def run_handover(self, goal_handle):
        self.send_move_feedback(goal_handle, "Handover: Open gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_OPEN_POS'], wait=True)
        time.sleep(5)

        self.send_move_feedback(goal_handle, "Handover: Move to home")
        self.arm.set_servo_angle(
            angle=self.CONFIG['HOME_POS_JOINTS_DEG'], speed=400, mvacc=200, wait=True)

        self.send_move_feedback(goal_handle, "Handover: Completed")
        time.sleep(3)


def main(args=None):
    rclpy.init(args=args)
    node = XArmPackBottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down XArm ROS bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()
