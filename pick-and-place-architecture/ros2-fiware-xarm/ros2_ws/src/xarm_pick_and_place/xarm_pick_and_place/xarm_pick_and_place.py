import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import String
import json
import os
import utils
from ament_index_python.packages import get_package_share_directory

from custom_interfaces.action import PickAndPlace


class XArmROS(Node):

    def __init__(self):
        super().__init__('xarm_pick_and_place')

        pkg = get_package_share_directory('xarm_pick_and_place')
        default_config_path = os.path.join(
            pkg, 'config', 'xarm_pick_and_place.json')

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
            String, '/xarm_pick_and_place/robot_status', 10)

        self.busy = False

        self._action_server = ActionServer(
            self,
            PickAndPlace,
            '/xarm_pick_and_place/pick_and_place',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        self.publish_status('IDLE')
        self.get_logger().info("XArm Action Server ready")

    # ---------- ACTION CALLBACKS ----------

    def goal_cb(self, goal_request):
        if self.busy:
            self.get_logger().warn("Rejecting goal: robot is BUSY")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested (not supported)")
        return CancelResponse.REJECT

    async def execute_cb(self, goal_handle):
        self.busy = True
        self.publish_status('BUSY')

        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()

        try:
            data = json.loads(goal_handle.request.pick_and_place_json)
            self.get_logger().info("Executing pick & place")

            self.run_pick_and_place(data, feedback, goal_handle)

            result.success = True
            result.message = "Pick & place completed"
            self.publish_status('OK')

            goal_handle.succeed()

        except Exception as e:
            self.get_logger().error(str(e))
            result.success = False
            result.message = str(e)
            self.publish_status('ERROR')
            goal_handle.abort()

        self.busy = False
        return result

    # ---------- ROBOT LOGIC ----------

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def send_feedback(self, goal_handle, text):
        self.get_logger().info(text)
        fb = PickAndPlace.Feedback()
        fb.state = text
        goal_handle.publish_feedback(fb)

    def move(self, x, y, z, r=180, p=0, yw=0):
        self.arm.set_position(x, y, z, r, p, yw,
                              wait=True, speed=900, mvacc=500)
        self.get_logger().info(
            f"Move to ({x}, {y}, {z}) with orientation ({r}, {p}, {yw})")

    def run_pick_and_place(self, d, fb, goal_handle):
        pick_pose = d['pick_pose']
        place_pose = d['place_pose']

        x, y, z, roll, pitch, yaw = self.CONFIG['HOME_POS']

        self.send_feedback(goal_handle, "Open gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_OPEN_POS'], wait=True)

        self.send_feedback(goal_handle, "Move to home.")
        self.move(x, y, z, roll, pitch, yaw)

        z = self.CONFIG['APPROACH_HEIGHT']
        self.send_feedback(goal_handle, "Move to approach height.")
        self.move(x, y, z, roll, pitch, yaw)

        roll = pick_pose['roll_degrees']
        pitch = pick_pose['pitch_degrees']
        yaw = pick_pose['yaw_degrees']
        self.send_feedback(goal_handle, "Move to pick orientation.")
        self.move(x, y, z, roll, pitch, yaw)

        x = pick_pose['x']
        y = pick_pose['y']
        self.send_feedback(goal_handle, "Move to pick XY position.")
        self.move(x, y, z, roll, pitch, yaw)

        z = pick_pose['z']
        self.send_feedback(goal_handle, "Move to pick Z position.")
        self.move(x, y, z, roll, pitch, yaw)

        self.send_feedback(goal_handle, "Close gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_CLOSE_POS'], wait=True)

        z = self.CONFIG['APPROACH_HEIGHT']
        self.send_feedback(goal_handle, "Move to approach height.")
        self.move(x, y, z, roll, pitch, yaw)

        roll = place_pose['roll_degrees']
        pitch = place_pose['pitch_degrees']
        yaw = place_pose['yaw_degrees']
        self.send_feedback(goal_handle, "Move to place orientation.")
        self.move(x, y, z, roll, pitch, yaw)

        x = place_pose['x']
        y = place_pose['y']
        self.send_feedback(goal_handle, "Move to place XY position.")
        self.move(x, y, z, roll, pitch, yaw)

        z = place_pose['z']
        self.send_feedback(goal_handle, "Move to place Z position.")
        self.move(x, y, z, roll, pitch, yaw)

        self.send_feedback(goal_handle, "Open gripper")
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_OPEN_POS'], wait=True)

        z = self.CONFIG['APPROACH_HEIGHT']
        self.send_feedback(goal_handle, "Move to approach height.")
        self.move(x, y, z, roll, pitch, yaw)

        x, y, z, roll, pitch, yaw = self.CONFIG['HOME_POS']
        self.send_feedback(goal_handle, "Move to home.")

        self.arm.set_servo_angle(
            angle=self.CONFIG['HOME_POS_ANGLES'], wait=True)
        self.arm.set_gripper_position(
            self.CONFIG['GRIPPER_OPEN_POS'], wait=True)


def main(args=None):
    rclpy.init(args=args)
    node = XArmROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down XArm ROS bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()
