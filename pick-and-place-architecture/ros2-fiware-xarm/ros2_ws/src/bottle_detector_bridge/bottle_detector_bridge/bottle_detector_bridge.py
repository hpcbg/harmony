import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ament_index_python.packages import get_package_share_directory

import requests
import time
import json

from std_msgs.msg import String
import utils
import os

from custom_interfaces.action import DetectBottle


class BottleDetectorBridge(Node):

    def __init__(self):
        super().__init__("bottle_detector_bridge")

        pkg = get_package_share_directory('bottle_detector_bridge')
        default_config_path = os.path.join(
            pkg, 'config', 'bottle_detector_bridge.json')

        self.declare_parameter('config_path', default_config_path)

        config_path = self.get_parameter('config_path').value
        self.get_logger().info(f"Loading config from: {config_path}")

        self.CONFIG = utils.json_config.load(config_path)

        self.api_base = f"{self.CONFIG['API_URL']}/api/v1"

        # Publishers
        self.processed_image_pub = self.create_publisher(
            String, "/bottle_detector_brigde/processed_image", 1)

        self.found_bottle_pub = self.create_publisher(
            String, "/bottle_detector_brigde/found_bottle", 1)

        self.pick_and_place_pub = self.create_publisher(
            String, "/bottle_detector_brigde/pick_and_place", 1)

        self.status_pub = self.create_publisher(
            String, "/bottle_detector_brigde/api_status", 1)

        # Action server
        self._action_server = ActionServer(
            self,
            DetectBottle,
            "/bottle_detector_bridge/detect_bottle",
            self.execute_callback
        )

        # API health timer
        self.create_timer(1.0, self.publish_api_status)

        self.get_logger().info("Perception bridge node started")

    def publish_api_status(self):
        try:
            r = requests.get(
                f"{self.api_base.replace('/api/v1', '')}/health", timeout=0.5)
            status = "OK" if r.status_code == 200 else "ERROR"
        except Exception:
            status = "OFFLINE"

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Perception action started")

        feedback = DetectBottle.Feedback()

        # Create job
        try:
            r = requests.post(f"{self.api_base}/jobs", timeout=1.0)
            r.raise_for_status()
            job_id = r.json()["job_id"]
        except Exception as e:
            goal_handle.abort()
            return DetectBottle.Result(
                success=False,
                job_id="",
                message=str(e)
            )

        # Poll status
        while rclpy.ok():
            r = requests.get(f"{self.api_base}/jobs/{job_id}/status")
            status = r.json()["status"]

            feedback.status = status
            goal_handle.publish_feedback(feedback)

            if status == "DONE":
                break
            if status == "FAILED":
                goal_handle.abort()
                return DetectBottle.Result(
                    success=False,
                    job_id=job_id,
                    message="Job failed"
                )
            time.sleep(0.5)

        # Get processed image
        ros_img = String()
        ros_img.data = f"{self.api_base}/jobs/{job_id}/image/processed"
        self.processed_image_pub.publish(ros_img)

        # Get pick & place
        pp_resp = requests.get(
            f"{self.api_base}/jobs/{job_id}/pick-place")
        pp_resp = pp_resp.json()

        pp_msg = String()
        pp_msg.data = "Not found!"
        if 'pick_pose' in pp_resp:
            pp_msg.data = f"x: {pp_resp['pick_pose']['x']:.1f}, y: {pp_resp['pick_pose']['y']:.1f}, yaw: {int(pp_resp['pick_pose']['yaw_degrees'])}"
        self.found_bottle_pub.publish(pp_msg)
        pp_msg.data = json.dumps(pp_resp)
        self.pick_and_place_pub.publish(pp_msg)

        goal_handle.succeed()

        return DetectBottle.Result(
            success=True,
            job_id=job_id,
            message="Perception completed"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
