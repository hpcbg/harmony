import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from custom_interfaces.action import FillBottle

from std_msgs.msg import String, Int32


class FillingStation(Node):

    def __init__(self):
        super().__init__('filling_station')

        self.progress = 100
        self.status = 'ok'
        self.busy = False

        self.progress_publisher = self.create_publisher(
            Int32, '/filling_station/progress', 1)

        self.status_publisher = self.create_publisher(
            String, '/filling_station/status_json', 1)

        self._fill_action_server = ActionServer(
            self,
            FillBottle,
            '/filling_station/fill_bottle',
            execute_callback=self.fill_execute_cb,
            goal_callback=self.fill_goal_cb,
            cancel_callback=self.fill_cancel_cb
        )

        self.timer = self.create_timer(0.5, self.main_loop)

    def fill_goal_cb(self, goal_request):
        if self.busy:
            self.get_logger().warn("Rejecting goal: filling station is BUSY")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def fill_cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested (not supported)")
        return CancelResponse.REJECT

    async def fill_execute_cb(self, goal_handle):
        self.get_logger().info("Requested bottle fill.")

        fb = FillBottle.Feedback()
        fb.state = "Starting bottle filling..."
        goal_handle.publish_feedback(fb)

        self.progress = self.progress - json.loads(
            goal_handle.request.amount_json)

        result = FillBottle.Result()
        result.success = True
        goal_handle.succeed()
        return result

    def main_loop(self):
        msg = String()
        msg.data = json.dumps({'status': self.status})
        self.status_publisher.publish(msg)

        if self.progress + 10 >= 100:
            time.sleep(1)

        if self.progress < 100:
            self.progress = self.progress + 10
        msg = Int32()
        msg.data = self.progress
        self.progress_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    filling_station = FillingStation()

    rclpy.spin(filling_station)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filling_station.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
