import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from custom_interfaces.action import FillBottle
from custom_interfaces.srv import ManipulateState

from std_msgs.msg import String


class FillingStation(Node):

    def __init__(self):
        super().__init__('filling_station')

        self.progress = 100
        self.status = 'ok'

        self.progress_publisher = self.create_publisher(
            String, '/filling_station/progress_json', 1)

        self.status_publisher = self.create_publisher(
            String, '/filling_station/status_json', 1)

        self._manipulate_state_srv = self.create_service(
            ManipulateState, '/filling_station/manipulate_state', self.manipulate_state_callback)

        self._move_gripper_action_server = ActionServer(
            self,
            FillBottle,
            '/filling_station/fill_bottle',
            self.execute_fill_bottle_callback)

        self.timer = self.create_timer(1.0, self.main_loop)

    def execute_fill_bottle_callback(self, goal_handle):
        self.progress = self.progress - json.loads(
            goal_handle.request.amount_json)

        self.get_logger().info(
            f'Requested bottle fill.')

        result = FillBottle.Result()
        result.success = True
        goal_handle.succeed()
        return result

    def manipulate_state_callback(self, request, response):
        manipulated_state = json.loads(request.manipulated_state_json)

        for k in manipulated_state:
            try:
                self.__dict__[k] = manipulated_state[k]
            except:
                print(k)

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Filling Station state manipulated.')

        return response

    def main_loop(self):
        msg = String()
        msg.data = json.dumps({'status': self.status})
        self.status_publisher.publish(msg)

        if self.progress < 100:
            self.progress = self.progress + 10
        msg = String()
        msg.data = json.dumps({'progress': self.progress})
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
