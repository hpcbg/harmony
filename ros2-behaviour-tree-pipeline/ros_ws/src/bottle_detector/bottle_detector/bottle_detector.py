import json

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import ManipulateState

from std_msgs.msg import String


class BottleDetector(Node):

    def __init__(self):
        super().__init__('bottle_detector')

        self.detected_bottle = {
            'found': False,
            'pose': {
                'x': 0,
                'y': 0,
                'z': 0
            }
        }

        self.detected_bottle_publisher = self.create_publisher(
            String, '/bottle_detector/detected_bottle_json', 1)

        self._manipulate_state_srv = self.create_service(
            ManipulateState, '/bottle_detector/manipulate_state', self.manipulate_state_callback)

        self.timer = self.create_timer(1.0, self.main_loop)

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

        self.get_logger().info('Bottle Detector state manipulated.')

        return response

    def main_loop(self):
        msg = String()
        msg.data = json.dumps(self.detected_bottle)
        self.detected_bottle_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bottle_detector = BottleDetector()

    rclpy.spin(bottle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bottle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
