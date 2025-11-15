import json

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import ManipulateState

from std_msgs.msg import String


class HumanIntentDetector(Node):

    def __init__(self):
        super().__init__('human_intent_detector')

        self.detected_intent = 'unknown'
        self.timeout = 0

        self.detected_intent_publisher = self.create_publisher(
            String, '/human_intent_detector/detected_intent_json', 1)

        self._manipulate_state_srv = self.create_service(
            ManipulateState, '/human_intent_detector/manipulate_state', self.manipulate_state_callback)

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

        self.get_logger().info('Human Intent Detector state manipulated.')

        return response

    def main_loop(self):
        if self.timeout <= 0:
            self.detected_intent = 'unknown'
        else:
            self.timeout = self.timeout - 1

        msg = String()
        msg.data = json.dumps(self.detected_intent)
        self.detected_intent_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    human_intent_detector = HumanIntentDetector()

    rclpy.spin(human_intent_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    human_intent_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
