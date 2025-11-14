import json

import time

import rclpy
from rclpy.node import Node
import py_trees
from rclpy.action import ActionClient

from std_msgs.msg import String

from custom_interfaces.action import RotateGripper, MoveRobot
from custom_interfaces.srv import Void


class ReportStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='capping', access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.status_publisher = self.node.create_publisher(
            String, '/system_skill_cap/status_json', 1)

    def update(self):
        msg = String()
        msg.data = json.dumps({'active': self.blackboard.capping})
        self.status_publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class CapActive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='capping', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.capping:
            self.logger.info('Active')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GestureStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name, ):
        super().__init__(name)
        self.subscriber = None
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='detected_gesture',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.detected_gesture = 'unknown'

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscriber = self.node.create_subscription(
            String, '/gesture_detector/detected_gesture_json', self.callback, 10
        )

    def callback(self, msg):
        data = json.loads(msg.data)
        self.blackboard.detected_gesture = data

    def update(self):
        return py_trees.common.Status.RUNNING


class HumanIntentStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name, ):
        super().__init__(name)
        self.subscriber = None
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='detected_intent',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.detected_intent = 'unknown'

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscriber = self.node.create_subscription(
            String, '/human_intent_detector/detected_intent_json', self.callback, 10
        )

    def callback(self, msg):
        data = json.loads(msg.data)
        self.blackboard.detected_intent = data

    def update(self):
        return py_trees.common.Status.RUNNING


class RobotStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.pose_subscriber = None
        self.node = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='robot_status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.robot_status = {'status': 'busy'}
        self.blackboard.register_key(
            key='robot_pose',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.robot_pose = {'x': 0, 'y': 0, 'z': 0}
        self.blackboard.register_key(
            key='robot_gripper',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.robot_gripper = {'rotation': 0.0, 'position': 1.0}

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.pose_subscriber = self.node.create_subscription(
            String, '/robot_control/pose_json', self.pose_callback, 10
        )
        self.gripper_subscriber = self.node.create_subscription(
            String, '/robot_control/gripper_json', self.gripper_callback, 10
        )
        self.status_subscriber = self.node.create_subscription(
            String, '/robot_control/status_json', self.status_callback, 10
        )

    def pose_callback(self, msg):
        self.blackboard.robot_pose = json.loads(msg.data)

    def gripper_callback(self, msg):
        self.blackboard.robot_gripper = json.loads(msg.data)

    def status_callback(self, msg):
        self.blackboard.robot_status = json.loads(msg.data)

    def update(self):
        return py_trees.common.Status.RUNNING


class RobotAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='robot_status', access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key='robot_pose', access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key='robot_gripper', access=py_trees.common.Access.READ)

    def update(self):
        robot_data = f'Pose ({self.blackboard.robot_pose['x']:.2f}, {self.blackboard.robot_pose['y']:.2f}, {self.blackboard.robot_pose['z']:.2f}), Gripper (position: {self.blackboard.robot_gripper['position']:.2f}, rotation: {self.blackboard.robot_gripper['rotation']:.2f})'
        if self.blackboard.robot_status['status'] == 'available':
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info(
                f'Robot is moving... {robot_data}')
            return py_trees.common.Status.RUNNING


class CapInHand(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='detected_gesture', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.detected_gesture == 'cap_in_hand':
            self.logger.info(
                'Gesture detected: cap is holded by human operator.')
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info('Waiting for gesture...')
            return py_trees.common.Status.RUNNING


class RotationRequested(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='detected_intent', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.detected_intent == 'rotate_bottle':
            self.logger.info(
                'Intent detected: human operator requested bottle rotation.')
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info('Waiting for intent...')
            return py_trees.common.Status.RUNNING


class SetCapPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, MoveRobot, '/robot_control/move')

    def update(self):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error('Robot control unavailable!')
            return py_trees.common.Status.FAILURE
        goal_msg = MoveRobot.Goal()
        goal_msg.pose_json = json.dumps({'x': 1.4, 'y': 0.8, 'z': 0.8})
        self.node.get_logger().info('Send action to move to fill position.')
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


class SendGripperRotate(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, RotateGripper, '/robot_control/rotate_gripper')

    def update(self):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error('Robot control unavailable!')
            return py_trees.common.Status.FAILURE
        goal_msg = RotateGripper.Goal()
        goal_msg.rotation_json = json.dumps(2.0)
        self.node.get_logger().info('Send action to rotate the bottle.')
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


class WaitNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, wait_time: float):
        super().__init__(name)
        self.wait_time = wait_time
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()
        self.logger.info(f'Waiting for {self.wait_time:.1f} seconds...')

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed < self.wait_time:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


class CompleteCapping(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='capping',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        self.blackboard.capping = False
        self.logger.info('Cap completed.')
        return py_trees.common.Status.SUCCESS


class CapNode(Node):
    def __init__(self):
        super().__init__('system_skill_cap')

        # behaviour
        root = py_trees.composites.Parallel(
            name='System Skill Cap',
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            )
        )

        system_state = py_trees.composites.Sequence(
            'System State', memory=True)
        system_state.add_children([
            ReportStatus('Report Status'),
            RobotStatus('Robot Status'),
            GestureStatus('Gesture Detector Status'),
            HumanIntentStatus('Human Intent Detector Status'),
        ])

        go_to_cap_position_seq = py_trees.composites.Sequence(
            'Go to Cap Position', memory=True)
        go_to_cap_position_seq.add_children([
            RobotAvailable('Robot Stationary?'),
            SetCapPosition('Send Cap Position'),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        detectors_seq = py_trees.composites.Parallel(
            name='Detectors',
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        detectors_seq.add_children([
            RotationRequested('Human Requested Rotation?'),
            CapInHand('Gesture Cap Holded Detected?')
        ])
        execute_cap_seq = py_trees.composites.Sequence(
            'Rotate Bottle', memory=True)
        execute_cap_seq.add_children([
            RobotAvailable('Robot at Cap Position?'),
            detectors_seq,
            SendGripperRotate('Send Bottle Rotate Action'),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        complete_capping_seq = py_trees.composites.Sequence(
            'Finish Cap', memory=True)
        complete_capping_seq.add_children([
            RobotAvailable('Robot Stationary?'),
            CompleteCapping('Finish Cap')
        ])

        cap_seq = py_trees.composites.Sequence(
            'Cap', memory=True)
        cap_seq.add_children([
            CapActive('Active?'),
            go_to_cap_position_seq,
            execute_cap_seq,
            complete_capping_seq
        ])

        root.add_children([system_state, cap_seq])

        # behaviour tree
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(node=self)
        # render the tree
        py_trees.display.render_dot_tree(
            root,
            target_directory='diagrams',
            # with_blackboard_variables=True
        )

        self.blackboard = py_trees.blackboard.Client(name='Global')
        self.blackboard.register_key(
            key='capping',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.capping = False

        # tick_tree timer
        self.timer = self.create_timer(1.0, self.tick_tree)

        self._start_capping_srv = self.create_service(
            Void, '/system_skill_cap/execute', self.start_capping_callback)

        self._abort_capping_srv = self.create_service(
            Void, '/system_skill_cap/abort', self.abort_capping_callback)

    def start_capping_callback(self, request, response):
        self.blackboard.capping = True

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Cap started.')

        return response

    def abort_capping_callback(self, request, response):
        self.blackboard.capping = False

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Cap aborted.')

        return response

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = CapNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
