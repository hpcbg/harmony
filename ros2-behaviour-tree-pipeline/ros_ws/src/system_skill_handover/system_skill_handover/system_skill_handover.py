import json

import time

import rclpy
from rclpy.node import Node
import py_trees
from rclpy.action import ActionClient

from std_msgs.msg import String

from custom_interfaces.action import RotateGripper, MoveRobot, MoveGripper
from custom_interfaces.srv import Void


class ReportStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='handingover', access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.status_publisher = self.node.create_publisher(
            String, '/system_skill_handover/status_json', 1)

    def update(self):
        msg = String()
        msg.data = json.dumps({'active': self.blackboard.handingover})
        self.status_publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class HandoverActive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='handingover', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.handingover:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class GestureStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
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
    def __init__(self, name):
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


class BottleInHand(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='detected_gesture', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.detected_gesture == 'bottle_in_hand':
            self.logger.info('Gesture detected: bottle is held by human.')
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info('Waiting for gesture...')
            return py_trees.common.Status.RUNNING


class ReleaseRequested(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='detected_intent', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.detected_intent == 'release_bottle':
            self.logger.info(
                'Intent detected: human operator requested bottle release.')
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info('Waiting for intent...')
            return py_trees.common.Status.RUNNING


class SetHandoverPosition(py_trees.behaviour.Behaviour):
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
        goal_msg.pose_json = json.dumps({'x': 1.4, 'y': 2.8, 'z': 0.6})
        self.node.get_logger().info('Send action to move to handover position.')
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


class SetHomePosition(py_trees.behaviour.Behaviour):
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
        goal_msg.pose_json = json.dumps({'x': 0.0, 'y': 0.0, 'z': 0.0})
        self.node.get_logger().info('Send action to move to home position.')
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
        goal_msg.rotation_json = json.dumps(0)
        self.node.get_logger().info('Send action to reset gripper rotation.')
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


class ReleaseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, MoveGripper, '/robot_control/move_gripper')

    def update(self):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error('Robot control unavailable!')
            return py_trees.common.Status.FAILURE
        goal_msg = MoveGripper.Goal()
        goal_msg.position_json = json.dumps(1.0)
        self.node.get_logger().info('Send action to release gripper.')
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


class CompleteHandingOver(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='handingover',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        self.blackboard.handingover = False
        self.logger.info('Handover completed.')
        return py_trees.common.Status.SUCCESS


class HandOverNode(Node):
    def __init__(self):
        super().__init__('system_skill_handover')

        # behaviour
        root = py_trees.composites.Parallel(
            name='System Skill Handover',
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

        go_to_handover_position_seq = py_trees.composites.Sequence(
            'Go to Handover Position', memory=True)
        go_to_handover_position_seq.add_children([
            RobotAvailable('Robot Stationary'),
            SetHandoverPosition('Send Handover Position'),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        detectors_seq = py_trees.composites.Parallel(
            name='Detectors',
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        detectors_seq.add_children([
            ReleaseRequested('Human Requested Bottle Release?'),
            BottleInHand('Gesture Bottle Holded?')
        ])
        execute_handover_seq = py_trees.composites.Sequence(
            'Execute Handover', memory=True)
        execute_handover_seq.add_children([
            RobotAvailable("Robot at Handover Position?"),
            detectors_seq,
            ReleaseGripper('Release Bottle'),
            WaitNode('Wait 1 sec', wait_time=1.0),
            RobotAvailable('Robot Stationary?'),
        ])

        complete_handingover_seq = py_trees.composites.Sequence(
            'Finish Handover', memory=True)
        complete_handingover_seq.add_children([
            SendGripperRotate('Reset Gripper Rotation'),
            SetHomePosition('Set Home Position'),
            WaitNode('Wait 1 sec', wait_time=1.0),
            RobotAvailable('Robot Stationary?'),
            CompleteHandingOver('Finish Handover')
        ])

        handover_seq = py_trees.composites.Sequence(
            'Handover', memory=True)
        handover_seq.add_children([
            HandoverActive('Active?'),
            go_to_handover_position_seq,
            execute_handover_seq,
            complete_handingover_seq
        ])

        root.add_children([system_state, handover_seq])

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
            key='handingover',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.handingover = False

        # tick_tree timer
        self.timer = self.create_timer(1.0, self.tick_tree)

        self._start_handingover_srv = self.create_service(
            Void, '/system_skill_handover/execute', self.start_handingover_callback)

        self._abort_handingover_srv = self.create_service(
            Void, '/system_skill_handover/abort', self.abort_handingover_callback)

    def start_handingover_callback(self, request, response):
        self.blackboard.handingover = True

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Handover started.')

        return response

    def abort_handingover_callback(self, request, response):
        self.blackboard.handingover = False

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Handover aborted.')

        return response

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = HandOverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
