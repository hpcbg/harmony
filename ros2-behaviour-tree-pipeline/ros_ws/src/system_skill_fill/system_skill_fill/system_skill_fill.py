import json

import time

import rclpy
from rclpy.node import Node
import py_trees
from rclpy.action import ActionClient

from std_msgs.msg import String

from custom_interfaces.action import FillBottle, MoveRobot
from custom_interfaces.srv import Void


class ReportStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='filling', access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.status_publisher = self.node.create_publisher(
            String, '/system_skill_fill/status_json', 1)

    def update(self):
        msg = String()
        msg.data = json.dumps({'active': self.blackboard.filling})
        self.status_publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class FillActive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='filling', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.filling:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class FillingStationStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='fill_progress',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.fill_progress = 100

        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.status = 'wait'

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.progress_subscriber = self.node.create_subscription(
            String, '/filling_station/progress_json', self.progress_callback, 10
        )
        self.status_subscriber = self.node.create_subscription(
            String, '/filling_station/status_json', self.status_callback, 10
        )

    def progress_callback(self, msg):
        data = json.loads(msg.data)
        self.blackboard.fill_progress = data['progress']

    def status_callback(self, msg):
        data = json.loads(msg.data)
        self.blackboard.status = data['status']

    def update(self):
        return py_trees.common.Status.RUNNING


class SendFillBottleAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, FillBottle, '/filling_station/fill_bottle')

    def update(self):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error('Robot control unavailable!')
            return py_trees.common.Status.FAILURE
        goal_msg = FillBottle.Goal()
        goal_msg.amount_json = json.dumps(100)
        self.node.get_logger().info('Send Action to fill the bottle.')
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


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


class StationAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='status', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.status == 'ok':
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info('Filling Station not available!')
            return py_trees.common.Status.RUNNING


class FillingProgress(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='fill_progress', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.fill_progress >= 100:
            self.logger.info('Bottle is filled.')
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info(
                f'Bottle is filling {self.blackboard.fill_progress}/100')
            return py_trees.common.Status.RUNNING


class SetFillPosition(py_trees.behaviour.Behaviour):
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
        goal_msg.pose_json = json.dumps({'x': 0.4, 'y': 0.4, 'z': 0.4})
        self.node.get_logger().info('Send Action to move to fill position.')
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


class CompleteFilling(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='filling',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        self.blackboard.filling = False
        self.logger.info('Fill completed.')
        return py_trees.common.Status.SUCCESS


class FillNode(Node):
    def __init__(self):
        super().__init__('system_skill_fill')

        # behaviour
        root = py_trees.composites.Parallel(
            name='System Skill Fill',
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            )
        )

        system_state = py_trees.composites.Sequence(
            'System State', memory=True)
        system_state.add_children([
            ReportStatus('Report Status'),
            RobotStatus('Robot Status'),
            FillingStationStatus('Filling Station Status')
        ])

        go_to_filling_seq = py_trees.composites.Sequence(
            'Go to Filling Station', memory=True)
        go_to_filling_seq.add_children([
            RobotAvailable('Robot Stationary?'),
            SetFillPosition('Send Filling Station Position'),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        bottle_fill_seq = py_trees.composites.Sequence(
            'Bottle Fill', memory=True)
        bottle_fill_seq.add_children([
            RobotAvailable('Robot at Fill Position?'),
            SendFillBottleAction('Send Fill Bottle Action'),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        complete_filling_seq = py_trees.composites.Sequence(
            'Finish Fill', memory=True)
        complete_filling_seq.add_children([
            FillingProgress('Bottle Filled?'),
            CompleteFilling('Finish Fill')
        ])

        fill_seq = py_trees.composites.Sequence(
            'Fill', memory=True)
        fill_seq.add_children([
            FillActive('Active?'),
            StationAvailable('Filling Station Ready?'),
            go_to_filling_seq,
            bottle_fill_seq,
            complete_filling_seq
        ])

        root.add_children([system_state, fill_seq])

        # behaviour tree
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(node=self)
        # render the tree
        # py_trees.display.render_dot_tree(
        #     root,
        #     name='system_skill_fill',
        #     target_directory='diagrams'
        # )
        # py_trees.display.render_dot_tree(
        #     root,
        #     name='system_skill_fill_with_bb',
        #     target_directory='diagrams',
        #     with_blackboard_variables=True
        # )

        self.blackboard = py_trees.blackboard.Client(name='Global')
        self.blackboard.register_key(
            key='filling',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.filling = False

        # tick_tree timer
        self.timer = self.create_timer(1.0, self.tick_tree)

        self._start_filling_srv = self.create_service(
            Void, '/system_skill_fill/execute', self.start_filling_callback)

        self._abort_filling_srv = self.create_service(
            Void, '/system_skill_fill/abort', self.abort_filling_callback)

    def start_filling_callback(self, request, response):
        self.blackboard.filling = True

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Fill started.')

        return response

    def abort_filling_callback(self, request, response):
        self.blackboard.filling = False

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Fill aborted.')

        return response

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = FillNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
