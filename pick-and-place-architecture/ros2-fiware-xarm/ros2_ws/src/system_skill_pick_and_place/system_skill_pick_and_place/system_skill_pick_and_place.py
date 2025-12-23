import json

import time

import rclpy
from rclpy.node import Node
import py_trees

from std_msgs.msg import Bool, String

from custom_interfaces.action import DetectBottle, PickAndPlace
from rclpy.action import ActionClient
from enum import IntEnum


class Stages(IntEnum):
    ACTIVE = 0
    DETECT_EXECUTE = 1
    DETECT_READY = 2
    PICK_AND_PLACE_EXECUTE = 3
    PICK_AND_PLACE_READY = 4
    IDLE = 10


class ReportStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='status', access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.status_publisher = self.node.create_publisher(
            String, '/system_skill_pick_and_place/status', 1)

    def update(self):
        msg = String()
        msg.data = self.blackboard.status
        self.status_publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class StageCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name, stage):
        super().__init__(name)
        self.stage = stage
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.stage == self.stage:
            self.logger.info('OK')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class IdleStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        self.blackboard.status = "Waiting for user request..."
        self.logger.info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class UserRequestsMonitor(py_trees.behaviour.Behaviour):
    def __init__(self, name, ):
        super().__init__(name)
        self.start_button_subscriber = None
        self.stop_button_subscriber = None
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.start_button_subscriber = self.node.create_subscription(
            Bool, '/user_inputs/start_button', self.start_button_callback, 1
        )
        self.stop_button_subscriber = self.node.create_subscription(
            Bool, '/user_inputs/stop_button', self.stop_button_callback, 1
        )

    def start_button_callback(self, msg):
        if msg.data and self.blackboard.stage == Stages.IDLE:
            self.blackboard.stage = Stages.ACTIVE
            self.blackboard.status = "User started Pick and Place system skill."
            self.logger.info(self.blackboard.status)

    def stop_button_callback(self, msg):
        if msg.data and self.blackboard.stage != Stages.IDLE:
            self.blackboard.stage = Stages.IDLE
            self.blackboard.status = "User aborted Pick and Place system skill."
            self.logger.info(self.blackboard.status)

    def update(self):
        return py_trees.common.Status.RUNNING


class BottleDetectorStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name, ):
        super().__init__(name)
        self.subscriber = None
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='pick_and_place',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.pick_and_place = {}

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscriber = self.node.create_subscription(
            String, '/bottle_detector_brigde/pick_and_place', self.callback, 1
        )

    def callback(self, msg):
        pick_place = json.loads(msg.data)
        if self.blackboard.stage == Stages.DETECT_EXECUTE:
            if 'pick_pose' in pick_place:
                self.blackboard.stage = Stages.DETECT_READY
                self.blackboard.pick_and_place = pick_place
                self.blackboard.status = "Bottle was detected."
                self.logger.info(self.blackboard.status)
                return
            self.blackboard.stage = Stages.ACTIVE
            self.blackboard.status = "Bottle was not detected!"
            self.logger.info(self.blackboard.status)

    def update(self):
        self.blackboard.status = "Waiting for a bottle to be detected..."
        self.logger.info(self.blackboard.status)
        return py_trees.common.Status.RUNNING


class RunBottleDetection(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.action_client = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, DetectBottle, '/bottle_detector_bridge/detect_bottle')

    def update(self):
        self.blackboard.stage = Stages.DETECT_EXECUTE
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Bottle detection is unavailable!"
            self.node.get_logger().error(self.blackboard.status)
            return py_trees.common.Status.FAILURE
        goal_msg = DetectBottle.Goal()
        goal_msg.run = True
        self.blackboard.status = "Send action to run bottle detection."
        self.node.get_logger().info(self.blackboard.status)
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


class RunPickPlaceAsync(py_trees.behaviour.Behaviour):
    """
    Non-blocking py_trees behaviour to call PickPlace action asynchronously.
    While action is running, returns RUNNING.
    After completion, sets blackboard result and returns SUCCESS/FAILURE.
    """

    def __init__(self, name):
        super().__init__(name)
        self.action_client = None
        self.blackboard = py_trees.blackboard.Client(name=name)

        self.goal_handle = None
        self.result = None
        self.status_sent = False
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='pick_and_place',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, PickAndPlace, '/xarm_pick_and_place/pick_and_place')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Pick and Place action server is unavailable!"
            self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = PickAndPlace.Goal()
        goal_msg.pick_and_place_json = json.dumps(
            self.blackboard.pick_and_place)
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        self.blackboard.status = "Sending goal..."
        self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = f"Pick and Place action feedback: {feedback_msg.feedback.state}"
        self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Pick and Place goal was rejected"
                    self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                self.blackboard.status = "Pick and Place goal was accepted."
                self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    self.blackboard.status = f"Pick and Place action completed: {self.result.message}"
                    self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.PICK_AND_PLACE_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Pick and Place Action failed: {self.result.message}"
                    self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.PICK_AND_PLACE_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.PICK_AND_PLACE_READY
        self.blackboard.status = "Pick and Place is running."
        self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class WaitNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, wait_time: float):
        super().__init__(name)
        self.wait_time = wait_time
        self.start_time = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        self.start_time = time.time()
        self.blackboard.status = f'Waiting for {self.wait_time:.1f} seconds...'
        self.logger.info(self.blackboard.status)

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed < self.wait_time:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


class CompletePickAndPlace(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        self.blackboard.stage = Stages.IDLE
        self.blackboard.status = "Pick and Place system skill completed."
        self.logger.info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('system_skill_pick_and_place')

        # behaviour
        root = py_trees.composites.Parallel(
            name='System Skill Pick and Place',
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            )
        )

        system_monitor = py_trees.composites.Sequence(
            'System Monitor', memory=True)
        system_monitor.add_children([
            ReportStatus('Report Status'),
            BottleDetectorStatus('Bottle Detector Status'),
            UserRequestsMonitor('User Requests Monitor'),
        ])

        detect_seq = py_trees.composites.Sequence(
            'Detect', memory=True)
        detect_seq.add_children([
            StageCheck('Pick Requested?', Stages.ACTIVE),
            RunBottleDetection('Run Bottle Detection'),
            WaitNode('Wait 1 sec', wait_time=1.0)
        ])

        pick_and_place_seq = py_trees.composites.Sequence(
            'Pick and Place', memory=True)
        pick_and_place_seq.add_children([
            StageCheck('Bottle Detected?', Stages.DETECT_READY),
            RunPickPlaceAsync('Run Pick and Place'),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        complete_seq = py_trees.composites.Sequence(
            'Finish System Skill', memory=True)
        complete_seq.add_children([
            StageCheck('Pick and Place Executed?',
                       Stages.PICK_AND_PLACE_READY),
            CompletePickAndPlace('Finish Pick and Place')
        ])

        workflow_seq = py_trees.composites.Selector(
            'Workflow', memory=True)
        workflow_seq.add_children([
            detect_seq,
            pick_and_place_seq,
            complete_seq,
            IdleStatus('Idle Status')
        ])

        root.add_children([
            system_monitor,
            workflow_seq
        ])

        # behaviour tree
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(node=self)
        # render the tree
        # py_trees.display.render_dot_tree(
        #     root,
        #     name='system_skill_pick_and_place',
        #     target_directory='../diagrams'
        # )
        # py_trees.display.render_dot_tree(
        #     root,
        #     name='system_skill_pick_and_place_with_bb',
        #     target_directory='../diagrams',
        #     with_blackboard_variables=True
        # )

        self.blackboard = py_trees.blackboard.Client(name='Global')
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.stage = Stages.IDLE
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.status = ""

        # tick_tree timer
        self.timer = self.create_timer(1.0, self.tick_tree)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
