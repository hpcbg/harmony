import json

import time

import rclpy
from rclpy.node import Node
import py_trees

from std_msgs.msg import Bool, String, Int32

from custom_interfaces.action import DetectBottle, Move, FillBottle
from rclpy.action import ActionClient
from enum import IntEnum


class Stages(IntEnum):
    ACTIVE = 0
    DETECT_EXECUTE = 1
    DETECT_READY = 2
    PICK_EXECUTE = 3
    PICK_READY = 4
    MOVE_TO_FILL_EXECUTE = 5
    MOVE_TO_FILL_READY = 6
    START_FILL_EXECUTE = 7
    START_FILL_READY = 8
    FILL_EXECUTE = 9
    FILL_READY = 10
    MOVE_TO_CAP_EXECUTE = 11
    MOVE_TO_CAP_READY = 12
    CAP_REQUESTED = 13
    CAP_EXECUTE = 14
    CAP_READY = 15
    HANDOVER_REQUESTED = 16
    HANDOVER_EXECUTE = 17
    HANDOVER_READY = 18
    IDLE = 100


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
            String, '/task_pack_bottle/status', 1)

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
            # self.logger.info('OK')
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
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.READ
        )

    def update(self):
        if self.blackboard.stage == Stages.MOVE_TO_CAP_READY:
            self.blackboard.status = f"Cap: Waiting for a user request..."
        elif self.blackboard.stage == Stages.CAP_READY:
            self.blackboard.status = f"Handover: Waiting for a user request..."
        else:
            self.blackboard.status = f"Waiting for a user request..."
        # self.logger.info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class FillingStationStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='fill_progress',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.fill_progress = 0

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.progress_subscriber = self.node.create_subscription(
            Int32, '/filling_station/progress', self.progress_callback, 10
        )

    def progress_callback(self, msg):
        data = msg.data
        self.blackboard.fill_progress = data

    def update(self):
        return py_trees.common.Status.RUNNING


class UserRequestsMonitor(py_trees.behaviour.Behaviour):
    def __init__(self, name, ):
        super().__init__(name)
        self.start_button_subscriber = None
        self.stop_button_subscriber = None
        self.voice_command_subscriber = None
        self.gesture_command_subscriber = None
        self.prev_voice_command = None
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
        self.voice_command_subscriber = self.node.create_subscription(
            String, '/user_inputs/voice_command', self.voice_command_callback, 1
        )
        self.gesture_command_subscriber = self.node.create_subscription(
            String, '/user_inputs/gesture_command', self.gesture_command_callback, 1
        )

    def start_button_callback(self, msg):
        if msg.data and self.blackboard.stage == Stages.IDLE:
            self.blackboard.stage = Stages.ACTIVE
            self.blackboard.status = "User requested pick"
            # self.logger.info(self.blackboard.status)
        if msg.data and self.blackboard.stage == Stages.MOVE_TO_CAP_READY:
            self.blackboard.stage = Stages.CAP_REQUESTED
            self.blackboard.status = "User requested cap"
            # self.logger.info(self.blackboard.status)
        if msg.data and self.blackboard.stage == Stages.CAP_READY:
            self.blackboard.stage = Stages.HANDOVER_REQUESTED
            self.blackboard.status = "User requested handover"
            # self.logger.info(self.blackboard.status)

    def stop_button_callback(self, msg):
        if msg.data and self.blackboard.stage != Stages.IDLE:
            self.blackboard.stage = Stages.IDLE
            self.blackboard.status = "User aborted the task"
            # self.logger.info(self.blackboard.status)

    def voice_command_callback(self, msg):
        if self.prev_voice_command == "GO":
            if msg.data == "PICK" and self.blackboard.stage == Stages.IDLE:
                self.blackboard.stage = Stages.ACTIVE
                self.blackboard.status = "User requested pick"
                # self.logger.info(self.blackboard.status)
            if msg.data == "CAP" and self.blackboard.stage == Stages.MOVE_TO_CAP_READY:
                self.blackboard.stage = Stages.CAP_REQUESTED
                self.blackboard.status = "User requested cap"
                # self.logger.info(self.blackboard.status)
            if msg.data == "GIVE" and self.blackboard.stage == Stages.CAP_READY:
                self.blackboard.stage = Stages.HANDOVER_REQUESTED
                self.blackboard.status = "User requested handover"
                # self.logger.info(self.blackboard.status)
        self.prev_voice_command = msg.data

    def gesture_command_callback(self, msg):
        if msg.data == "CAP_PLACED" and self.blackboard.stage == Stages.MOVE_TO_CAP_READY:
            self.blackboard.stage = Stages.CAP_REQUESTED
            self.blackboard.status = "User requested cap"
            # self.logger.info(self.blackboard.status)
        if msg.data == "SIDE_GRIP" and self.blackboard.stage == Stages.CAP_READY:
            self.blackboard.stage = Stages.HANDOVER_REQUESTED
            self.blackboard.status = "User requested handover"
            # self.logger.info(self.blackboard.status)

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
                self.blackboard.status = "Bottle was detected"
                # self.logger.info(self.blackboard.status)
                return
            self.blackboard.stage = Stages.ACTIVE
            self.blackboard.status = "Bottle was not detected!"
            # self.logger.info(self.blackboard.status)

    def update(self):
        self.blackboard.status = "Waiting for a bottle to be detected..."
        # self.logger.info(self.blackboard.status)
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
            # self.node.get_logger().error(self.blackboard.status)
            return py_trees.common.Status.FAILURE
        goal_msg = DetectBottle.Goal()
        goal_msg.run = True
        self.blackboard.status = "Send action to run bottle detection."
        # self.node.get_logger().info(self.blackboard.status)
        self.action_client.send_goal_async(goal_msg)
        return py_trees.common.Status.SUCCESS


class RunPickAsync(py_trees.behaviour.Behaviour):
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
            self.node, Move, '/xarm_pack_bottle/pick')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action server is unavailable!"
            # self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = Move.Goal()
        goal_msg.pose_json = json.dumps(
            self.blackboard.pick_and_place)
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        # self.blackboard.status = "Sending goal..."
        # self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state
        # self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Action was rejected!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                # self.blackboard.status = "Pick goal was accepted."
                # self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    # self.blackboard.status = self.result.message
                    # self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.PICK_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Action failed: {self.result.message}!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.PICK_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.PICK_READY
        # self.blackboard.status = "Pick is ready."
        # self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class RunGoToFillAsync(py_trees.behaviour.Behaviour):
    """
    Non-blocking py_trees behaviour to call Fill action asynchronously.
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
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, Move, '/xarm_pack_bottle/fill')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action server is unavailable!"
            # self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = Move.Goal()
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        # self.blackboard.status = "Sending goal..."
        # self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state
        # self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Action was rejected!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                # self.blackboard.status = "Fill goal was accepted."
                # self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    # self.blackboard.status = self.result.message
                    # self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.MOVE_TO_FILL_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Action failed: {self.result.message}!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.MOVE_TO_FILL_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.MOVE_TO_FILL_READY
        # self.blackboard.status = "Move to Fill is ready."
        # self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class RunSendFillActionAsync(py_trees.behaviour.Behaviour):
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
            self.node, FillBottle, '/filling_station/fill_bottle')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action server is unavailable!"
            # self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = FillBottle.Goal()
        goal_msg.amount_json = json.dumps(100)
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        # self.blackboard.status = "Sending goal..."
        # self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state
        # self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Action was rejected!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                # self.blackboard.status = "Pick goal was accepted."
                # self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    # self.blackboard.status = f"Fill action completed"
                    # self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.START_FILL_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = "Action failed!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.START_FILL_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.START_FILL_READY
        # self.blackboard.status = "Fill is ready."
        # self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class WaitFilling(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='fill_progress', access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        if self.blackboard.fill_progress >= 100:
            # self.blackboard.status = "Bottle is full"
            # self.logger.info(self.blackboard.status)
            self.blackboard.stage = Stages.FILL_READY
            return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.status = f"Bottle is filling {self.blackboard.fill_progress}/100"
            # self.logger.info(self.blackboard.status)
            self.blackboard.stage = Stages.FILL_EXECUTE

            return py_trees.common.Status.RUNNING


class RunGoToCapAsync(py_trees.behaviour.Behaviour):
    """
    Non-blocking py_trees behaviour to call Fill action asynchronously.
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
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, Move, '/xarm_pack_bottle/move_to_cap')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action action server is unavailable!"
            # self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = Move.Goal()
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        # self.blackboard.status = "Sending goal..."
        # self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state
        # self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Action was rejected!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                # self.blackboard.status = "Move to cap goal was accepted."
                # self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    # self.blackboard.status = self.result.message
                    # self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.MOVE_TO_CAP_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Action failed: {self.result.message}!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.MOVE_TO_CAP_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.MOVE_TO_CAP_READY
        # self.blackboard.status = "Move to cap is ready."
        # self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class RunCapAsync(py_trees.behaviour.Behaviour):
    """
    Non-blocking py_trees behaviour to call Fill action asynchronously.
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
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, Move, '/xarm_pack_bottle/cap')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action server is unavailable!"
            # self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = Move.Goal()
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        # self.blackboard.status = "Sending goal..."
        # self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state
        # self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Action was rejected!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                # self.blackboard.status = "Cap goal was accepted."
                # self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    # self.blackboard.status = self.result.message
                    # self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.CAP_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Action failed: {self.result.message}!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.CAP_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.CAP_READY
        # self.blackboard.status = "Cap is ready."
        # self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class RunHandoverAsync(py_trees.behaviour.Behaviour):
    """
    Non-blocking py_trees behaviour to call Fill action asynchronously.
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
            key='status',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, Move, '/xarm_pack_bottle/handover')

    def initialise(self):
        # Reset state
        self.goal_handle = None
        self.result = None
        self.status_sent = False

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action server is unavailable!"
            # self.node.get_logger().error(self.blackboard.status)
            self.result = None
            return

        # Send goal
        goal_msg = Move.Goal()
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        # self.blackboard.status = "Sending goal..."
        # self.node.get_logger().info(self.blackboard.status)

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state
        # self.node.get_logger().info(self.blackboard.status)

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE
        # Goal handle not yet ready
        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Action was rejected!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
                # Get result future
                self.result_future = self.goal_handle.get_result_async()
                # self.blackboard.status = "handover goal was accepted."
                # self.node.get_logger().info(self.blackboard.status)
            return py_trees.common.Status.RUNNING

        # Result not yet ready
        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result
                if self.result.success:
                    # self.blackboard.status = self.result.message
                    # self.node.get_logger().info(self.blackboard.status)
                    self.blackboard.stage = Stages.HANDOVER_READY
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Action failed: {self.result.message}!"
                    # self.node.get_logger().error(self.blackboard.status)
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = Stages.HANDOVER_EXECUTE
                return py_trees.common.Status.RUNNING

        # Already finished
        self.blackboard.stage = Stages.HANDOVER_READY
        self.blackboard.status = "handover is ready."
        # self.node.get_logger().info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class CompletePackBottle(py_trees.behaviour.Behaviour):
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
        self.blackboard.status = "Pack bottle completed."
        # self.logger.info(self.blackboard.status)
        return py_trees.common.Status.SUCCESS


class TaskPackBottleNode(Node):
    def __init__(self):
        super().__init__('task_pack_bottle')

        # behaviour
        root = py_trees.composites.Parallel(
            name='Task Pack Bottle',
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
            FillingStationStatus('Filling Station Status')
        ])

        detect_seq = py_trees.composites.Sequence(
            'Detect', memory=True)
        detect_seq.add_children([
            StageCheck('Pick Requested?', Stages.ACTIVE),
            RunBottleDetection('Run Bottle Detection')
        ])

        pick_seq = py_trees.composites.Sequence(
            'Pick', memory=True)
        pick_seq.add_children([
            StageCheck('Bottle Detected?', Stages.DETECT_READY),
            RunPickAsync('Run Pick')
        ])

        go_to_fill_seq = py_trees.composites.Sequence(
            'Go to Fill', memory=True)
        go_to_fill_seq.add_children([
            StageCheck('Pick Ready?', Stages.PICK_READY),
            RunGoToFillAsync('Move to Fill')
        ])

        initiate_fill_seq = py_trees.composites.Sequence(
            'Initiate Fill', memory=True)
        initiate_fill_seq.add_children([
            StageCheck('At Fill?', Stages.MOVE_TO_FILL_READY),
            RunSendFillActionAsync('Start Fill')
        ])

        wait_to_fill_seq = py_trees.composites.Sequence(
            'Wait to Fill', memory=True)
        wait_to_fill_seq.add_children([
            StageCheck('Fill Started?', Stages.START_FILL_READY),
            WaitFilling('Wait to Fill')
        ])

        go_to_cap_seq = py_trees.composites.Sequence(
            'Go to Cap', memory=True)
        go_to_cap_seq.add_children([
            StageCheck('Fill Completed?', Stages.FILL_READY),
            RunGoToCapAsync('Move to Cap')
        ])

        cap_seq = py_trees.composites.Sequence(
            'Cap', memory=True)
        cap_seq.add_children([
            StageCheck('Cap Requested?', Stages.CAP_REQUESTED),
            RunCapAsync('Start Cap')
        ])

        handover_seq = py_trees.composites.Sequence(
            'Handover', memory=True)
        handover_seq.add_children([
            StageCheck('Handover Requested?', Stages.HANDOVER_REQUESTED),
            RunHandoverAsync('Start Handover')
        ])

        complete_seq = py_trees.composites.Sequence(
            'Finish Task', memory=True)
        complete_seq.add_children([
            StageCheck('Handover Executed?',
                       Stages.HANDOVER_READY),
            CompletePackBottle('Finish Pack Bottle')
        ])

        workflow_seq = py_trees.composites.Selector(
            'Workflow', memory=True)
        workflow_seq.add_children([
            detect_seq,
            pick_seq,
            go_to_fill_seq,
            initiate_fill_seq,
            wait_to_fill_seq,
            go_to_cap_seq,
            cap_seq,
            handover_seq,
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
        py_trees.display.render_dot_tree(
            root,
            name='task_pack_bottle',
            target_directory='../diagrams'
        )
        # py_trees.display.render_dot_tree(
        #     root,
        #     name='task_pack_bottle_with_bb',
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
    node = TaskPackBottleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
