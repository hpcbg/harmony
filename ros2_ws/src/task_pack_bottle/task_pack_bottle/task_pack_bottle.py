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
            return py_trees.common.Status.SUCCESS
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
    def __init__(self, name):
        super().__init__(name)
        self.prev_voice_command = None
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            key='status', access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.create_subscription(
            Bool, '/user_inputs/start_button', self.start_button_callback, 1)
        self.node.create_subscription(
            Bool, '/user_inputs/stop_button', self.stop_button_callback, 1)
        self.node.create_subscription(
            String, '/user_inputs/voice_command', self.voice_command_callback, 1)
        self.node.create_subscription(
            String, '/user_inputs/gesture_command', self.gesture_command_callback, 1)

    def _transition(self, new_stage, status_msg):
        self.blackboard.stage = new_stage
        self.blackboard.status = status_msg

    def start_button_callback(self, msg):
        if not msg.data:
            return
        stage = self.blackboard.stage
        if stage == Stages.IDLE:
            self._transition(Stages.ACTIVE, "User requested pick")
        elif stage == Stages.MOVE_TO_CAP_READY:
            self._transition(Stages.CAP_REQUESTED, "User requested cap")
        elif stage == Stages.CAP_READY:
            self._transition(Stages.HANDOVER_REQUESTED,
                             "User requested handover")

    def stop_button_callback(self, msg):
        if msg.data and self.blackboard.stage != Stages.IDLE:
            self._transition(Stages.IDLE, "User aborted the task")

    def voice_command_callback(self, msg):
        if self.prev_voice_command == "GO":
            stage = self.blackboard.stage
            if msg.data == "PICK" and stage == Stages.IDLE:
                self._transition(Stages.ACTIVE, "User requested pick")
            elif msg.data == "CAP" and stage == Stages.MOVE_TO_CAP_READY:
                self._transition(Stages.CAP_REQUESTED, "User requested cap")
            elif msg.data == "GIVE" and stage == Stages.CAP_READY:
                self._transition(Stages.HANDOVER_REQUESTED,
                                 "User requested handover")
            elif msg.data == "STOP" and stage != Stages.IDLE:
                self._transition(Stages.IDLE, "User aborted the task")
        self.prev_voice_command = msg.data

    def gesture_command_callback(self, msg):
        if msg.data == "CAP_PLACED" and self.blackboard.stage == Stages.MOVE_TO_CAP_READY:
            self._transition(Stages.CAP_REQUESTED, "User requested cap")
        elif msg.data == "SIDE_GRIP" and self.blackboard.stage == Stages.CAP_READY:
            self._transition(Stages.HANDOVER_REQUESTED,
                             "User requested handover")

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
            key='pick_pose',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.pick_pose = {}

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscriber = self.node.create_subscription(
            String, '/bottle_detection/job_json', self.callback, 1
        )

    def callback(self, msg):
        if self.blackboard.stage != Stages.DETECT_EXECUTE:
            return
        job = json.loads(msg.data)

        if "status" in job and job["status"] == "DONE":
            if "x" in job["pick_pose"]:
                self.blackboard.stage = Stages.DETECT_READY
                self.blackboard.pick_pose = job["pick_pose"]
                self.blackboard.status = "Bottle was detected"
            else:
                self.blackboard.stage = Stages.ACTIVE
                self.blackboard.status = "Bottle was not detected!"

    def update(self):
        self.blackboard.status = "Waiting for a bottle to be detected..."
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
        self.run_command_publisher = self.node.create_publisher(
            String, '/bottle_detection/command', 1)

    def update(self):
        self.blackboard.stage = Stages.DETECT_EXECUTE
        msg = String()
        msg.data = "START"
        self.run_command_publisher.publish(msg)
        self.blackboard.status = "Detecting bottle..."
        return py_trees.common.Status.SUCCESS


class RunActionAsync(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        action_type,
        action_name,
        goal_builder,
        execute_stage,
        ready_stage,
        extra_keys=None
    ):
        super().__init__(name)

        self.action_type = action_type
        self.action_name = action_name
        self.goal_builder = goal_builder
        self.execute_stage = execute_stage
        self.ready_stage = ready_stage

        self.action_client = None
        self.node = None

        self.goal_handle = None
        self.result = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            'stage', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            'status', access=py_trees.common.Access.WRITE)

        if extra_keys:
            for key, access in extra_keys:
                self.blackboard.register_key(key, access=access)

        if hasattr(goal_builder, "required_keys"):
            for key, access in goal_builder.required_keys:
                self.blackboard.register_key(key, access=access)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(
            self.node, self.action_type, self.action_name
        )

    def initialise(self):
        self.goal_handle = None
        self.result = None

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.blackboard.status = "Action server unavailable!"
            return

        goal_msg = self.goal_builder(self.blackboard)
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb
        )

    def feedback_cb(self, feedback_msg):
        self.blackboard.status = feedback_msg.feedback.state

    def update(self):
        if self.blackboard.stage == Stages.IDLE:
            return py_trees.common.Status.FAILURE

        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.blackboard.status = "Goal rejected!"
                    return py_trees.common.Status.FAILURE

                self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result().result

                if self.result.success:
                    self.blackboard.stage = self.ready_stage
                    return py_trees.common.Status.SUCCESS
                else:
                    self.blackboard.status = f"Action failed: {self.result.message}!"
                    return py_trees.common.Status.FAILURE
            else:
                self.blackboard.stage = self.execute_stage
                return py_trees.common.Status.RUNNING

        self.blackboard.stage = self.ready_stage
        return py_trees.common.Status.SUCCESS


def goal(fn, required_keys=None):
    fn.required_keys = required_keys or []
    return fn


def EMPTY_MOVE_GOAL(bb): return Move.Goal()


def RunPickAsync(name): return RunActionAsync(
    name,
    Move,
    '/xarm_pack_bottle/pick',
    goal(
        lambda bb: Move.Goal(
            pose_json=json.dumps(bb.pick_pose)
        ),
        required_keys=[
            ('pick_pose', py_trees.common.Access.READ)
        ]
    ),
    Stages.PICK_EXECUTE,
    Stages.PICK_READY
)


def RunGoToFillAsync(name): return RunActionAsync(
    name,
    Move,
    '/xarm_pack_bottle/fill',
    EMPTY_MOVE_GOAL,
    Stages.MOVE_TO_FILL_EXECUTE,
    Stages.MOVE_TO_FILL_READY
)


def RunSendFillActionAsync(name): return RunActionAsync(
    name,
    FillBottle,
    '/filling_station/fill_bottle',
    lambda _: FillBottle.Goal(
        amount_json=json.dumps(100)
    ),
    Stages.START_FILL_EXECUTE,
    Stages.START_FILL_READY
)


def RunGoToCapAsync(name): return RunActionAsync(
    name,
    Move,
    '/xarm_pack_bottle/move_to_cap',
    EMPTY_MOVE_GOAL,
    Stages.MOVE_TO_CAP_EXECUTE,
    Stages.MOVE_TO_CAP_READY
)


def RunCapAsync(name): return RunActionAsync(
    name,
    Move,
    '/xarm_pack_bottle/cap',
    EMPTY_MOVE_GOAL,
    Stages.CAP_EXECUTE,
    Stages.CAP_READY
)


def RunHandoverAsync(name): return RunActionAsync(
    name,
    Move,
    '/xarm_pack_bottle/handover',
    EMPTY_MOVE_GOAL,
    Stages.HANDOVER_EXECUTE,
    Stages.HANDOVER_READY
)


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
            self.blackboard.stage = Stages.FILL_READY
            return py_trees.common.Status.SUCCESS
        else:
            self.blackboard.status = f"Bottle is filling {self.blackboard.fill_progress}/100"
            self.blackboard.stage = Stages.FILL_EXECUTE

            return py_trees.common.Status.RUNNING


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
        self.blackboard.status = "Pack bottle: Completed"
        return py_trees.common.Status.SUCCESS


class TaskPackBottleNode(Node):
    def __init__(self):
        super().__init__('task_pack_bottle')

        # behaviour
        root = py_trees.composites.Parallel(
            name="Task Pack Bottle",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            )
        )

        system_monitor = py_trees.composites.Sequence(
            "System Monitor", memory=True)
        system_monitor.add_children([
            ReportStatus("Report Status"),
            BottleDetectorStatus("Bottle Detector Status"),
            UserRequestsMonitor("User Requests Monitor"),
            FillingStationStatus("Filling Station Status")
        ])

        # --- workflow sequences ---
        def make_seq(name, precondition_name, precondition_stage, behaviour):
            seq = py_trees.composites.Sequence(name, memory=True)
            seq.add_children(
                [StageCheck(precondition_name, precondition_stage), behaviour])
            return seq

        detect_seq = make_seq(
            "Detect",
            "Pick Requested?", Stages.ACTIVE,
            RunBottleDetection("Run Bottle Detection"))

        pick_seq = make_seq(
            "Pick",
            "Bottle Detected?", Stages.DETECT_READY,
            RunPickAsync("Run Pick"))

        go_to_fill_seq = make_seq(
            "Go to Fill",
            "Pick Ready?", Stages.PICK_READY,
            RunGoToFillAsync("Move to Fill"))

        initiate_fill_seq = make_seq(
            "Initiate Fill",
            "At Fill?", Stages.MOVE_TO_FILL_READY,
            RunSendFillActionAsync("Start Fill"))

        wait_to_fill_seq = make_seq(
            "Fill",
            "Fill Started?", Stages.START_FILL_READY,
            WaitFilling("Wait Filling"))

        go_to_cap_seq = make_seq(
            "Go to Cap",
            "Fill Completed?", Stages.FILL_READY,
            RunGoToCapAsync("Move to Cap"))

        cap_seq = make_seq(
            "Cap",
            "Cap Requested?", Stages.CAP_REQUESTED,
            RunCapAsync("Start Cap"))

        handover_seq = make_seq(
            "Handover",
            "Handover Requested?", Stages.HANDOVER_REQUESTED,
            RunHandoverAsync("Start Handover"))

        complete_seq = make_seq(
            "Finish Task",
            "Handover Executed?", Stages.HANDOVER_READY,
            CompletePackBottle("Finish Pack Bottle"))

        workflow_seq = py_trees.composites.Selector(
            "Workflow", memory=True)
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
            IdleStatus("Idle Status")
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
        #     name='task_pack_bottle',
        #     target_directory='../diagrams'
        # )
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
        self.timer = self.create_timer(0.5, self.tick_tree)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = TaskPackBottleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
