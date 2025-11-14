import json

import time

import rclpy
from rclpy.node import Node
import py_trees

from std_msgs.msg import String

from custom_interfaces.srv import Void

from enum import IntEnum


class Stages(IntEnum):
    ACTIVE = 0
    PICK_EXECUTE = 1
    PICK_READY = 2
    FILL_EXECUTE = 3
    FILL_READY = 4
    CAP_EXECUTE = 5
    CAP_READY = 6
    HANDOVER_EXECUTE = 7
    HANDOVER_READY = 8
    ABORT = 10


class ReportStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='active', access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.status_publisher = self.node.create_publisher(
            String, '/task_pack_bottle/status_json', 1)

    def update(self):
        msg = String()
        msg.data = json.dumps({'active': self.blackboard.active})
        self.status_publisher.publish(msg)
        return py_trees.common.Status.RUNNING


class PackBottleActive(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='active', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.active:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class SystemCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='system_status', access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.system_status == 'ok':
            self.logger.info('System is OK.')
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info('System Error!')
            return py_trees.common.Status.FAILURE


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
            key='stage', access=py_trees.common.Access.READ)

    def update(self):
        self.logger.info(f'Stage {self.blackboard.stage.name} running...')
        return py_trees.common.Status.SUCCESS


class SystemMonitorStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name, ):
        super().__init__(name)
        self.subscriber = None
        self.node = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='system_status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.system_status = 'unknown'

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.subscriber = self.node.create_subscription(
            String, '/system_monitor/status_json', self.callback, 10
        )

    def callback(self, msg):
        data = json.loads(msg.data)
        self.blackboard.system_status = data

    def update(self):
        return py_trees.common.Status.RUNNING


class SystemSkillsStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.node = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='pick_status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.pick_status = {'status': 'unknown'}

        self.blackboard.register_key(
            key='fill_status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.fill_status = {'status': 'unknown'}

        self.blackboard.register_key(
            key='cap_status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.cap_status = {'status': 'unknown'}

        self.blackboard.register_key(
            key='handover_status',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.handover_status = {'status': 'unknown'}

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.pick_subscriber = self.node.create_subscription(
            String, '/system_skill_pick/status_json', self.pick_callback, 10
        )
        self.fill_subscriber = self.node.create_subscription(
            String, '/system_skill_fill/status_json', self.fill_callback, 10
        )
        self.cap_subscriber = self.node.create_subscription(
            String, '/system_skill_cap/status_json', self.cap_callback, 10
        )
        self.handover_subscriber = self.node.create_subscription(
            String, '/system_skill_handover/status_json', self.handover_callback, 10
        )

    def pick_callback(self, msg):
        self.blackboard.pick_status = json.loads(msg.data)
        if self.blackboard.pick_status['active'] == False and self.blackboard.stage == Stages.PICK_EXECUTE:
            self.blackboard.stage = Stages.PICK_READY

    def fill_callback(self, msg):
        self.blackboard.fill_status = json.loads(msg.data)
        if self.blackboard.fill_status['active'] == False and self.blackboard.stage == Stages.FILL_EXECUTE:
            self.blackboard.stage = Stages.FILL_READY

    def cap_callback(self, msg):
        self.blackboard.cap_status = json.loads(msg.data)
        if self.blackboard.cap_status['active'] == False and self.blackboard.stage == Stages.CAP_EXECUTE:
            self.blackboard.stage = Stages.CAP_READY

    def handover_callback(self, msg):
        self.blackboard.handover_status = json.loads(msg.data)
        if self.blackboard.handover_status['active'] == False and self.blackboard.stage == Stages.HANDOVER_EXECUTE:
            self.blackboard.stage = Stages.HANDOVER_READY

    def update(self):
        return py_trees.common.Status.RUNNING


class CallService(py_trees.behaviour.Behaviour):
    def __init__(self, name, service, stage):
        super().__init__(name)
        self.node = None
        self.service = service
        self.stage = stage
        self.client = None
        self.future = None
        self.called = False

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.node = rclpy.create_node(f'service_caller_stage_{self.stage}')

        self.client = self.node.create_client(Void, self.service)

    def update(self):
        if self.stage == Stages.ABORT or self.blackboard.stage < self.stage:
            if not self.client.service_is_ready():
                self.logger.warning(f'Service {self.service} unavailable!')
                return py_trees.common.Status.FAILURE
            if self.future is None:
                request = Void.Request()
                self.future = self.client.call_async(request)
                return py_trees.common.Status.RUNNING
            if self.future.done():
                if self.future.result() is not None:
                    self.logger.info(f'Service {self.service} called.')
                    self.blackboard.stage = self.stage
                    self.future = None
                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.error(
                        f'Service {self.service} error: {self.future.exception()}')
                    return py_trees.common.Status.FAILURE
            self.logger.info(f'Service {self.service} running...')
            return py_trees.common.Status.RUNNING
        self.logger.info('Service failure!')
        return py_trees.common.Status.FAILURE


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


class CompletePackBottle(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='active',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        self.blackboard.active = False
        self.logger.info('Pack Bottle completed.')
        return py_trees.common.Status.SUCCESS


class PackBottleNode(Node):
    def __init__(self):
        super().__init__('task_pack_bottle')

        # behaviour
        root = py_trees.composites.Parallel(
            name='Task Pack Bottle',
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            )
        )

        system_state = py_trees.composites.Sequence(
            'System State', memory=True)
        system_state.add_children([
            ReportStatus('Report Status'),
            SystemSkillsStatus('System Skills Status'),
            SystemMonitorStatus('System Monitor Status'),
        ])

        pick_seq = py_trees.composites.Sequence(
            'Pick', memory=True)
        pick_seq.add_children([
            StageCheck('System Active?', Stages.ACTIVE),
            CallService('Call Pick', '/system_skill_pick/execute',
                        Stages.PICK_EXECUTE),
            WaitNode('Wait 1 sec', wait_time=1.0)
        ])

        fill_seq = py_trees.composites.Sequence(
            'Fill', memory=True)
        fill_seq.add_children([
            StageCheck('Pick Ready?', Stages.PICK_READY),
            CallService('Call Fill', '/system_skill_fill/execute',
                        Stages.FILL_EXECUTE),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        cap_seq = py_trees.composites.Sequence(
            'Cap', memory=True)
        cap_seq.add_children([
            StageCheck('Fill Ready?', Stages.FILL_READY),
            CallService('Call Cap', '/system_skill_cap/execute',
                        Stages.CAP_EXECUTE),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        handover_seq = py_trees.composites.Sequence(
            'Handover', memory=True)
        handover_seq.add_children([
            StageCheck('Cap Ready?', Stages.CAP_READY),
            CallService(
                'Call Handover', '/system_skill_handover/execute', Stages.HANDOVER_EXECUTE),
            WaitNode('Wait 1 sec', wait_time=1.0),
        ])

        complete_seq = py_trees.composites.Sequence(
            'Finish Pack Bottle', memory=True)
        complete_seq.add_children([
            StageCheck('Handover Ready?', Stages.HANDOVER_READY),
            CompletePackBottle('Finish Pack Bottle')
        ])

        system_skills_seq = py_trees.composites.Selector(
            'System Skills', memory=True)
        system_skills_seq.add_children([
            pick_seq,
            fill_seq,
            cap_seq,
            handover_seq,
            complete_seq,
            IdleStatus('Idle Status')
        ])

        pack_bottle_seq = py_trees.composites.Sequence(
            'Pack Bottle', memory=True)
        pack_bottle_seq.add_children([
            PackBottleActive('Active?'),
            system_skills_seq
        ])

        workflow_seq = py_trees.composites.Sequence(
            'Workflow', memory=True)
        workflow_seq.add_children([
            SystemCheck("System OK?"),
            pack_bottle_seq
        ])

        fallback_seq = py_trees.composites.Sequence(
            'Fallback', memory=True)
        fallback_seq.add_children([
            PackBottleActive('Active?'),
            CallService(
                'Abort Pick', '/system_skill_pick/abort', Stages.ABORT),
            CallService(
                'Abort Fill', '/system_skill_fill/abort', Stages.ABORT),
            CallService(
                'Abort Cap', '/system_skill_cap/abort', Stages.ABORT),
            CallService(
                'Abort Handover', '/system_skill_handover/abort', Stages.ABORT),
            CompletePackBottle('Abort Pack Bottle')
        ])

        system_monitor_sel = py_trees.composites.Selector(
            'System Monitor', memory=True)
        system_monitor_sel.add_children([
            workflow_seq,
            fallback_seq,
        ])

        root.add_children([
            system_state,
            system_monitor_sel
        ])

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
            key='active',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.active = False
        self.blackboard.register_key(
            key='stage',
            access=py_trees.common.Access.WRITE
        )
        self.blackboard.stage = Stages.ACTIVE

        # tick_tree timer
        self.timer = self.create_timer(1.0, self.tick_tree)

        self._start_pack_bottle_srv = self.create_service(
            Void, '/task_pack_bottle/execute', self.start_pack_bottle_callback)

        self._abort_pack_bottle_srv = self.create_service(
            Void, '/task_pack_bottle/abort', self.abort_pack_bottle_callback)

    def start_pack_bottle_callback(self, request, response):
        self.blackboard.stage = Stages.ACTIVE
        self.blackboard.active = True

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Pack Bottle started.')

        return response

    def abort_pack_bottle_callback(self, request, response):
        self.blackboard.active = False

        status = {
            'status': 'OK'
        }
        response.status = json.dumps(status)

        self.get_logger().info('Pack Bottle aborted.')

        return response

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = PackBottleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
