"""Adapter for real bosdyn SDK protobufs and common synchronous client methods.

This is an application-level adapter, not an emulation of Spot's gRPC endpoint.
Use RobotCommandBuilder as usual and replace robot creation with SpotSimRobot.
"""
import time
from bosdyn.api import basic_command_pb2, robot_command_pb2, robot_state_pb2, header_pb2
from .client import BridgeClient


class SimCommandClient:
    default_service_name = 'robot-command'

    def __init__(self, bridge):
        self.bridge = bridge

    def robot_command(self, command, end_time_secs=None, timesync_endpoint=None,
                      lease=None, timeout=None, **kwargs):
        if lease is not None or timesync_endpoint is not None or kwargs:
            raise NotImplementedError('Simulation has no lease or remote time-sync service')
        which = command.WhichOneof('command')
        if which == 'full_body_command':
            kind = command.full_body_command.WhichOneof('command')
            if kind == 'stop_request':
                return self.bridge.command('stop')
            raise NotImplementedError(f'Unsupported full-body command: {kind}')
        if which != 'synchronized_command':
            raise ValueError('Expected a synchronized mobility command')
        sync = command.synchronized_command
        if sync.HasField('arm_command') or sync.HasField('gripper_command'):
            raise NotImplementedError('The simulated Spot has no arm or gripper')
        mobility = sync.mobility_command
        # Default RobotCommandBuilder params carry Spot-specific hints. Only the
        # nominal body pose is supported. Reject non-default motion parameters.
        if mobility.HasField('params'):
            from bosdyn.api.spot import robot_command_pb2 as spot_pb2
            params = spot_pb2.MobilityParams()
            if not mobility.params.Unpack(params):
                raise NotImplementedError('Unknown mobility params type')
            # RobotCommandBuilder defaults include locomotion_hint=HINT_AUTO,
            # a zero body offset and identity orientation. Compare against the
            # builder's own nominal params, rather than assuming wire defaults.
            from bosdyn.client.robot_command import RobotCommandBuilder
            nominal_any = RobotCommandBuilder.synchro_stand_command().synchronized_command.mobility_command.params
            nominal = spot_pb2.MobilityParams()
            nominal_any.Unpack(nominal)
            if params != nominal:
                raise NotImplementedError('Only default mobility params / nominal body pose are supported')
        kind = mobility.WhichOneof('command')
        if kind in ('stand_request', 'sit_request', 'stop_request'):
            return self.bridge.command(kind.removesuffix('_request'))
        if kind == 'se2_velocity_request':
            req = mobility.se2_velocity_request
            end = end_time_secs
            if end is None and req.HasField('end_time'):
                end = req.end_time.seconds + req.end_time.nanos/1e9
            if end is None:
                raise ValueError('Velocity requires end_time_secs')
            v = req.velocity
            return self.bridge.command('velocity', vx=v.linear.x, vy=v.linear.y,
                                       wz=v.angular, frame=req.se2_frame_name,
                                       duration=end-time.time())
        raise NotImplementedError(f'Unsupported mobility command: {kind}')

    def robot_command_feedback(self, robot_command_id, timeout=None, **kwargs):
        info = self.bridge.feedback(robot_command_id)
        response = robot_command_pb2.RobotCommandFeedbackResponse()
        response.header.error.code = header_pb2.CommonError.CODE_OK
        mobility = response.feedback.synchronized_feedback.mobility_command_feedback
        statuses = basic_command_pb2.RobotCommandFeedbackStatus
        mobility.status = {'processing': statuses.STATUS_PROCESSING,
                           'overridden': statuses.STATUS_COMMAND_OVERRIDDEN,
                           'timed_out': statuses.STATUS_COMMAND_TIMED_OUT}[info['status']]
        if info['kind'] == 'stand':
            mobility.stand_feedback.status = (basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING
                if info['complete'] else basic_command_pb2.StandCommand.Feedback.STATUS_IN_PROGRESS)
        elif info['kind'] == 'sit':
            mobility.sit_feedback.status = (basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING
                if info['complete'] else basic_command_pb2.SitCommand.Feedback.STATUS_IN_PROGRESS)
        elif info['kind'] == 'velocity':
            mobility.se2_velocity_feedback.SetInParent()
        else:
            mobility.stop_feedback.SetInParent()
        return response


class SimStateClient:
    default_service_name = 'robot-state'

    def __init__(self, bridge):
        self.bridge = bridge

    def get_robot_state(self, timeout=None, **kwargs):
        raw = self.bridge.state()
        if not raw['ready']:
            raise RuntimeError('Gazebo odometry is missing or stale')
        state = robot_state_pb2.RobotState()
        state.power_state.motor_power_state = (robot_state_pb2.PowerState.MOTOR_POWER_STATE_ON
            if raw['powered'] else robot_state_pb2.PowerState.MOTOR_POWER_STATE_OFF)
        kin = state.kinematic_state
        kin.acquisition_timestamp.FromNanoseconds(time.time_ns())
        # vision and odom coincide in this simulation; body is measured in Gazebo.
        for child, parent in [('vision', ''), ('odom', 'vision'), ('body', 'odom')]:
            edge = kin.transforms_snapshot.child_to_parent_edge_map[child]
            edge.parent_frame_name = parent
            edge.parent_tform_child.rotation.w = 1.
            if child == 'body':
                p, q = edge.parent_tform_child.position, edge.parent_tform_child.rotation
                p.x, p.y, p.z, q.x, q.y, q.z, q.w = raw['pose']
        v = kin.velocity_of_body_in_odom
        v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z = raw['velocity']
        kin.velocity_of_body_in_vision.CopyFrom(v)
        for name, angle in raw['joints'].items():
            joint = kin.joint_states.add()
            leg, axis = name.split('_')
            joint.name = f'{leg}.{axis}'
            joint.position.value = angle
        return state


class SpotSimRobot:
    """Small subset of the Robot interface for local simulated mobility programs."""
    def __init__(self, url='http://127.0.0.1:8080'):
        self.bridge = BridgeClient(url)
        self.clients = {'robot-command': SimCommandClient(self.bridge),
                        'robot-state': SimStateClient(self.bridge)}

    def ensure_client(self, service_name):
        if service_name not in self.clients:
            raise NotImplementedError(f'Unsupported simulation service: {service_name}')
        return self.clients[service_name]

    def power_on(self, timeout_sec=20, update_frequency=1):
        if not self.bridge.state()['ready']:
            raise RuntimeError('Wait for Gazebo odometry before powering on')
        self.bridge.power(True)

    def power_off(self, cut_immediately=False, timeout_sec=20, update_frequency=1):
        if not cut_immediately:
            cid = self.bridge.command('sit')
            deadline = time.monotonic()+timeout_sec
            while not self.bridge.feedback(cid)['complete']:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Simulation did not finish sitting')
                time.sleep(.05)
        self.bridge.power(False)

    def is_powered_on(self):
        return self.bridge.state()['powered']
