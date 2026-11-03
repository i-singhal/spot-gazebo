"""Real SDK schema tests; skipped only when optional bosdyn-client is absent."""
import importlib.util
import time
import unittest
from unittest.mock import Mock

SDK_AVAILABLE = importlib.util.find_spec('bosdyn') is not None


@unittest.skipUnless(SDK_AVAILABLE, 'Install bosdyn-client==5.2.0 to test the SDK adapter')
class SDKTests(unittest.TestCase):
    def setUp(self):
        from spot_gazebo.sdk import SimCommandClient, SimStateClient
        from bosdyn.client.robot_command import RobotCommandBuilder
        self.builder = RobotCommandBuilder
        self.bridge = Mock()
        self.bridge.command.return_value = 12
        self.command = SimCommandClient(self.bridge)
        self.state = SimStateClient(self.bridge)

    def test_stand_sit_stop_builders(self):
        for cmd, name in [(self.builder.synchro_stand_command(), 'stand'),
                          (self.builder.synchro_sit_command(), 'sit'),
                          (self.builder.stop_command(), 'stop')]:
            self.assertEqual(self.command.robot_command(cmd), 12)
            self.bridge.command.assert_called_with(name)

    def test_velocity_builder(self):
        cmd = self.builder.synchro_velocity_command(.2, .1, .3)
        self.command.robot_command(cmd, end_time_secs=time.time()+1)
        body = self.bridge.command.call_args.kwargs
        self.assertAlmostEqual(body['vx'], .2)
        self.assertAlmostEqual(body['vy'], .1)
        self.assertAlmostEqual(body['wz'], .3)
        self.assertGreater(body['duration'], .5)

    def test_explicit_body_pose_rejected(self):
        with self.assertRaises(NotImplementedError):
            self.command.robot_command(self.builder.synchro_stand_command(body_height=.1))

    def test_unsupported_trajectory(self):
        with self.assertRaises(NotImplementedError):
            self.command.robot_command(self.builder.synchro_se2_trajectory_point_command(1, 0, 0, 'odom'))

    def test_feedback_real_proto(self):
        from bosdyn.api import basic_command_pb2
        self.bridge.feedback.return_value = dict(kind='stand', status='processing', complete=True)
        result = self.command.robot_command_feedback(12)
        feedback = result.feedback.synchronized_feedback.mobility_command_feedback
        self.assertEqual(feedback.stand_feedback.status,
                         basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING)

    def test_state_frame_tree(self):
        from bosdyn.client.frame_helpers import get_odom_tform_body, validate_frame_tree_snapshot
        self.bridge.state.return_value = dict(ready=True, powered=True, pose=[1., 2., .47, 0., 0., 0., 1.],
                                              velocity=[.2, 0., 0., 0., 0., .1], joints={'fl_hx': .2})
        state = self.state.get_robot_state()
        self.assertTrue(validate_frame_tree_snapshot(state.kinematic_state.transforms_snapshot))
        pose = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
        self.assertAlmostEqual(pose.x, 1.)
        self.assertAlmostEqual(state.kinematic_state.joint_states[0].position.value, .2)


if __name__ == '__main__':
    unittest.main()
