import math
import unittest
from spot_gazebo.core import Controller
from spot_gazebo.model import STAND_HEIGHT, SIT_KNEE


class Backend:
    def velocity(self, *args):
        self.last_velocity = args
    def positions(self, values):
        self.targets = values


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.backend = Backend()
        self.c = Controller(self.backend, clock=lambda: self.now)
        self.odom()
        self.c.power(True)

    def odom(self, z=STAND_HEIGHT, yaw=0.):
        self.c.update_odom([0., 0., z, 0., 0., math.sin(yaw/2), math.cos(yaw/2)], [0.]*6)

    def test_power_required(self):
        self.c.power(False)
        with self.assertRaises(ValueError):
            self.c.command('stand')
        self.c.command('stop')

    def test_deadline_stops_without_further_commands(self):
        cid = self.c.command('velocity', vx=.4, duration=.3)
        self.c.tick()
        self.assertAlmostEqual(self.backend.last_velocity[0], .4)
        self.now += .31
        self.odom()
        self.c.tick()
        self.assertEqual(self.backend.last_velocity[:3], (0., 0., 0.))
        self.assertEqual(self.c.feedback(cid)['status'], 'timed_out')

    def test_stale_odometry_stops(self):
        cid = self.c.command('velocity', vx=.4, duration=2.)
        self.now += .6
        self.c.tick()
        self.assertEqual(self.backend.last_velocity, (0., 0., 0., 0.))
        self.assertEqual(self.c.feedback(cid)['status'], 'timed_out')
        with self.assertRaises(ValueError):
            self.c.command('velocity', vx=.4, duration=1.)

    def test_world_frame_converted_at_current_yaw(self):
        self.odom(yaw=math.pi/2)
        self.c.command('velocity', vx=.4, frame='odom', duration=1.)
        self.c.tick()
        self.assertAlmostEqual(self.backend.last_velocity[0], 0.)
        self.assertAlmostEqual(self.backend.last_velocity[1], -.4)

    def test_stand_feedback_requires_measured_joints_and_height(self):
        cid = self.c.command('stand')
        self.c.tick()
        self.assertFalse(self.c.feedback(cid)['complete'])
        self.c.update_joints(dict(self.backend.targets))
        self.c.tick()
        self.assertTrue(self.c.feedback(cid)['complete'])
        self.odom(z=.3)
        self.c.tick()
        self.assertFalse(self.c.feedback(cid)['complete'])

    def test_command_preemption_and_old_feedback(self):
        old = self.c.command('velocity', vx=.3, duration=1.)
        new = self.c.command('sit')
        self.c.tick()
        self.assertEqual(self.c.feedback(old)['status'], 'overridden')
        self.assertEqual(self.c.feedback(new)['kind'], 'sit')
        self.assertEqual(self.backend.last_velocity[:3], (0., 0., 0.))

    def test_reject_invalid_commands_without_overriding(self):
        cid = self.c.command('stand')
        for args in [dict(kind='velocity', duration=0), dict(kind='velocity', duration=11),
                     dict(kind='velocity', duration=1, vx=float('nan')),
                     dict(kind='velocity', duration=1, vx=1.1),
                     dict(kind='velocity', duration=1, frame='map'), dict(kind='dance')]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.c.command(**args)
        self.assertEqual(self.c.feedback(cid)['status'], 'processing')

    def test_sit_height_and_12_joint_targets(self):
        self.c.command('sit')
        self.c.tick()
        self.assertLess(self.backend.last_velocity[3], 0.)
        self.assertEqual(len(self.backend.targets), 12)
        self.assertAlmostEqual(self.backend.targets['fl_kn'], SIT_KNEE)

    def test_restart_does_not_resume_expired_velocity(self):
        self.c.command('velocity', vx=.3, duration=1.)
        self.now += 2.
        self.c.tick()
        self.odom()
        self.c.tick()
        self.assertEqual(self.backend.last_velocity[:3], (0., 0., 0.))

    def test_feedback_history_is_bounded(self):
        for _ in range(300):
            self.c.command('stand')
        self.assertEqual(len(self.c.history), 256)
        with self.assertRaises(ValueError):
            self.c.feedback(1)

    def test_emergency_stop(self):
        self.c.command('velocity', vx=.3, duration=1.)
        self.c.stop()
        self.assertEqual(self.backend.last_velocity, (0., 0., 0., 0.))

    def test_terminal_feedback_preserved(self):
        cid = self.c.command('velocity', vx=.3, duration=.1)
        self.now += .2
        self.c.tick()
        self.c.command('stand')
        self.assertEqual(self.c.feedback(cid)['status'], 'timed_out')


if __name__ == '__main__':
    unittest.main()
