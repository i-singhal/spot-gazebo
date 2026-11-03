import math
import unittest
from unittest.mock import Mock, patch
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.model_pb2 import Model
from spot_gazebo.gazebo import GazeboBackend, rotate_vector


class TransportTests(unittest.TestCase):
    def test_rotate_body_velocity_into_world(self):
        value = rotate_vector((0, 0, math.sin(math.pi/4), math.cos(math.pi/4)), (.3, 0, 0))
        self.assertAlmostEqual(value[0], 0.)
        self.assertAlmostEqual(value[1], .3)

    @patch('spot_gazebo.gazebo.Node')
    def test_real_gazebo_protobuf_callbacks(self, node):
        backend = GazeboBackend()
        controller = Mock()
        backend.attach(controller)
        odom = Odometry()
        odom.pose.position.x = 2.
        odom.pose.position.z = .47
        odom.pose.orientation.w = math.cos(math.pi/4)
        odom.pose.orientation.z = math.sin(math.pi/4)
        odom.twist.linear.x = .3
        backend.callbacks[0](odom)
        pose, velocity = controller.update_odom.call_args.args
        self.assertAlmostEqual(pose[0], 2.)
        self.assertAlmostEqual(velocity[1], .3)
        model = Model()
        joint = model.joint.add()
        joint.name = 'front_left_hip_x'
        joint.axis1.position = .25
        backend.callbacks[1](model)
        controller.update_joints.assert_called_with({'fl_hx': .25})

    @patch('spot_gazebo.gazebo.Node')
    def test_command_transport_protobufs(self, node):
        backend = GazeboBackend()
        backend.velocity(.2, .1, .3, -.1)
        twist = backend.cmd.publish.call_args.args[0]
        self.assertAlmostEqual(twist.linear.x, .2)
        self.assertAlmostEqual(twist.linear.z, -.1)
        self.assertAlmostEqual(twist.angular.z, .3)
        self.assertEqual(len(backend.joint_pubs), 12)
        backend.positions({'fl_hx': .5})
        self.assertAlmostEqual(backend.joint_pubs['fl_hx'].publish.call_args.args[0].data, .5)
