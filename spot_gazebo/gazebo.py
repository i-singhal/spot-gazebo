"""Native Gazebo Harmonic transport backend; no ROS bridge is required."""
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.model_pb2 import Model
from .model import JOINT_NAMES, URDF_TO_ALIAS


def rotate_vector(q, v):
    """Rotate a vector by a unit quaternion, into the parent frame."""
    x, y, z, w = q
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return [a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx]


class GazeboBackend:
    def __init__(self):
        self.node = Node()
        self.cmd = self.node.advertise('/model/spot/cmd_vel', Twist)
        self.joint_pubs = {
            alias: self.node.advertise(f'/model/spot/joint/{name}/cmd_pos', Double)
            for alias, name in JOINT_NAMES.items()}

    def attach(self, controller):
        def odom(msg):
            p, q, t = msg.pose.position, msg.pose.orientation, msg.twist
            # Gazebo OdometryPublisher expresses twist in the body frame;
            # the SDK's velocity_of_body_in_odom is expressed in odom.
            rotation = (q.x, q.y, q.z, q.w)
            velocity = (rotate_vector(rotation, (t.linear.x, t.linear.y, t.linear.z)) +
                        rotate_vector(rotation, (t.angular.x, t.angular.y, t.angular.z)))
            controller.update_odom([p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                                   velocity)
        def joints(msg):
            controller.update_joints({URDF_TO_ALIAS[j.name]: j.axis1.position
                                      for j in msg.joint if j.name in URDF_TO_ALIAS})
        self.callbacks = (odom, joints)
        if not self.node.subscribe(Odometry, '/model/spot/odometry', odom):
            raise RuntimeError('Cannot subscribe to Spot odometry')
        if not self.node.subscribe(Model, '/model/spot/joint_state', joints):
            raise RuntimeError('Cannot subscribe to Spot joints')

    def velocity(self, vx, vy, wz, vz=0.):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, vz
        msg.angular.z = wz
        self.cmd.publish(msg)

    def positions(self, positions):
        for name, angle in positions.items():
            self.joint_pubs[name].publish(Double(data=angle))
