"""Thread-safe command controller. Poses and joint readings come from Gazebo."""
import math
import threading
import time
from collections import OrderedDict
from .model import STAND_HEIGHT, SIT_HEIGHT, STAND_HIP, STAND_KNEE, SIT_HIP, SIT_KNEE

LEGS = ('fl', 'fr', 'hl', 'hr')


class Controller:
    def __init__(self, backend, clock=time.monotonic):
        self.backend, self.clock = backend, clock
        self.lock = threading.RLock()
        self.powered = False
        self.mode = 'sit'
        self.velocity = (0., 0., 0.)
        self.frame = 'body'
        self.deadline = 0.
        self.pose = [0., 0., SIT_HEIGHT, 0., 0., 0., 1.]
        self.measured_velocity = [0.] * 6
        self.joints = {}
        self.last_odom = -math.inf
        self.command_id = 0
        self.history = OrderedDict()
        self.phase = 0.
        self.last_tick = self.clock()

    def update_odom(self, pose, velocity):
        with self.lock:
            self.pose, self.measured_velocity = list(pose), list(velocity)
            self.last_odom = self.clock()

    def update_joints(self, joints):
        with self.lock:
            self.joints = joints

    def state(self):
        with self.lock:
            return dict(powered=self.powered, mode=self.mode, pose=self.pose[:],
                        velocity=self.measured_velocity[:], joints=dict(self.joints),
                        odom_age=self.clock()-self.last_odom if math.isfinite(self.last_odom) else None,
                        ready=self.clock()-self.last_odom < .5)

    def power(self, on):
        with self.lock:
            self.powered = bool(on)
            self.mode = 'sit'
            self.velocity = (0., 0., 0.)
            self.backend.velocity(0., 0., 0., 0.)
            self._override()

    def _override(self):
        if (self.command_id in self.history and
                self.history[self.command_id]['status'] == 'processing'):
            self.history[self.command_id]['status'] = 'overridden'

    def command(self, kind, vx=0., vy=0., wz=0., duration=0., frame='body'):
        with self.lock:
            if kind not in ('stand', 'sit', 'velocity', 'stop'):
                raise ValueError(f'Unsupported command: {kind}')
            values = (vx, vy, wz, duration)
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                raise ValueError('Command values must be finite numbers')
            if kind != 'stop' and not self.powered:
                raise ValueError('Power on the simulated motors first')
            if kind == 'velocity':
                if self.clock()-self.last_odom > .5:
                    raise ValueError('Gazebo odometry is missing or stale')
                if not 0 < duration <= 10:
                    raise ValueError('Velocity duration must be in (0, 10] seconds')
                if frame not in ('body', 'flat_body', 'odom', 'vision'):
                    raise ValueError(f'Unsupported frame: {frame}')
                if abs(vx) > 1 or abs(vy) > 1 or abs(wz) > 1.5:
                    raise ValueError('Velocity exceeds simulation limits: 1 m/s, 1.5 rad/s')
            self._override()
            self.command_id += 1
            self.history[self.command_id] = dict(kind=kind, status='processing', complete=False)
            while len(self.history) > 256:
                self.history.popitem(last=False)
            self.velocity = (vx, vy, wz) if kind == 'velocity' else (0., 0., 0.)
            self.mode = {'velocity': 'stand', 'stop': self.mode}.get(kind, kind)
            self.frame = frame
            self.deadline = self.clock() + duration if kind == 'velocity' else math.inf
            return self.command_id

    def feedback(self, command_id):
        with self.lock:
            if command_id not in self.history:
                raise ValueError('Unknown or evicted command ID')
            return dict(self.history[command_id])

    def tick(self):
        with self.lock:
            now = self.clock()
            dt = min(max(now-self.last_tick, 0), .1)
            self.last_tick = now
            stale = now-self.last_odom > .5
            current = self.history.get(self.command_id)
            if now >= self.deadline or stale or not self.powered:
                self.velocity = (0., 0., 0.)
                if current and current['status'] == 'processing' and current['kind'] == 'velocity':
                    current['status'] = 'timed_out'
            vx, vy, wz = self.velocity
            if self.frame in ('odom', 'vision'):
                qx, qy, qz, qw = self.pose[3:]
                yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                vx, vy = math.cos(yaw)*vx+math.sin(yaw)*vy, -math.sin(yaw)*vx+math.cos(yaw)*vy
            height = STAND_HEIGHT if self.mode == 'stand' else SIT_HEIGHT
            vz = max(-.2, min(.2, 3*(height-self.pose[2]))) if not stale else 0.
            self.backend.velocity(vx, vy, wz, vz)
            moving = abs(vx)+abs(vy)+abs(wz) > .001
            self.phase += dt*2*math.pi*1.6 if moving else 0
            hip, knee = (STAND_HIP, STAND_KNEE) if self.mode == 'stand' else (SIT_HIP, SIT_KNEE)
            targets = {}
            for leg in LEGS:
                phase = self.phase + (math.pi if leg in ('fr', 'hl') else 0)
                swing = .22*math.sin(phase) if moving else 0
                targets[leg+'_hx'] = 0.
                targets[leg+'_hy'] = hip+swing
                targets[leg+'_kn'] = knee-.25*max(0., math.cos(phase)) if moving else knee
            self.backend.positions(targets)
            if current and current['status'] == 'processing':
                settled = (not stale and abs(height-self.pose[2]) < .015 and
                           all(name in self.joints and abs(self.joints[name]-angle) < .08
                               for name, angle in targets.items()))
                current['complete'] = settled if current['kind'] in ('stand', 'sit') else not moving

    def stop(self):
        with self.lock:
            self.velocity = (0., 0., 0.)
            self.backend.velocity(0., 0., 0., 0.)
