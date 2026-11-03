"""Live measured-motion smoke test. Start ./run.sh --headless first."""
import argparse
import math
import time
from spot_gazebo.client import BridgeClient


def wait_for(predicate, seconds=10):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise TimeoutError('Simulation did not reach the expected state')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8080')
    parser.add_argument('--sdk', action='store_true', help='Also use real bosdyn builders, feedback and state')
    args = parser.parse_args()
    bridge = BridgeClient(args.url)
    wait_for(lambda: bridge.state()['ready'])
    if args.sdk:
        from spot_gazebo.sdk import SpotSimRobot
        from bosdyn.client.robot_command import RobotCommandBuilder, blocking_stand
        from bosdyn.client.frame_helpers import get_odom_tform_body
        robot = SpotSimRobot(args.url)
        command = robot.ensure_client('robot-command')
        robot.power_on()
        blocking_stand(command, timeout_sec=10)
        def move(vx, vy, wz):
            return command.robot_command(RobotCommandBuilder.synchro_velocity_command(vx, vy, wz),
                                         end_time_secs=time.time()+1.5)
        def state_check():
            state = robot.ensure_client('robot-state').get_robot_state()
            assert get_odom_tform_body(state.kinematic_state.transforms_snapshot) is not None
    else:
        bridge.power(True)
        cid = bridge.command('stand')
        wait_for(lambda: bridge.feedback(cid)['complete'])
        def move(vx, vy, wz):
            return bridge.command('velocity', vx=vx, vy=vy, wz=wz, duration=1.5)
        def state_check():
            pass
    try:
        start = bridge.state()['pose']
        cid = move(.3, 0., 0.)
        time.sleep(1.9)
        end = bridge.state()['pose']
        assert .25 < end[0]-start[0] < .65, (start, end)
        assert bridge.feedback(cid)['status'] == 'timed_out'
        time.sleep(.3)
        stopped = bridge.state()['pose']
        assert math.dist(end[:2], stopped[:2]) < .04, (end, stopped)
        start = stopped
        move(0., .2, 0.)
        time.sleep(1.9)
        end = bridge.state()['pose']
        assert .15 < end[1]-start[1] < .45, (start, end)
        move(0., 0., .4)
        time.sleep(1.9)
        q = bridge.state()['pose'][3:]
        yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        assert .35 < yaw < .85, yaw
        state_check()
        cid = bridge.command('sit')
        wait_for(lambda: bridge.feedback(cid)['complete'])
        assert abs(bridge.state()['pose'][2]-.27) < .02
        print('PASS: measured forward/strafe/turn motion, expiry stop, state and sit')
    finally:
        bridge.command('stop')
        bridge.power(False)


if __name__ == '__main__':
    main()
