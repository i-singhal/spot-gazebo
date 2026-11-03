"""Stand, move forward, strafe, turn, sit using real Spot SDK command builders."""
import argparse
import time
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_odom_tform_body
from spot_gazebo.sdk import SpotSimRobot

parser = argparse.ArgumentParser()
parser.add_argument('--url', default='http://127.0.0.1:8080')
args = parser.parse_args()
robot = SpotSimRobot(args.url)
command = robot.ensure_client(RobotCommandClient.default_service_name)
state = robot.ensure_client(RobotStateClient.default_service_name)
robot.power_on()
try:
    blocking_stand(command, timeout_sec=10)
    for vx, vy, wz in [(.3, 0., 0.), (0., .2, 0.), (0., 0., .4)]:
        command.robot_command(RobotCommandBuilder.synchro_velocity_command(vx, vy, wz),
                              end_time_secs=time.time()+2.)
        time.sleep(2.3)
        print(get_odom_tform_body(state.get_robot_state().kinematic_state.transforms_snapshot))
finally:
    command.robot_command(RobotCommandBuilder.stop_command())
    robot.power_off()
