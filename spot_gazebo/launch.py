"""Run Gazebo and the bridge together, and clean up both on exit."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from .client import BridgeClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM):
            pass
    except PermissionError:
        parser.exit(1, 'Socket creation is blocked by this execution sandbox. Run ./run.sh in a normal terminal.\n')
    root = Path(__file__).resolve().parents[1]
    # Regenerate absolute mesh paths if the checkout has been moved.
    subprocess.run([sys.executable, str(root/'tools/generate_world.py')], check=True)
    env = os.environ.copy()
    # Isolate this simulation from other Gazebo sessions in the same account.
    env.setdefault('GZ_PARTITION', 'spot_gazebo')
    processes = []
    stopping = threading.Event()
    def stop(*_):
        stopping.set()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        cmd = ['gz', 'sim', '-r', '-v', '2']
        if args.headless:
            cmd.append('-s')
        processes.append(subprocess.Popen(cmd+[str(root/'worlds/spot.sdf')], env=env))
        processes.append(subprocess.Popen([sys.executable, '-m', 'spot_gazebo.server', '--port', str(args.port)], env=env))
        client = BridgeClient(f'http://127.0.0.1:{args.port}')
        deadline = time.monotonic()+30
        while not stopping.wait(.2):
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Gazebo or bridge exited during startup')
            try:
                ready = client.state()['ready']
            except (OSError, ValueError):
                ready = False
            if ready:
                print(f'Ready: Spot in Gazebo; SDK bridge http://127.0.0.1:{args.port}', flush=True)
                break
            if time.monotonic() > deadline:
                raise TimeoutError('No Gazebo odometry within 30 seconds')
        while not stopping.wait(.2):
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Gazebo or bridge exited')
    finally:
        # Stop the bridge first so it publishes zero velocity before Gazebo exits.
        for p in reversed(processes):
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()


if __name__ == '__main__':
    main()
