"""Launch isolated ROS scenarios and stop the entire launch process group."""
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('Could not stop the ROS launch process group')


def main():
    os.chdir(ROOT)
    run_dir = ROOT/'results/integration'/f'{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}'
    base_domain = int(os.environ.get('A2_TEST_DOMAIN_ID', '72'))
    if not 0 <= base_domain <= 95:
        raise ValueError('A2_TEST_DOMAIN_ID must be 0..95, reserving five domains')
    scenarios = {
        'normal': [], 'busy': ['busy_seconds:=4.0'],
        'timeout': ['stall:=true', 'process_timeout:=3.0'],
        'station_timeout': ['busy_seconds:=30.0', 'station_timeout:=2.0'], 'cancel': [],
    }
    for index, (scenario, extra) in enumerate(scenarios.items()):
        directory = run_dir/scenario
        directory.mkdir(parents=True)
        environment = dict(os.environ, ROS_DOMAIN_ID=str(base_domain+index), ROS_LOCALHOST_ONLY='1')
        with (directory/'launch.log').open('w', encoding='utf-8') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'a2_demo', 'demo.launch.py', 'rviz:=false',
                 'motion_seconds:=0.7', 'process_seconds:=1.5', f'output_dir:={directory}', *extra],
                env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                subprocess.run(['ros2', 'run', 'a2_demo', 'acceptance', '--scenario', scenario,
                                '--output-dir', str(directory)], env=environment, check=True, timeout=110)
            except Exception:
                log.flush()
                print((directory/'launch.log').read_text(encoding='utf-8'), flush=True)
                raise
            finally:
                stop(process)
    print(f'All five ROS integration scenarios passed. Evidence: {run_dir}', flush=True)


if __name__ == '__main__':
    main()
