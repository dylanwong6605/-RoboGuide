#!/usr/bin/env python3
"""
test_runner.py
--------------
Runs performance tests across multiple scenarios automatically.
Launches world, waits for Nav2, sends goal, collects metrics, then moves to next scenario.

Usage:
  python3 test_runner.py              # Interactive — prompts for number of runs
  python3 test_runner.py --runs 10    # Run all scenarios 10 times each
  python3 test_runner.py --runs 50 --headless   # Overnight: 50 runs, no GUI
"""

import yaml
import subprocess
import time
import os
import signal
import sys
import csv
import glob
import argparse
from datetime import datetime

TARGET_MOVER_SCRIPT = '/ros2_ws/src/amr_gazebo/mover.py'
PERSON_CONFIG_DIR = '/ros2_ws/src/amr_gazebo/config'

# PID of this script — so we never kill ourselves
MY_PID = os.getpid()


class TestRunner:
    """Manages automated performance testing across multiple scenarios."""

    def __init__(self, config_file, num_runs=1, headless=False):
        self.config_file = config_file
        self.scenarios = []
        self.current_processes = []
        self.results_dir = '/ros2_ws/test_results'
        self.num_runs = num_runs
        self.headless = headless

        # Load configuration
        self.load_config()

        # Create results directory
        self.create_results_dir()

    def load_config(self):
        """Load test scenarios from YAML config."""
        print(f'[INFO] Loading config from {self.config_file}')

        with open(self.config_file, 'r') as f:
            config = yaml.safe_load(f)

        self.scenarios = config.get('scenarios', [])

        if len(self.scenarios) == 0:
            print('[ERROR] No scenarios found in config file!')
            sys.exit(1)

        print(f'[INFO] Loaded {len(self.scenarios)} scenarios')
        for i, scenario in enumerate(self.scenarios, 1):
            person_cfg = scenario.get('person_config', 'none')
            print(f'  {i}. {scenario["name"]}  (people: {person_cfg})')

    def create_results_dir(self):
        """Create timestamped results directory."""
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.results_dir = f'/ros2_ws/test_results/run_{timestamp}'
        os.makedirs(self.results_dir, exist_ok=True)
        print(f'[INFO] Results will be saved to: {self.results_dir}')

    def write_timeout_result(self, scenario, run_number, reason='timeout'):
        """Write a failure row to CSV when the logger didn't get to save results itself."""
        filename = os.path.join(self.results_dir, 'combined_results.csv')
        file_exists = os.path.isfile(filename)

        fieldnames = [
            'scenario_name', 'run_number', 'timestamp',
            'time_to_goal', 'success', 'path_efficiency',
            'num_recoveries', 'avg_speed', 'planning_time',
            'min_obstacle_dist', 'safety_violations', 'collision_count'
        ]

        row = {
            'scenario_name': scenario['name'],
            'run_number': run_number,
            'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
            'time_to_goal': 0.0,
            'success': False,
            'path_efficiency': 0.0,
            'num_recoveries': 0,
            'avg_speed': 0.0,
            'planning_time': 0.0,
            'min_obstacle_dist': 0.0,
            'safety_violations': 0,
            'collision_count': 0,
        }

        with open(filename, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

        print(f'[CSV] Wrote {reason} failure row for: {scenario["name"]} (run {run_number})')

    def launch_target_mover(self, scenario):
        """
        Launch mover.py with the scenario's person config.
        Returns the process, or None if no person_config is defined
        or the config file doesn't exist.
        """
        person_config = scenario.get('person_config')

        if not person_config:
            print('[MOVER] No person_config defined — skipping target mover')
            return None

        if os.path.isabs(person_config):
            config_path = person_config
        else:
            config_path = os.path.join(PERSON_CONFIG_DIR, person_config)

        if not os.path.isfile(config_path):
            print(f'[MOVER][WARN] Person config not found: {config_path}')
            print(f'[MOVER][WARN] Continuing without moving people')
            return None

        print(f'[MOVER] Launching target_mover.py with config: {config_path}')

        mover_cmd = ['python3', TARGET_MOVER_SCRIPT, '--config', config_path]

        try:
            process = subprocess.Popen(
                mover_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.current_processes.append(process)
            print(f'[MOVER] target_mover.py started (PID {process.pid})')
            return process
        except Exception as e:
            print(f'[MOVER][ERROR] Failed to launch target_mover.py: {e}')
            return None

    def wait_for_nav2_ready(self, timeout=120):
        """
        Wait until Nav2 lifecycle nodes are active by polling their state.
        Returns True if all nodes are active, False on timeout.
        """
        nodes_to_check = [
            'bt_navigator',
            'controller_server',
            'planner_server',
        ]

        print(f'[HEALTH] Waiting for Nav2 lifecycle nodes to become active (timeout: {timeout}s)...')
        start_time = time.time()

        while time.time() - start_time < timeout:
            all_active = True
            for node_name in nodes_to_check:
                try:
                    result = subprocess.run(
                        ['ros2', 'lifecycle', 'get', f'/{node_name}'],
                        capture_output=True, text=True, timeout=10
                    )
                    output = result.stdout.strip().lower()
                    if 'active' in output:
                        pass
                    else:
                        all_active = False
                        break
                except (subprocess.TimeoutExpired, Exception):
                    all_active = False
                    break

            if all_active:
                elapsed = time.time() - start_time
                print(f'[HEALTH] All Nav2 nodes active after {elapsed:.1f}s')
                return True

            time.sleep(3)

        print(f'[HEALTH][ERROR] Timeout waiting for Nav2 nodes after {timeout}s')
        return False

    def wait_for_amcl_localized(self, timeout=30):
        """
        Wait until AMCL is publishing on /amcl_pose, indicating localization.
        """
        print(f'[HEALTH] Waiting for AMCL localization (timeout: {timeout}s)...')
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', '/amcl_pose', '--once'],
                    capture_output=True, text=True, timeout=10
                )
                if result.returncode == 0 and len(result.stdout.strip()) > 0:
                    elapsed = time.time() - start_time
                    print(f'[HEALTH] AMCL localized after {elapsed:.1f}s')
                    return True
            except subprocess.TimeoutExpired:
                pass
            except Exception:
                pass

            time.sleep(2)

        print(f'[HEALTH][WARN] AMCL localization not confirmed after {timeout}s, proceeding anyway')
        return False

    def launch_scenario(self, scenario, run_number):
        """Launch a single test scenario. Returns 'success', 'timeout', or 'error'."""
        print(f'\n{"="*70}')
        print(f'[SCENARIO] {scenario["name"]}  (run {run_number}/{self.num_runs})')
        print(f'{"="*70}')
        print(f'  World:   {scenario["world"]}')
        print(f'  Launch:  {scenario["launch"]}')
        print(f'  Person config: {scenario.get("person_config", "none")}')
        print(f'  Timeout: {scenario["timeout"]}s')
        print(f'  Goal:    ({scenario["goal_pose"]["x"]}, {scenario["goal_pose"]["y"]})')

        launch_file = f'/ros2_ws/src/amr_gazebo/launch/{scenario["launch"]}'

        if not os.path.isfile(launch_file):
            print(f'[ERROR] Launch file not found: {launch_file}')
            print(f'[SKIP] Skipping scenario: {scenario["name"]}')
            return 'error'

        print(f'\n[LAUNCH] Starting {scenario["launch"]}...')
        launch_cmd = ['ros2', 'launch', 'amr_gazebo', scenario['launch']]

        # Headless: suppress GUI windows
        launch_env = os.environ.copy()
        if self.headless:
            launch_env['DISPLAY'] = ''
            print('[LAUNCH] Headless mode — GUI suppressed')

        try:
            launch_process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=launch_env,
            )
            self.current_processes.append(launch_process)

            # ── Phase 1: Wait for Gazebo to load ───────────────────────
            print('[WAIT] Waiting 45s for Gazebo to load world...')
            time.sleep(45)

            # ── Phase 2: Active health checks ──────────────────────────
            nav2_ready = self.wait_for_nav2_ready(timeout=120)
            if not nav2_ready:
                print('[ERROR] Nav2 did not become ready')
                return 'error'

            # ── Phase 3: Wait for AMCL localization ────────────────────
            self.wait_for_amcl_localized(timeout=30)

            # ── Phase 4: Launch target_mover ───────────────────────────
            self.launch_target_mover(scenario)

            if scenario.get('person_config'):
                print('[WAIT] Waiting 5s for people to start moving...')
                time.sleep(5)

            # ── Phase 5: Launch performance logger ─────────────────────
            print('[LAUNCH] Starting performance logger...')
            logger_cmd = [
                'ros2', 'run', 'amr_performance_testing', 'performance_logger',
                '--ros-args',
                '-p', f'scenario_name:={scenario["name"]}',
                '-p', f'run_number:={run_number}',
                '-p', f'output_dir:={self.results_dir}',
                '-p', f'goal_x:={scenario["goal_pose"]["x"]}',
                '-p', f'goal_y:={scenario["goal_pose"]["y"]}',
                '-p', f'start_x:={scenario["start_pose"]["x"]}',
                '-p', f'start_y:={scenario["start_pose"]["y"]}'
            ]

            logger_process = subprocess.Popen(
                logger_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.current_processes.append(logger_process)

            # Wait for completion or timeout
            print(f'[RUNNING] Test in progress (timeout: {scenario["timeout"]}s)...')
            start_time = time.time()

            while True:
                elapsed = time.time() - start_time

                if logger_process.poll() is not None:
                    print('[COMPLETE] Performance logger finished')
                    return 'success'

                if elapsed > scenario['timeout']:
                    print(f'[TIMEOUT] Test exceeded {scenario["timeout"]}s')
                    return 'timeout'

                if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                    print(f'  ... {int(elapsed)}s elapsed')

                time.sleep(1)

        except Exception as e:
            print(f'[ERROR] Failed to launch scenario: {e}')
            return 'error'

    def cleanup_processes(self):
        """Kill all running processes thoroughly without killing ourselves."""
        print('\n[CLEANUP] Stopping all processes...')

        my_pgid = os.getpgid(MY_PID)

        # ── Step 1: Graceful SIGTERM to tracked process groups ─────
        for process in self.current_processes:
            try:
                pgid = os.getpgid(process.pid)
                if pgid != my_pgid:
                    os.killpg(pgid, signal.SIGTERM)
            except (ProcessLookupError, OSError):
                pass

        time.sleep(5)

        # ── Step 2: Force kill anything still alive ────────────────
        for process in self.current_processes:
            try:
                if process.poll() is None:
                    pgid = os.getpgid(process.pid)
                    if pgid != my_pgid:
                        os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass

        self.current_processes = []

        # ── Step 3: Targeted kills — only specific binary names ────
        kill_binaries = [
            'gzserver',
            'gzclient',
            'rviz2',
            'robot_state_publisher',
            'map_server',
            'amcl',
            'planner_server',
            'controller_server',
            'bt_navigator',
            'lifecycle_manager',
            'yolo_perception',
            'yolo_reactive_controller',
            'component_container',
        ]
        for target in kill_binaries:
            try:
                subprocess.run(
                    ['pkill', '-9', target],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                )
            except Exception:
                pass

        # For python scripts, find by -f but exclude our own PID
        for pattern in ['mover.py', 'spawn_entity.py', 'performance_logger']:
            try:
                result = subprocess.run(
                    ['pgrep', '-f', pattern],
                    capture_output=True, text=True
                )
                if result.stdout.strip():
                    for pid_str in result.stdout.strip().split('\n'):
                        pid = int(pid_str.strip())
                        if pid != MY_PID:
                            try:
                                os.kill(pid, signal.SIGKILL)
                            except (ProcessLookupError, OSError):
                                pass
            except Exception:
                pass

        # ── Step 4: Clean up shared memory left by Gazebo ──────────
        print('[CLEANUP] Clearing shared memory...')
        try:
            for shm_file in glob.glob('/dev/shm/sem.*'):
                try:
                    os.remove(shm_file)
                except OSError:
                    pass
            for shm_file in glob.glob('/dev/shm/*'):
                try:
                    os.remove(shm_file)
                except OSError:
                    pass
        except Exception:
            pass

        # Clean up /tmp gazebo and ros2 temp files
        try:
            for tmp_pattern in ['/tmp/gazebo-*', '/tmp/ros2_*']:
                for tmp_file in glob.glob(tmp_pattern):
                    subprocess.run(
                        ['rm', '-rf', tmp_file],
                        stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                    )
        except Exception:
            pass

        # ── Step 5: Stop ROS2 daemon to clear DDS discovery state ──
        try:
            subprocess.run(
                ['ros2', 'daemon', 'stop'],
                stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                timeout=10
            )
        except Exception:
            pass

        print('[CLEANUP] All processes stopped')

        # ── Step 6: Wait for system to settle ──────────────────────
        print('[CLEANUP] Waiting 15s for system to settle...')
        time.sleep(15)

        # ── Step 7: Restart ROS2 daemon fresh ──────────────────────
        try:
            subprocess.run(
                ['ros2', 'daemon', 'start'],
                stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                timeout=10
            )
        except Exception:
            pass

        time.sleep(2)
        print('[CLEANUP] Ready for next scenario')

    def run_all_tests(self):
        """Run all test scenarios sequentially, repeated num_runs times."""
        total_tests = len(self.scenarios) * self.num_runs

        print(f'\n{"#"*70}')
        print(f'# STARTING AUTOMATED PERFORMANCE TESTING')
        print(f'# Scenarios:       {len(self.scenarios)}')
        print(f'# Runs per scene:  {self.num_runs}')
        print(f'# Total tests:     {total_tests}')
        print(f'# Results dir:     {self.results_dir}')
        if self.headless:
            print(f'# Mode:            HEADLESS (no GUI)')

        avg_time_per_test = 3
        est_hours = (total_tests * avg_time_per_test) / 60
        print(f'# Est. duration:   ~{est_hours:.1f} hours')
        print(f'{"#"*70}\n')

        completed = 0
        failed = 0
        timed_out = 0
        test_number = 0

        for run in range(1, self.num_runs + 1):
            print(f'\n{"*"*70}')
            print(f'* RUN {run}/{self.num_runs}')
            print(f'{"*"*70}')

            for i, scenario in enumerate(self.scenarios, 1):
                test_number += 1
                print(f'\n[PROGRESS] Test {test_number}/{total_tests}  '
                      f'(run {run}/{self.num_runs}, scenario {i}/{len(self.scenarios)})')

                result = self.launch_scenario(scenario, run_number=run)

                # Cleanup after each scenario — ALWAYS runs
                self.cleanup_processes()

                if result == 'success':
                    completed += 1
                    csv_path = os.path.join(self.results_dir, 'combined_results.csv')
                    if not os.path.isfile(csv_path):
                        print('[WARN] Logger exited but no CSV found, writing placeholder')
                        self.write_timeout_result(scenario, run, reason='logger_no_output')
                elif result == 'timeout':
                    timed_out += 1
                    self.write_timeout_result(scenario, run, reason='timeout')
                else:
                    failed += 1
                    self.write_timeout_result(scenario, run, reason='error')

                print(f'[STATS] So far: {completed} passed, {timed_out} timed out, '
                      f'{failed} failed  ({test_number}/{total_tests} done)')

        # Print final summary
        print(f'\n{"#"*70}')
        print(f'# ALL RUNS COMPLETE')
        print(f'{"#"*70}')
        print(f'  Total runs:      {self.num_runs}')
        print(f'  Scenarios/run:   {len(self.scenarios)}')
        print(f'  Total tests:     {total_tests}')
        print(f'  Completed:       {completed}')
        print(f'  Timed out:       {timed_out}')
        print(f'  Failed/Skipped:  {failed}')
        print(f'  Success rate:    {completed}/{total_tests} '
              f'({100*completed/max(total_tests,1):.1f}%)')
        print(f'  Results saved:   {self.results_dir}')
        print(f'{"#"*70}\n')


def main():
    """Main entry point."""
    config_file = '/ros2_ws/src/amr_performance_testing/config/test_scenarios.yaml'

    parser = argparse.ArgumentParser(
        description='AMR Performance Test Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_runner.py              # Interactive prompt
  python3 test_runner.py --runs 10    # 10 runs of all scenarios
  python3 test_runner.py --runs 50 --headless   # Overnight: 50 runs, no GUI
        """
    )
    parser.add_argument(
        '--runs', type=int, default=0,
        help='Number of times to run all scenarios (0 = interactive prompt)'
    )
    parser.add_argument(
        '--headless', action='store_true',
        help='Disable Gazebo and RViz GUI windows'
    )
    args = parser.parse_args()

    print('='*70)
    print('AMR PERFORMANCE TEST RUNNER')
    print('='*70)

    if not os.path.isfile(config_file):
        print(f'[ERROR] Config file not found: {config_file}')
        sys.exit(1)

    # ── Determine number of runs ───────────────────────────────────
    num_runs = args.runs

    if num_runs <= 0:
        print('\nHow many times do you want to run all scenarios?')
        print('  Common choices: 1 (quick test), 10 (short batch), 50 (overnight)')
        while True:
            try:
                user_input = input('\nEnter number of runs: ').strip()
                num_runs = int(user_input)
                if num_runs < 1:
                    print('Please enter a positive number.')
                    continue
                break
            except ValueError:
                print('Please enter a valid number.')
            except (EOFError, KeyboardInterrupt):
                print('\nExiting.')
                sys.exit(0)

    print(f'\n[INFO] Will run all scenarios {num_runs} time(s)')
    if args.headless:
        print('[INFO] Headless mode enabled (no GUI)')

    # ── Check shared memory ────────────────────────────────────────
    try:
        result = subprocess.run(['df', '-h', '/dev/shm'], capture_output=True, text=True)
        print(f'[INFO] Shared memory status:')
        for line in result.stdout.strip().split('\n'):
            print(f'  {line}')
        if '64M' in result.stdout:
            print('[WARN] /dev/shm is only 64MB! Gazebo may crash after a few runs.')
            print('[WARN] Add shm_size: "4g" to your docker-compose.yml')
    except Exception:
        pass

    runner = TestRunner(config_file, num_runs=num_runs, headless=args.headless)

    def signal_handler(sig, frame):
        print('\n[INTERRUPT] Received Ctrl+C, cleaning up...')
        runner.cleanup_processes()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    runner.run_all_tests()


if __name__ == '__main__':
    main()