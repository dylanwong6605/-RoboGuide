#!/usr/bin/env python3
"""
test_runner.py
--------------
Runs performance tests across multiple scenarios automatically.
Launches world, waits for Nav2, sends goal, collects metrics, then moves to next scenario.

Usage:
  python3 test_runner.py              # Interactive — prompts for number of runs
  python3 test_runner.py --runs 10    # Run all scenarios 10 times each
  python3 test_runner.py --headless   # Run without Gazebo/RViz GUI for faster batches
  python3 test_runner.py --runs 50    # Run all scenarios 50 times each (overnight)
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
import re
import math
import json
import shutil
from datetime import datetime

TARGET_MOVER_SCRIPT = '/ros2_ws/src/amr_gazebo/mover.py'
PERSON_CONFIG_DIR = '/ros2_ws/src/amr_gazebo/config'

# PID of this script — so we never kill ourselves
MY_PID = os.getpid()
LIFECYCLE_STATE_RE = re.compile(
    r'\b(unconfigured|inactive|active|finalized|configuring|activating|deactivating|cleaningup|shuttingdown|errorprocessing)\b',
    re.IGNORECASE,
)
FASTDDS_SHM_ERR_RE = re.compile(r'rtps_transport_shm error', re.IGNORECASE)
NAV2_MANAGED_ACTIVE_RE = re.compile(
    r'lifecycle_manager_navigation.*Managed nodes are active',
    re.IGNORECASE,
)


class TestRunner:
    """Manages automated performance testing across multiple scenarios."""

    def __init__(self, config_file, num_runs=1, headless=False):
        self.config_file = config_file
        self.scenarios = []
        self.current_processes = []
        self.log_files = []
        self.results_dir = '/ros2_ws/test_results'
        self.num_runs = num_runs
        self.headless = headless
        self._results_cache_mtime = None
        self._results_cache_rows = {}

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

        # Invalidate in-memory CSV cache so subsequent reads pick up this row.
        self._results_cache_mtime = None

        print(f'[CSV] Wrote {reason} failure row for: {scenario["name"]} (run {run_number})')

    def _open_process_log(self, process_name, scenario_name=None, run_number=None):
        """Open a log file for a spawned process and keep the handle for cleanup."""
        logs_dir = os.path.join(self.results_dir, 'process_logs')
        os.makedirs(logs_dir, exist_ok=True)

        safe_process = process_name.replace(' ', '_')
        if scenario_name is not None and run_number is not None:
            safe_scenario = str(scenario_name).replace(' ', '_')
            log_filename = f'{safe_scenario}_run{run_number}_{safe_process}.log'
        else:
            log_filename = f'{safe_process}.log'

        log_path = os.path.join(logs_dir, log_filename)
        log_file = open(log_path, 'a', buffering=1)
        self.log_files.append(log_file)
        return log_file, log_path

    def get_scenario_result(self, scenario_name, run_number):
        """Return latest CSV row for scenario+run, or None if not found."""
        csv_path = os.path.join(self.results_dir, 'combined_results.csv')
        if not os.path.isfile(csv_path):
            return None

        try:
            mtime = os.path.getmtime(csv_path)
            if mtime != self._results_cache_mtime:
                latest_rows = {}
                with open(csv_path, 'r', newline='') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        key = (row.get('scenario_name'), str(row.get('run_number')))
                        latest_rows[key] = row
                self._results_cache_rows = latest_rows
                self._results_cache_mtime = mtime
        except Exception as e:
            print(f'[CSV][WARN] Could not read combined results: {e}')
            return None

        key = (scenario_name, str(run_number))
        return self._results_cache_rows.get(key)

    @staticmethod
    def _csv_success_to_bool(value):
        if isinstance(value, bool):
            return value
        if value is None:
            return False
        return str(value).strip().lower() in ('1', 'true', 'yes')

    def launch_target_mover(self, scenario, run_number=None):
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
            log_file, log_path = self._open_process_log(
                'mover', scenario.get('name'), run_number
            )
            process = subprocess.Popen(
                mover_cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            self.current_processes.append(process)
            print(f'[MOVER] target_mover.py started (PID {process.pid}) | log: {log_path}')
            return process
        except Exception as e:
            print(f'[MOVER][ERROR] Failed to launch target_mover.py: {e}')
            return None

    def wait_for_nav2_ready(self, timeout=120, poll_interval=1.0, launch_log_path=None):
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
        startup_requested = False
        last_reported_state = {}
        last_reported_query_output = {}
        last_query_output_log_time = {}
        last_nonzero_rc_state_log = {}
        launch_log_read_pos = 0

        def extract_state(output_text):
            matches = LIFECYCLE_STATE_RE.findall(output_text or '')
            if not matches:
                return None
            return matches[-1].lower()

        def strip_transient_noise(text):
            if not text:
                return ''
            lines = []
            for line in text.splitlines():
                if FASTDDS_SHM_ERR_RE.search(line):
                    continue
                lines.append(line)
            return '\n'.join(lines).strip()

        def nav2_marked_active_in_launch_log():
            nonlocal launch_log_read_pos
            if not launch_log_path:
                return False
            if not os.path.isfile(launch_log_path):
                return False

            try:
                file_size = os.path.getsize(launch_log_path)
                if file_size < launch_log_read_pos:
                    launch_log_read_pos = 0

                with open(launch_log_path, 'r', encoding='utf-8', errors='ignore') as f:
                    f.seek(launch_log_read_pos)
                    new_content = f.read()
                    launch_log_read_pos = f.tell()

                if not new_content:
                    return False
                return NAV2_MANAGED_ACTIVE_RE.search(new_content) is not None
            except Exception:
                return False

        def try_nav2_startup():
            """Ask lifecycle manager to startup Nav2 if it is stuck unconfigured/inactive."""
            startup_cmd = [
                'ros2', 'service', 'call',
                '/lifecycle_manager_navigation/manage_nodes',
                'nav2_msgs/srv/ManageLifecycleNodes',
                '{command: 0}'
            ]
            try:
                result = subprocess.run(
                    startup_cmd,
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                combined = f"{result.stdout or ''}\n{result.stderr or ''}".strip()
                if result.returncode == 0:
                    print('[HEALTH] Requested Nav2 lifecycle startup via lifecycle_manager_navigation')
                    return True
                if combined:
                    print(f'[HEALTH][WARN] Nav2 startup request failed: {combined}')
            except Exception as e:
                print(f'[HEALTH][WARN] Exception requesting Nav2 startup: {e}')
            return False

        while time.time() - start_time < timeout:
            # Fallback readiness signal from launch log when ROS2 CLI lifecycle query is flaky.
            if nav2_marked_active_in_launch_log():
                elapsed = time.time() - start_time
                print(f'[HEALTH] Nav2 marked active by lifecycle_manager_navigation log after {elapsed:.1f}s')
                return True

            all_active = True
            non_active_states = []
            for node_name in nodes_to_check:
                try:
                    result = subprocess.run(
                        ['ros2', 'lifecycle', 'get', f'/{node_name}'],
                        capture_output=True, text=True, timeout=3
                    )
                    output = (result.stdout or '').strip()
                    stderr = (result.stderr or '').strip()
                    combined = strip_transient_noise(f'{output}\n{stderr}')

                    # Treat parsed lifecycle state as source of truth. `ros2 lifecycle get`
                    # can return non-zero due transient daemon/DDS issues even when state is active.
                    state = extract_state(output) or extract_state(combined)
                    if state == 'active':
                        if result.returncode != 0:
                            prev_logged_state = last_nonzero_rc_state_log.get(node_name)
                            if prev_logged_state != state:
                                print(
                                    f'[HEALTH][WARN] {node_name} reported active with non-zero lifecycle rc '
                                    f'({result.returncode}); proceeding'
                                )
                                last_nonzero_rc_state_log[node_name] = state
                        pass
                    else:
                        if state:
                            if last_reported_state.get(node_name) != state:
                                print(f'[HEALTH] {node_name} state: {state}')
                                last_reported_state[node_name] = state
                            non_active_states.append(state)
                        elif combined:
                            now = time.time()
                            prev_output = last_reported_query_output.get(node_name)
                            prev_time = last_query_output_log_time.get(node_name, 0.0)

                            # Log noisy outputs like "Node not found" only on change and occasionally thereafter.
                            if prev_output != combined or (now - prev_time) >= 10.0:
                                print(f'[HEALTH] {node_name} lifecycle query output: {combined}')
                                last_reported_query_output[node_name] = combined
                                last_query_output_log_time[node_name] = now
                        all_active = False
                        break
                except subprocess.TimeoutExpired:
                    all_active = False
                    break
                except Exception as e:
                    if node_name not in last_reported_query_output:
                        print(f'[HEALTH][WARN] lifecycle query exception for {node_name}: {e}')
                        last_reported_query_output[node_name] = f'exception::{e}'
                    all_active = False
                    break

            if all_active:
                elapsed = time.time() - start_time
                print(f'[HEALTH] All Nav2 nodes active after {elapsed:.1f}s')
                return True

            elapsed = time.time() - start_time
            if (
                not startup_requested
                and elapsed > 20
                and any(state in ('unconfigured', 'inactive') for state in non_active_states)
            ):
                startup_requested = try_nav2_startup()

            time.sleep(max(0.2, float(poll_interval)))

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

    def publish_initial_pose(self, scenario):
        """Publish /initialpose from scenario start_pose to re-seed AMCL each run."""
        start_pose = scenario.get('start_pose', {})
        x = float(start_pose.get('x', 0.0))
        y = float(start_pose.get('y', 0.0))
        theta = float(start_pose.get('theta', 0.0))

        # Convert yaw to quaternion for PoseWithCovarianceStamped.
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        msg = json.dumps({
            'header': {'frame_id': 'map'},
            'pose': {
                'pose': {
                    'position': {'x': x, 'y': y, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': qz, 'w': qw},
                },
                'covariance': [
                    0.10, 0, 0, 0, 0, 0,
                    0, 0.10, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.03,
                ],
            },
        })

        print(f'[HEALTH] Publishing /initialpose at ({x:.2f}, {y:.2f}, theta={theta:.2f})')
        try:
            result = subprocess.run(
                [
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped', msg,
                ],
                capture_output=True,
                text=True,
                timeout=20,
            )
            if result.returncode != 0:
                stderr = (result.stderr or '').strip()
                print(f'[HEALTH][WARN] Failed to publish /initialpose (rc={result.returncode}): {stderr}')
                return False
            return True
        except Exception as e:
            print(f'[HEALTH][WARN] Exception publishing /initialpose: {e}')
            return False

    def stop_gui_processes(self):
        """Best-effort stop of GUI-only processes for headless test runs."""
        for target in ('gzclient', 'rviz2'):
            try:
                subprocess.run(
                    ['pkill', '-15', target],
                    stderr=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                )
            except Exception:
                pass

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
        launch_env = os.environ.copy()

        if self.headless:
            # Force GUI apps to skip visible windows in batch mode.
            launch_env['DISPLAY'] = ''
            launch_env.setdefault('QT_QPA_PLATFORM', 'offscreen')
            launch_env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
            print('[LAUNCH] Headless mode enabled: suppressing Gazebo/RViz GUI')

        try:
            launch_log_file, launch_log_path = self._open_process_log(
                'launch', scenario['name'], run_number
            )
            launch_process = subprocess.Popen(
                launch_cmd,
                stdout=launch_log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                env=launch_env,
            )
            self.current_processes.append(launch_process)
            print(f'[LAUNCH] amr_gazebo launch started (PID {launch_process.pid}) | log: {launch_log_path}')

            if self.headless:
                # Launch files start rviz2/gzclient unconditionally; stop them for headless throughput.
                self.stop_gui_processes()

            # ── Phase 1: Short startup grace period (configurable) ─────
            startup_grace = float(scenario.get('startup_grace_s', 40.0))
            startup_grace = max(0.0, startup_grace)
            if startup_grace > 0:
                print(f'[WAIT] Waiting {startup_grace:.1f}s startup grace...')
                time.sleep(startup_grace)
                if self.headless:
                    self.stop_gui_processes()

            # ── Phase 2: Active health checks ──────────────────────────
            nav2_ready = self.wait_for_nav2_ready(
                timeout=120,
                poll_interval=1.0,
                launch_log_path=launch_log_path,
            )
            if not nav2_ready:
                print('[HEALTH][ERROR] Nav2 did not become active for this run. Aborting scenario early.')
                return 'error'

            # ── Phase 3: Wait for AMCL localization ────────────────────
            self.wait_for_amcl_localized(timeout=30)

            # ── Phase 3b: Re-seed AMCL pose from scenario start pose ───
            if self.publish_initial_pose(scenario):
                print('[WAIT] Waiting 3s after /initialpose publication...')
                time.sleep(3)

            # ── Phase 4: Launch target_mover ───────────────────────────
            self.launch_target_mover(scenario, run_number=run_number)

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
                '-p', f'start_y:={scenario["start_pose"]["y"]}',
                # test_runner already performs Nav2/AMCL readiness checks.
                '-p', 'wait_for_action_server:=false',
                '-p', 'wait_for_nav2_lifecycle:=false',
                '-p', 'amcl_settle_seconds:=0.0',
            ]

            logger_log_file, logger_log_path = self._open_process_log(
                'performance_logger', scenario['name'], run_number
            )

            logger_process = subprocess.Popen(
                logger_cmd,
                stdout=logger_log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            self.current_processes.append(logger_process)
            print(f'[LAUNCH] performance_logger started (PID {logger_process.pid}) | log: {logger_log_path}')

            # Wait for completion or timeout
            print(f'[RUNNING] Test in progress (timeout: {scenario["timeout"]}s)...')
            start_time = time.time()

            while True:
                elapsed = time.time() - start_time

                # End run as soon as logger records a result for this scenario/run.
                result_row = self.get_scenario_result(scenario['name'], run_number)
                if result_row is not None:
                    if self._csv_success_to_bool(result_row.get('success')):
                        print('[COMPLETE] Nav2 reached goal (CSV success=True). Ending run now.')
                        return 'success'

                    print('[COMPLETE][WARN] Logger recorded navigation failure. Ending run now.')
                    return 'error'

                if logger_process.poll() is not None:
                    logger_rc = logger_process.returncode
                    print(f'[COMPLETE] Performance logger exited with code {logger_rc}')

                    result_row = self.get_scenario_result(scenario['name'], run_number)
                    if result_row is None:
                        print('[COMPLETE][WARN] No matching CSV row found for this run')
                        return 'error'

                    if self._csv_success_to_bool(result_row.get('success')):
                        return 'success'

                    print('[COMPLETE][WARN] CSV result indicates navigation failure')
                    return 'error'

                if launch_process.poll() is not None:
                    print(f'[ERROR] Launch process exited early with code {launch_process.returncode}')
                    return 'error'

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
        force_kill_used = False

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
                        force_kill_used = True
            except (ProcessLookupError, OSError):
                pass

        self.current_processes = []

        for log_file in self.log_files:
            try:
                log_file.close()
            except Exception:
                pass
        self.log_files = []

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
                    ['pkill', '-15', target],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                )
            except Exception:
                pass

        time.sleep(1)

        for target in kill_binaries:
            try:
                result = subprocess.run(
                    ['pkill', '-9', target],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                )
                if result.returncode == 0:
                    force_kill_used = True
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
                                force_kill_used = True
                            except (ProcessLookupError, OSError):
                                pass
            except Exception:
                pass

        # ── Step 4: Clean up shared memory left by Gazebo ──────────
        print('[CLEANUP] Clearing shared memory...')
        try:
            # Remove known ROS/Gazebo/FastDDS shared-memory artifacts only.
            shm_patterns = [
                '/dev/shm/sem.fastrtps_*',
                '/dev/shm/sem.fastdds_*',
                '/dev/shm/sem.gazebo*',
                '/dev/shm/fastrtps_*',
                '/dev/shm/fastdds_*',
                '/dev/shm/gazebo*',
                '/dev/shm/ignition*',
                '/dev/shm/ros2_*',
            ]
            for pattern in shm_patterns:
                for shm_file in glob.glob(pattern):
                    try:
                        os.remove(shm_file)
                    except OSError:
                        pass
        except Exception:
            pass

        # Also clean up stale /tmp gazebo/ros2 artifacts.
        # Safety guards: only current-user files, skip symlinks, and keep recent files.
        try:
            cleanup_grace_seconds = 300
            now = time.time()
            current_uid = os.getuid() if hasattr(os, 'getuid') else None

            for pattern in ('/tmp/gazebo-*', '/tmp/ros2_*'):
                for tmp_file in glob.glob(pattern):
                    try:
                        st = os.lstat(tmp_file)

                        # Never follow or remove symlinks via recursive delete path.
                        if os.path.islink(tmp_file):
                            continue

                        # Avoid deleting artifacts from other users in shared environments.
                        if current_uid is not None and hasattr(st, 'st_uid') and st.st_uid != current_uid:
                            continue

                        # Keep very recent files to avoid racing active processes.
                        if (now - st.st_mtime) < cleanup_grace_seconds:
                            continue

                        if os.path.isdir(tmp_file):
                            shutil.rmtree(tmp_file, ignore_errors=True)
                        else:
                            os.remove(tmp_file)
                    except OSError:
                        pass
        except Exception:
            pass

        # ── Step 5: Stop ROS2 daemon only after hard kills to clear DDS state ──
        if force_kill_used:
            try:
                subprocess.run(
                    ['ros2', 'daemon', 'stop'],
                    stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    timeout=10
                )
            except Exception:
                pass

            # Best-effort cleanup for stale FastDDS SHM lock files.
            if shutil.which('fastdds'):
                try:
                    subprocess.run(
                        ['fastdds', 'shm', 'clean', '-f'],
                        stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                        timeout=10
                    )
                except Exception:
                    pass

        print('[CLEANUP] All processes stopped')

        # ── Step 5: Wait for system to settle ──────────────────────
        settle_seconds = 15 if force_kill_used else 5
        print(f'[CLEANUP] Waiting {settle_seconds}s for system to settle...')
        time.sleep(settle_seconds)

        # ── Step 6: Restart ROS2 daemon fresh ──────────────────────
        if force_kill_used:
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

        # Estimate time
        avg_time_per_test = 3  # ~3 minutes (launch + test + cleanup)
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
                    # Verify logger actually wrote a CSV row for this scenario/run.
                    if self.get_scenario_result(scenario['name'], run) is None:
                        print('[WARN] Logger exited but no CSV found, writing placeholder')
                        self.write_timeout_result(scenario, run, reason='logger_no_output')
                elif result == 'timeout':
                    timed_out += 1
                    if self.get_scenario_result(scenario['name'], run) is None:
                        self.write_timeout_result(scenario, run, reason='timeout')
                    else:
                        print('[CSV] Existing row already present for timeout run; skipping placeholder write')
                else:
                    failed += 1
                    if self.get_scenario_result(scenario['name'], run) is None:
                        self.write_timeout_result(scenario, run, reason='error')
                    else:
                        print('[CSV] Existing row already present for failed run; skipping placeholder write')

                # Print running totals
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

    # ── Parse CLI arguments ────────────────────────────────────────
    parser = argparse.ArgumentParser(
        description='AMR Performance Test Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_runner.py              # Interactive prompt
  python3 test_runner.py --runs 10    # 10 runs of all scenarios
  python3 test_runner.py --runs 50    # Overnight: 50 runs
        """
    )
    parser.add_argument(
        '--runs', type=int, default=0,
        help='Number of times to run all scenarios (0 = interactive prompt)'
    )
    parser.add_argument(
        '--headless', '--no-gui', action='store_true',
        help='Disable Gazebo and RViz GUI windows for faster batch testing'
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
        # Warn if shm is small
        if '64M' in result.stdout:
            print('[WARN] /dev/shm is only 64MB! Gazebo may crash after a few runs.')
            print('[WARN] Restart container with: docker run --shm-size=4g ...')
    except Exception:
        pass

    # Create test runner
    runner = TestRunner(config_file, num_runs=num_runs, headless=args.headless)

    # Setup signal handler for clean exit
    def signal_handler(sig, frame):
        print('\n[INTERRUPT] Received Ctrl+C, cleaning up...')
        runner.cleanup_processes()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Run all tests
    runner.run_all_tests()


if __name__ == '__main__':
    main()