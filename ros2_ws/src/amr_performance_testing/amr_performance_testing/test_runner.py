#!/usr/bin/env python3
"""
test_runner.py
--------------
Runs performance tests across multiple scenarios automatically.
Launches world, waits for Nav2, sends goal, collects metrics, then moves to next scenario.
"""

import yaml
import subprocess
import time
import os
import signal
import sys
from datetime import datetime

TARGET_MOVER_SCRIPT = '/ros2_ws/src/amr_gazebo/mover.py'
PERSON_CONFIG_DIR = '/ros2_ws/src/amr_gazebo/config'


class TestRunner:
    """Manages automated performance testing across multiple scenarios."""
    
    def __init__(self, config_file):
        self.config_file = config_file
        self.scenarios = []
        self.current_processes = []
        self.results_dir = '/ros2_ws/test_results'
        
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

        # Support absolute paths or just a filename (resolved from PERSON_CONFIG_DIR)
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
    
    def launch_scenario(self, scenario):
        """Launch a single test scenario."""
        print(f'\n{"="*70}')
        print(f'[SCENARIO] {scenario["name"]}')
        print(f'{"="*70}')
        print(f'  World:   {scenario["world"]}')
        print(f'  Launch:  {scenario["launch"]}')
        print(f'  Person config: {scenario.get("person_config", "none")}')
        print(f'  Timeout: {scenario["timeout"]}s')
        print(f'  Goal:    ({scenario["goal_pose"]["x"]}, {scenario["goal_pose"]["y"]})')
        
        # Build launch command
        launch_file = f'/ros2_ws/src/amr_gazebo/launch/{scenario["launch"]}'
        
        # Check if launch file exists
        if not os.path.isfile(launch_file):
            print(f'[ERROR] Launch file not found: {launch_file}')
            print(f'[SKIP] Skipping scenario: {scenario["name"]}')
            return False
        
        # Launch the scenario
        print(f'\n[LAUNCH] Starting {scenario["launch"]}...')
        launch_cmd = ['ros2', 'launch', 'amr_gazebo', scenario['launch']]
        
        try:
            launch_process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean killing
            )
            self.current_processes.append(launch_process)
            
            # Wait for system to initialize
            print('[WAIT] Waiting for Gazebo and Nav2 to initialize (30s)...')
            time.sleep(30)

            # ── Launch target_mover with this scenario's person config ──────
            self.launch_target_mover(scenario)

            # Give people a moment to start moving before the test begins
            if scenario.get('person_config'):
                print('[WAIT] Waiting 3s for people to start moving...')
                time.sleep(3)

            # Launch performance logger
            print('[LAUNCH] Starting performance logger...')
            logger_cmd = [
                'ros2', 'run', 'amr_performance_testing', 'performance_logger',
                '--ros-args',
                '-p', f'scenario_name:={scenario["name"]}',
                '-p', 'run_number:=1',
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
            )
            self.current_processes.append(logger_process)

            # Wait for completion or timeout
            print(f'[RUNNING] Test in progress (timeout: {scenario["timeout"]}s)...')
            start_time = time.time()

            while True:
                elapsed = time.time() - start_time

                if logger_process.poll() is not None:
                    print('[COMPLETE] Performance logger finished')
                    break

                if elapsed > scenario['timeout']:
                    print(f'[TIMEOUT] Test exceeded {scenario["timeout"]}s')
                    break
                
                # Progress indicator
                if int(elapsed) % 10 == 0:
                    print(f'  ... {int(elapsed)}s elapsed')

                time.sleep(1)

            return True

        except Exception as e:
            print(f'[ERROR] Failed to launch scenario: {e}')
            return False

    def cleanup_processes(self):
        """Kill all running processes."""
        print('\n[CLEANUP] Stopping all processes...')
        
        for process in self.current_processes:
            try:
                # Try graceful termination first
                process.terminate()
                try:
                    process.wait(timeout=3)
                except:
                    # Force kill if still running
                    process.kill()
            except:
                pass
        
        self.current_processes = []
        
        # Extra cleanup - kill any lingering gazebo/ros processes
        try:
            subprocess.run(['pkill', '-9', 'gzserver'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', 'gzclient'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-f', 'rviz2'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-f', 'mover'], stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-f', 'performance_logger'], stderr=subprocess.DEVNULL)
        except:
            pass
        
        print('[CLEANUP] All processes stopped')
        
        # Wait a bit for ports to free up
        time.sleep(5)
    
    def run_all_tests(self):
        """Run all test scenarios sequentially."""
        print(f'\n{"#"*70}')
        print(f'# STARTING AUTOMATED PERFORMANCE TESTING')
        print(f'# Total scenarios: {len(self.scenarios)}')
        print(f'# Results directory: {self.results_dir}')
        print(f'{"#"*70}\n')
        
        completed = 0
        failed = 0
        
        for i, scenario in enumerate(self.scenarios, 1):
            print(f'\n[PROGRESS] Scenario {i}/{len(self.scenarios)}')
            
            success = self.launch_scenario(scenario)
            
            # Cleanup after each scenario
            self.cleanup_processes()
            
            if success:
                completed += 1
            else:
                failed += 1
        
        # Print summary
        print(f'\n{"#"*70}')
        print(f'# TEST RUN COMPLETE')
        print(f'{"#"*70}')
        print(f'  Total scenarios: {len(self.scenarios)}')
        print(f'  Completed:       {completed}')
        print(f'  Failed/Skipped:  {failed}')
        print(f'  Results saved:   {self.results_dir}')
        print(f'{"#"*70}\n')


def main():
    """Main entry point."""
    config_file = '/ros2_ws/src/amr_performance_testing/config/test_scenarios.yaml'
    
    print('='*70)
    print('AMR PERFORMANCE TEST RUNNER')
    print('='*70)
    
    # Check if config exists
    if not os.path.isfile(config_file):
        print(f'[ERROR] Config file not found: {config_file}')
        sys.exit(1)
    
    # Create test runner
    runner = TestRunner(config_file)
    
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
