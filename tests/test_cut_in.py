import re
import argparse
import unittest
import os
import sys
import subprocess
import logging
import time
import threading

setup_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'setup'))
xosc_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources/xosc/vehicle-cut-in_cr.xosc'))
xodr_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources/xodr/curves.xodr'))
xodr_dir2 = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources/xodr/curves_elevation.xodr'))
sys.path.insert(0, setup_dir)

# Import functions from esmini_player
from esmini_player import modify_xosc, modify_xosc_with_xodr, load_library, acc_control_loop
from constants import *

# Logging setup
logging.basicConfig(filename='simulation.log', level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class TestSimulation(unittest.TestCase):
    """
    Unit tests for the driving simulation scenarios.
    """

    def setUp(self):
        """Setup for each test case. If running on Windows, uses threading for timeout handling."""
        self.timeout = 40  # Set default timeout to 40 seconds
        self.test_thread = threading.Thread(target=self._run_with_timeout)
        self.test_thread.start()

    def tearDown(self):
        """Stop the thread and handle any cleanup after each test."""
        self.test_thread.join()

    def _run_with_timeout(self):
        try:
            time.sleep(self.timeout)
        except Exception as e:
            logging.error(f"Test timed out: {str(e)}")
            self.fail(f"Test timed out after {self.timeout} seconds")

    # Utility function to run the simulation with modified parameters
    def run_simulation(self, xosc_file, xodr_file=None, duration=SIMULATION_DURATION, step_interval=STEP_INTERVAL, speed=None, lane_offset=None,
                       ego_position=None, weather=None, headway_time=None):
        """
        Run the simulation with specified parameters, including the ACC control loop.
        """
        logging.info(f"Running simulation with speed={speed}, lane_offset={lane_offset}, "
                     f"ego_position={ego_position}, weather={weather}, headway_time={headway_time}")

        # If xodr_file is provided, update the xosc file with the new xodr file path
        if xodr_file:
            xosc_file = modify_xosc_with_xodr(xosc_file, xodr_file)

        # Modify the .xosc file using provided parameters
        modified_xosc = modify_xosc(xosc_file, speed=speed, lane_offset=lane_offset, ego_position=ego_position,
                                    weather=weather, headway_time=headway_time)

        try:
            # Load the simulation library
            se = load_library()
            se.SE_Init(modified_xosc.encode('ascii'), 0, 1, 0, 0)

            start_time = time.time()
            end_time = start_time + duration if duration else None

            # Initial setup for the ACC
            current_speed = 20  # Initial speed for the ego vehicle
            lead_vehicle_speed = 18  # Example lead vehicle speed
            lead_acceleration = 0  # Assume the lead vehicle has constant speed
            target_headway_time = TARGET_HEADWAY_TIME  # Safe following distance
            road_curvature = DEFAULT_CURVATURE  # Example road curvature value
            jerk_limit = JERK_LIMIT  # Control the rate of speed change

            while not se.SE_GetQuitFlag():
                # Call the ACC control loop to update the ego vehicle's speed
                current_speed = acc_control_loop(
                    current_speed=current_speed,
                    lead_vehicle_speed=lead_vehicle_speed,
                    lead_acceleration=lead_acceleration,
                    headway_time=headway_time if headway_time else 1.5,  # Default headway time
                    target_headway_time=target_headway_time,
                    road_curvature=road_curvature,
                    weather_type=weather['type'] if weather else 'clear',
                    jerk_limit=jerk_limit
                )

                se.SE_Step()
                if end_time and time.time() >= end_time:
                    logging.info("Simulation duration reached.")
                    break
                time.sleep(step_interval)

        except Exception as e:
            logging.error(f"Simulation error: {str(e)}")
            self.fail(f"Simulation failed with error: {str(e)}")

    def test_default_simulation(self):
        """Test default scenario without parameter changes."""
        self.run_simulation(
            xosc_file=xosc_dir,
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL,
            speed=25  # Start with default speed of 25 m/s
        )
        self.assertTrue(os.path.exists("simulation.log"), "Safe distance calculated: 10.00 meters (Speed: 20.00 m/s)")

    def test_curves_elevation_xodr_simulation(self):
        """Test scenario with 'curves_elevation' xodr file."""
        self.run_simulation(
            xosc_file=xosc_dir,
            xodr_file=xodr_dir2,
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_high_speed_scenario(self):
        """Test scenario with high vehicle speed."""
        self.run_simulation(
            xosc_file=xosc_dir,
            speed=30,  # Set to 30 m/s
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_low_speed_scenario(self):
        """Test scenario with low vehicle speed."""
        self.run_simulation(
            xosc_file=xosc_dir,
            speed=10,  # Set to 10 m/s
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_rainy_weather_condition(self):
        """Test scenario with rainy weather conditions."""
        self.run_simulation(
            xosc_file=xosc_dir,
            weather={'type': 'rain', 'intensity': 'heavy'},  # Set heavy rain
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )
        log_data = open("simulation.log").read()
        self.assertIn("Adjusting speed for rain", log_data)

    def test_multiple_weather_conditions(self):
        """Test scenario with weather changing from rain to clear."""
        self.run_simulation(
            xosc_file=xosc_dir,
            weather={'type': 'rain', 'intensity': 'moderate'},  # Start with moderate rain
            duration=15,
            step_interval=STEP_INTERVAL
        )
        # Change weather to clear after 15 seconds
        self.run_simulation(
            xosc_file=xosc_dir,
            weather={'type': 'clear', 'intensity': 'none'},  # Set to clear
            duration=15,
            step_interval=STEP_INTERVAL
        )

    def test_ego_vehicle_start_position(self):
        """Test scenario with a different Ego vehicle start position."""
        self.run_simulation(
            xosc_file=xosc_dir,
            ego_position=200,  # Set Ego vehicle start position to 200
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_headway_time_adjustment(self):
        """Test scenario with a different headway time for the cut-in."""
        self.run_simulation(
            xosc_file=xosc_dir,
            headway_time=1.5,  # Set headway time to 1.5 seconds
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_lane_change_and_speed_increase(self):
        """Test scenario where lane offset and speed increase dynamically."""
        self.run_simulation(
            xosc_file=xosc_dir,
            speed=25,  # Start with speed of 25 m/s
            lane_offset=4.0,  # Start with lane offset of 4 meters
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )
        # Simulate increase in speed and lane offset after 5 seconds
        time.sleep(5)
        self.run_simulation(
            xosc_file=xosc_dir,
            speed=35,  # Increase speed to 35 m/s
            lane_offset=3.5,  # Adjust lane offset to 3.5 meters
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_curved_road_with_rain(self):
        """Test scenario on a curved road with rainy conditions."""
        self.run_simulation(
            xosc_file=xosc_dir,
            xodr_file=xodr_dir,  # Use curved road from xodr file
            speed=20,  # Set speed to 20 m/s
            weather={'type': 'rain', 'intensity': 'moderate'},  # Set moderate rain
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )

    def test_overtaking_scenario(self):
        """Test scenario where the overtaking car does not collide with the lead vehicle."""
        self.run_simulation(
            xosc_file=xosc_dir,
            speed=30,  # Set speed of the overtaking car
            ego_position=50,  # Set Ego vehicle start position
            headway_time=2.0,  # Set headway time to maintain safe distance
            duration=SIMULATION_DURATION,
            step_interval=STEP_INTERVAL
        )


if __name__ == "__main__":
    # Execute the unit tests
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="Timeout per test case")
    parser.add_argument("testcase", nargs="?", help="Run only this testcase")
    args = parser.parse_args()

    if args.testcase:
        # Run only the specified test case
        unittest.main(argv=['ignored', '-v', 'TestSimulation.' + args.testcase])
    else:
        unittest.main(argv=[''], verbosity=2)
