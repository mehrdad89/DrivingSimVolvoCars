import ctypes
import sys
import os
import logging
import time
import xml.etree.ElementTree as ET
import argparse  # For argument parsing
from constants import *

# Setup logging
logging.basicConfig(filename='simulation.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


# Load the simulation library based on platform
def load_library():
    """
    Load the ESmini simulation library based on the platform.
    Exits the program if the platform is unsupported.
    """
    logging.info("Loading ESmini library...")
    if sys.platform == "linux" or sys.platform == "linux2":
        return ctypes.CDLL("../bin/libesminiLib.so")
    elif sys.platform == "darwin":
        return ctypes.CDLL("../bin/libesminiLib.dylib")
    elif sys.platform == "win32":
        return ctypes.CDLL("../bin/esminiLib.dll")
    else:
        logging.error("Unsupported platform: %s", sys.platform)
        sys.exit(1)


def dynamic_safe_distance(current_speed):
    """
    Calculate dynamic safe following distance based on current speed.
    Minimum safe distance is set to 5 meters.
    """
    safe_distance = max(5.0, current_speed * 0.5)
    logging.info(f"Safe distance calculated: {safe_distance:.2f} meters (Speed: {current_speed:.2f} m/s)")
    return safe_distance


# ACC Logic
def predict_lead_vehicle_position(current_speed, lead_acceleration, time_interval):
    """
    Predict the position of the lead vehicle after a time interval based on speed and acceleration.
    """
    position = current_speed * time_interval + 0.5 * lead_acceleration * (time_interval ** 2)
    logging.debug(f"Predicted lead vehicle position after {time_interval} seconds: {position:.2f} meters")
    return position


def smooth_speed_transition(current_speed, target_speed, max_acceleration, time_interval):
    """
    Smoothly transition the speed of the ego vehicle towards the target speed using maximum acceleration.
    """
    speed_diff = target_speed - current_speed
    if abs(speed_diff) < 0.1:
        logging.debug(f"Target speed {target_speed:.2f} reached (Current speed: {current_speed:.2f}).")
        return target_speed

    accel = max_acceleration if speed_diff > 0 else -max_acceleration
    smooth_speed = current_speed + accel * time_interval
    logging.info(f"Smooth speed transition: {current_speed:.2f} -> {smooth_speed:.2f} (Target: {target_speed:.2f})")
    return smooth_speed


def handle_emergency_cut_in(current_speed, headway_time, min_safe_distance, emergency_brake_acceleration):
    """
    Handle emergency cut-in events by triggering emergency braking if the headway time is dangerously low.
    """
    if headway_time < 0.5:  # Example threshold for emergency
        logging.warning("Emergency cut-in detected! Triggering emergency brake.")
        return max(current_speed - emergency_brake_acceleration, 0)
    return current_speed


def adjust_speed_for_weather(current_speed, weather_type):
    """
    Adjust the ego vehicle's speed based on weather conditions.
    Reduces speed in rain and snow to simulate reduced traction.
    """
    if weather_type == "rain":
        adjusted_speed = current_speed * RAIN_SPEED_REDUCTION  # Reduce speed by 15% in rain
        logging.info(f"Adjusting speed for rain: {current_speed:.2f} -> {adjusted_speed:.2f}")
        return adjusted_speed
    elif weather_type == "snow":
        adjusted_speed = current_speed * SNOW_SPEED_REDUCTION  # Reduce speed by 25% in snow
        logging.info(f"Adjusting speed for snow: {current_speed:.2f} -> {adjusted_speed:.2f}")
        return adjusted_speed
    return current_speed


def adjust_speed_for_curvature(current_speed, road_curvature):
    """
    Adjust the ego vehicle's speed based on the curvature of the road.
    Reduces speed on sharp curves to enhance safety.
    """
    if road_curvature > 0.1:  # Example threshold for sharper curves
        adjusted_speed = current_speed * (1 - min(road_curvature, 0.2))  # Reduce speed based on curve sharpness
        logging.info(f"Adjusting speed for road curvature: {current_speed:.2f} -> {adjusted_speed:.2f}")
        return adjusted_speed
    return current_speed


def handle_jerk_control(current_speed, target_speed, max_acceleration, max_deceleration, jerk_limit, time_interval):
    """
    Apply jerk limits to smooth acceleration and deceleration.
    Controls how quickly the ego vehicle can change its speed to avoid abrupt transitions.
    """
    speed_diff = target_speed - current_speed
    if abs(speed_diff) < 0.1:
        logging.debug(f"Jerk control: Target speed {target_speed:.2f} reached (Current speed: {current_speed:.2f}).")
        return target_speed

    max_acceleration_change = jerk_limit * time_interval
    if speed_diff > 0:
        accel = min(max_acceleration, max_acceleration_change)
    else:
        accel = -min(max_deceleration, max_acceleration_change)

    smooth_speed = current_speed + accel * time_interval
    logging.info(f"Jerk-controlled speed transition: {current_speed:.2f} -> {smooth_speed:.2f}")
    return smooth_speed


def update_ego_speed(current_speed, lead_vehicle_speed, lead_acceleration, headway_time, target_headway_time,
                     max_acceleration, max_deceleration, min_safe_distance, road_curvature, weather_type, jerk_limit):
    """
    Update the speed of the ego vehicle based on various factors including the speed of the lead vehicle,
    road conditions, weather, and adaptive cruise control logic.
    """
    time_interval = 0.5  # Time step in seconds

    # Step 1: Predict lead vehicle's future position
    predicted_lead_position = predict_lead_vehicle_position(lead_vehicle_speed, lead_acceleration, time_interval)

    # Step 2: Dynamically adjust speed based on road curvature
    target_speed = adjust_speed_for_curvature(current_speed, road_curvature)

    # Step 3: Adjust speed based on weather conditions
    target_speed = adjust_speed_for_weather(target_speed, weather_type)

    # Step 4: If the lead vehicle is too close, decelerate
    if headway_time < target_headway_time:
        target_speed = lead_vehicle_speed - max_deceleration
        logging.info(f"Lead vehicle too close. Decelerating: {current_speed:.2f} -> {target_speed:.2f}")

    # Step 5: Smooth the speed transition using jerk control
    smooth_speed = handle_jerk_control(current_speed, target_speed, max_acceleration, max_deceleration, jerk_limit, time_interval)

    # Step 6: Handle emergency situations where the lead vehicle cuts in too close
    final_speed = handle_emergency_cut_in(smooth_speed, headway_time, min_safe_distance, max_deceleration)

    logging.info(f"Ego speed updated: {current_speed:.2f} -> {final_speed:.2f}")
    return final_speed


def acc_control_loop(current_speed, lead_vehicle_speed, lead_acceleration, headway_time, target_headway_time,
                     max_acceleration=MAX_ACCELERATION, max_deceleration=MAX_DECELERATION, jerk_limit=JERK_LIMIT,
                     road_curvature=0.05, weather_type=DEFAULT_WEATHER_TYPE, min_safe_distance_func=dynamic_safe_distance):
    """
    Control loop for adaptive cruise control (ACC). Adjusts ego vehicle speed based on lead vehicle behavior,
    road conditions, and weather.
    """
    # Dynamic safe distance based on current speed
    min_safe_distance = min_safe_distance_func(current_speed)

    # Update ego vehicle speed with improved ACC logic
    return update_ego_speed(current_speed, lead_vehicle_speed, lead_acceleration, headway_time, target_headway_time,
                            max_acceleration, max_deceleration, min_safe_distance, road_curvature, weather_type,
                            jerk_limit)


# XODR and XOSC modification functions
def modify_xodr(xodr_file, curvature=None, lane_width=None, road_length=None):
    """
    Modify the XODR file to adjust curvature, lane width, or road length.
    Saves the modified file and logs changes.
    """
    logging.info("Modifying XODR file: %s", xodr_file)
    tree = ET.parse(xodr_file)
    root = tree.getroot()

    if curvature:
        for arc in root.findall(".//geometry/arc"):
            arc.set('curvature', str(curvature))
            logging.info(f"Curvature updated to: {curvature}")

    if lane_width:
        for lane in root.findall(".//laneSection//lane"):
            for width in lane.findall("width"):
                width.set('a', str(lane_width))
                logging.info(f"Lane width updated to: {lane_width}")

    if road_length:
        for geometry in root.findall(".//geometry"):
            geometry.set('length', str(road_length))
            logging.info(f"Road length updated to: {road_length}")

    modified_file = xodr_file.replace('.xodr', '_modified.xodr')
    tree.write(modified_file)
    logging.info(f"Modified XODR file saved as: {modified_file}")
    return modified_file


def modify_xosc_with_xodr(xosc_file, xodr_file):
    """
    Modify the XOSC file to point to the updated XODR file.
    """
    logging.info(f"Modifying XOSC file to include XODR file path: {xodr_file}")
    tree = ET.parse(xosc_file)
    root = tree.getroot()

    road_network = root.find(".//RoadNetwork")
    if road_network is None:
        logging.error("No RoadNetwork section found in the XOSC file.")
        sys.exit(-1)

    logic_file = road_network.find("LogicFile")
    if logic_file is not None:
        logic_file.set('filepath', xodr_file)
        logging.info(f"Updated XODR file path in XOSC: {xodr_file}")
    else:
        logging.error("No LogicFile element found in the XOSC file.")
        sys.exit(-1)

    modified_file = xosc_file.replace('.xosc', '_with_xodr.xosc')
    tree.write(modified_file)
    logging.info(f"Modified XOSC file with XODR path saved as: {modified_file}")
    return modified_file


def modify_xosc(xosc_file, speed=None, lane_offset=None, ego_position=None, weather=None,
                headway_time=None):
    """
    Modify the XOSC file to update speed, lane offset, ego position, weather conditions, and headway time.
    Saves the modified file and logs changes.
    """
    logging.info("Modifying XOSC file: %s", xosc_file)
    tree = ET.parse(xosc_file)
    root = tree.getroot()

    if speed:
        for param in root.findall(".//ParameterDeclaration"):
            if param.get('name') == 'Speed':
                param.set('value', str(speed))
                logging.info(f"Speed updated to: {speed}")

    if lane_offset:
        for param in root.findall(".//ParameterDeclaration"):
            if param.get('name') == 'LaneOffset':
                param.set('value', str(lane_offset))
                logging.info(f"LaneOffset updated to: {lane_offset}")

    if ego_position:
        for position in root.findall(".//PrivateAction//TeleportAction//Position//LanePosition[@entityRef='Ego']"):
            position.set('s', str(ego_position))
            logging.info(f"Ego vehicle position updated to: {ego_position}")

    if weather:
        environment = root.find('.//RoadNetwork')
        if environment is not None:
            weather_element = ET.SubElement(environment, 'Environment')
            weather_conditions = ET.SubElement(weather_element, 'Weather')
            ET.SubElement(weather_conditions, 'Precipitation', type=weather['type'], intensity=weather['intensity'])
            logging.info(f"Weather conditions set to {weather['type']} with intensity {weather['intensity']}")

    if headway_time:
        for param in root.findall(".//ParameterDeclaration"):
            if param.get('name') == 'HeadwayTime_LaneChange':
                param.set('value', str(headway_time))
                logging.info(f"HeadwayTime for lane change updated to: {headway_time}")

    modified_file = xosc_file.replace('.xosc', '_modified.xosc')
    tree.write(modified_file)
    logging.info(f"Modified XOSC file saved as: {modified_file}")
    return modified_file


# Argument Parser Setup
def setup_arg_parser():
    """
    Setup argument parser for simulation parameters such as XODR file, speed, road length, lane offset, etc.
    """
    parser = argparse.ArgumentParser(description="Run simulation with adaptive cruise control.")
    parser.add_argument('xosc_file', help="The path to the XOSC file.")
    parser.add_argument('--xodr_file', help="The path to the XODR file.", default=None)
    parser.add_argument('--duration', type=float, default=SIMULATION_DURATION, help="Duration of the simulation.")
    parser.add_argument('--step_interval', type=float, default=STEP_INTERVAL, help="Step interval for the simulation.")
    parser.add_argument('--curvature', type=float, help="Curvature for the XODR file.", default=None)
    parser.add_argument('--lane_width', type=float, help="Lane width for the XODR file.", default=None)
    parser.add_argument('--road_length', type=float, help="Road length for the XODR file.", default=None)
    parser.add_argument('--speed', type=float, help="Speed for the vehicles.", default=None)
    parser.add_argument('--lane_offset', type=float, help="Lane offset for the ego vehicle.", default=None)
    parser.add_argument('--ego_position', type=float, help="Starting position for the ego vehicle.", default=None)
    parser.add_argument('--weather_type', help="Weather type (rain, snow, etc.)", default=None)
    parser.add_argument('--weather_intensity', help="Weather intensity.", default=None)
    parser.add_argument('--headway_time', type=float, help="Headway time for the cut-in scenario.", default=None)
    return parser


# Main Simulation Function
def main():
    """
    Main function for running the adaptive cruise control simulation.
    Handles XODR/XOSC modification, simulation loop, and logging.
    """
    parser = setup_arg_parser()
    args = parser.parse_args()

    xosc_file = args.xosc_file
    xodr_file = args.xodr_file
    duration = args.duration
    step_interval = args.step_interval
    curvature = args.curvature
    lane_width = args.lane_width
    road_length = args.road_length
    speed = args.speed
    lane_offset = args.lane_offset
    ego_position = args.ego_position
    weather_type = args.weather_type
    weather_intensity = args.weather_intensity
    headway_time = args.headway_time

    # Modify XODR file if parameters provided
    if any([curvature, lane_width, road_length]):
        xodr_file = modify_xodr(xodr_file, curvature=curvature, lane_width=lane_width, road_length=road_length)

    # Update XOSC file with the new XODR file path if provided
    if xodr_file:
        xosc_file = modify_xosc_with_xodr(xosc_file, xodr_file)

    # Modify XOSC file if additional parameters are provided
    weather = {'type': weather_type, 'intensity': weather_intensity} if weather_type and weather_intensity else None
    if any([speed, lane_offset, ego_position, weather, headway_time]):
        xosc_file = modify_xosc(xosc_file, speed=speed, lane_offset=lane_offset, ego_position=ego_position,
                                weather=weather, headway_time=headway_time)

    try:
        se = load_library()
        se.SE_Init(xosc_file.encode('ascii'), 0, 1, 0, 0)
    except Exception as e:
        logging.error("Error initializing simulation: %s", str(e))
        sys.exit(-1)

    start_time = time.time()
    end_time = start_time + duration if duration is not None else None

    # Simulation setup and initial parameters (same as before)
    current_speed = 20  # Initial speed for ego vehicle
    lead_vehicle_speed = 18  # Initial speed for lead vehicle
    lead_acceleration = 0  # Assume lead vehicle is constant
    headway_time = 1.5  # Initial headway time
    target_headway_time = TARGET_HEADWAY_TIME  # Safe following distance
    road_curvature = DEFAULT_CURVATURE  # Example road curvature value
    weather_type = 'rain'  # Example weather condition (rain, snow, clear)
    jerk_limit = JERK_LIMIT  # Limit on the rate of acceleration change

    try:
        while not se.SE_GetQuitFlag():
            logging.info("acc_control_loop is called.")
            current_speed = acc_control_loop(current_speed, lead_vehicle_speed, lead_acceleration, headway_time,
                                             target_headway_time, road_curvature=road_curvature, weather_type=weather_type,
                                             jerk_limit=jerk_limit)
            se.SE_Step()
            if end_time and time.time() >= end_time:
                logging.info("Simulation duration reached.")
                break
            time.sleep(step_interval)

    except KeyboardInterrupt:
        logging.info("Simulation interrupted by user.")
    except Exception as e:
        logging.error("Error during simulation: %s", str(e))
    finally:
        logging.info("Simulation ended.")


if __name__ == "__main__":
    main()
