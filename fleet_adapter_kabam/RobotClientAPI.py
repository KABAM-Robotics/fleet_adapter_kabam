from urllib.error import HTTPError
from xmlrpc.client import Boolean
import requests
import math

'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(
            self,
            logger,
            robot_prefix: str,
            robot_user: str,
            robot_password: str,
            cloud_prefix: str,
            cloud_user: str,
            cloud_password: str,):
        # Initialising access credentials to Robot and Cloud API.
        self.robot_prefix = robot_prefix
        self.robot_user = robot_user
        self.robot_password = robot_password
        self.cloud_prefix = cloud_prefix
        self.cloud_user = cloud_user
        self.cloud_password = cloud_password

        self.ros_logger = logger
        self.connected = False
        self.api_url = robot_prefix
        self.pause_pose = False
        self.last_pose = None
        self.last_goal = None
        self.robot_status = {}
        self.buffer_threshold = 0.5  # meters

        self.ros_logger.info("Connecting to Robot...")
        self.connected = self.check_connection()
        # Test connectivity
        if self.connected:
            self.ros_logger.info("Successfully able to query API server")
            self.update_robot_status()
        else:
            logger.error("Unable to query API server")

    def check_connection(self):
        """Return True if connection to the robot API server is successful."""
        url = f'{self.api_url}/robot/status'
        try:
            response = requests.get(url)
            if response.status_code == 200:
                return True
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False

    def update_robot_status(self):
        """Return True if connection to the robot API server is successful."""
        url = self.api_url + '/robot/status'
        try:
            response = requests.get(url)
            if response.status_code == 200:
                self.robot_status = response.json()
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the Robot frame."""
        if self.pause_pose:
            return self.last_pose
        url = self.api_url + '/robot/pose'
        try:
            response = requests.get(url)
            if response.status_code == 200:
                x = response.json()['x']
                y = response.json()['y']
                theta = response.json()['theta']
                self.last_pose = (x, y, theta)
                return x, y, theta
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return None

    def pause_localization(self):
        self.ros_logger.info("Pausing Localization")
        self.pause_pose = True

    def resume_localization(self):
        self.ros_logger.info("Resuming Localization")
        self.pause_pose = False

    def is_at_destination(self, robot_name: str, pose):
        curr_x, curr_y, _ = self.position(robot_name)
        is_x_in_range = abs(pose[0] - curr_x) < self.buffer_threshold
        is_y_in_range = abs(pose[1] - curr_y) < self.buffer_threshold

        if is_x_in_range and is_y_in_range:
            return True
        else:
            return False

    def navigate(self, robot_name: str, pose):
        """
        Request the robot to navigate in Robot frame.

        This function returns True if the robot has accepted the request,
        """
        # Check if robot is already at destination to prevent unnecessary wiggle motion
        if self.is_at_destination(robot_name, pose):
            self.ros_logger.info("Robot is already at destination...")
            return True
        self.ros_logger.info("Sending navigation request...")

        url = self.api_url + '/robot/command/go_to'
        pose_json = {}
        pose_json["x"] = pose[0]
        pose_json["y"] = pose[1]
        pose_json["theta"] = pose[2]
        self.last_goal = (pose[0], pose[1], pose[2])
        try:
            response = requests.post(url, json=pose_json)
            if response.status_code == 200:
                return True
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False

    def start_process(self, robot_name: str, process: str, map_name: str):
        """
        Request the robot to begin a process.

        This function return True if the robot has accepted the request.
        """
        return False

    def stop(self, robot_name: str):
        """Command the robot to stop. Return True if robot has successfully stopped."""
        url = self.api_url + '/robot/trigger/stop'
        try:
            response = requests.post(url)
            if response.status_code == 200:
                return True
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False

    def navigation_remaining_duration(self, robot_name: str):
        """Return the number of seconds remaining for the robot to reach its destination."""
        try:
            x_diff = self.last_pose[0] - self.last_goal[0]
            y_diff = self.last_pose[1] - self.last_goal[1]
            euc_D = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        except ZeroDivisionError:
            # Defaults to 30 seconds
            euc_D = 30*0.2
        return (euc_D/0.2)

    def navigation_completed(self, robot_name: str, target_robot_pose):
        """Return True if the robot has successfully completed its previous navigation request."""
        self.ros_logger.info("Checking [navigation_completed]...")

        assert target_robot_pose is not None

        if self.is_at_destination(robot_name, target_robot_pose):
            return True
        else:
            return False

    def process_completed(self, robot_name: str):
        """Return True if the robot has successfully completed its previous process request."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def localize(self, robot_name: str, pose):
        """
        Request the robot to localize to Robot Coordinate Frame.

        This function should return True if the robot has accepted
        the request, else False.
        """
        url = self.api_url + '/robot/command/localize'
        pose_json = {}
        pose_json["x"] = pose[0]
        pose_json["y"] = pose[1]
        pose_json["theta"] = pose[2]
        try:
            response = requests.post(url, json=pose_json)
            if response.status_code == 200:
                return True
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False

    def switch_map(self, map_name: str):
        """Trigger the robot to switch its map for going to new floor."""
        url = self.api_url + '/robot/command/switch_map'

        # TODO (cardboardcode): Implement simple python function that finds
        # the corresponding Map Name to Map ID which was received in robot status.
        # Feed Map Name as map_name here to get it to work.
        payload = {
            "map_name": f"{map_name}"
        }
        try:
            response = requests.post(url, json=payload)
            if response.status_code == 200:
                return Boolean(response.json())
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False

    def trigger_docking(self):
        """Triggers the docking process for the robot."""
        url = self.api_url + '/robot/trigger/trigger_dock'
        try:
            response = requests.post(url)
            if response.status_code == 200:
                return True
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False

    def rotate(self, robot_name: str, theta):
        if self.last_pose is None:
            return False
        self.navigate(
            robot_name,
            (self.last_pose[0],
             self.last_pose[1],
             theta),
            None
            )
        return True

    def battery_soc(self, robot_name: str):
        """
        Return the state of charge of the robot.

        This function should return a value between 0.0
        and 1.0. Else return None if any errors are encountered.
        """
        return float((self.robot_status['battery']['level'])/100)

    def is_charging(self):
        """Check if the robot is currently charging."""
        return self.robot_status['battery']['charging']

    def current_map(self):
        """Return the current map name."""
        url = f'{self.api_url}/robot/mapinfo'
        try:
            response = requests.get(url)
            data = response.json()
            if response.status_code == 200:
                return data["map_label"]
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
            return None
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
            return None

    def is_initialized_finished(self):
        """Check if the robot's initialization process has finished."""
        return self.robot_status['navigation']['localization']

    def is_localised(self):
        """Check if the robot's initialization process has finished."""
        url = self.api_url + '/robot/status'
        try:
            response = requests.get(url)
            if response.status_code == 200:
                local_robot_status = response.json()
                return local_robot_status['navigation']['localization']
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")

    def trigger_image_capture(self):
        """Trigger the image capture process."""
        url = self.api_url + '/robot/command/camera_capture'
        payload = {
            "categories_list": "Image Capture",
            "capture_duration": 1,
            "interval": 1
        }
        try:
            response = requests.post(url, json=payload)
            if response.status_code == 200:
                return True
        except HTTPError as http_err:
            self.ros_logger.warn(f"HTTP error: {http_err}")
        except Exception as err:
            self.ros_logger.warn(f"Other error: {err}")
        return False
