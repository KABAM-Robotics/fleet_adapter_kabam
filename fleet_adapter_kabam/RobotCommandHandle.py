from rclpy.duration import Duration
import time
import rmf_adapter as adpt

from .RobotClientAPI import RobotAPI

import numpy as np

import threading
import math
import enum

from datetime import timedelta


# States for RobotCommandHandle's state machine used when guiding robot along
# a new path
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2


class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 input_graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api: RobotAPI,
                 waypoints_info):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = input_graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        # Get the index of the charger waypoint
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, f"Charger waypoint {charger_waypoint} \
          does not exist in the navigation graph"
        self.charger_waypoint = charger_waypoint
        self.charger_waypoint_index = waypoint.index
        self.charger_is_set = False
        self.update_frequency = update_frequency
        self.update_handle = None  # RobotUpdateHandle
        self.battery_soc = 1.0
        self.api = api
        self.position = position  # (x,y,theta) in RMF coordinates (meters, radians)
        self.initialized = False
        self.state = RobotState.IDLE
        self.dock_name = ""
        self.adapter = adapter
        self.is_switch_map_called = False
        self.is_mov_out_from_lift = False
        self.is_map_switch_triggered = False
        self.is_localization_called = False
        self.localization_time_count = 30

        self.waypoint_network = waypoints_info

        self.requested_waypoints = []  # RMF Plan waypoints
        self.remaining_waypoints = []
        self.path_finished_callback = None
        self.next_arrival_estimator = None
        self.path_index = 0
        self.docking_finished_callback = None
        self.map_transition = None
        self.localization_waypoint = None

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        # if robot is waiting at a waypoint. This is a Graph::Waypoint index
        self.on_waypoint = None
        # if robot is travelling on a lane. This is a Graph::Lane index
        self.on_lane = None
        self.target_waypoint = None  # this is a Plan::Waypoint
        # The graph index of the waypoint the robot is currently docking into
        self.dock_waypoint_index = None

        # Threading variables
        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()

        self.node.get_logger().info(
            f"The robot is starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        # Update tracking variables
        if start.lane is not None:  # If the robot is on a lane
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:  # Otherwise, the robot is on a waypoint
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        self.state_update_timer = self.node.create_timer(
            1.0 / self.update_frequency,
            self.update)

        self.status_update_timer = self.node.create_timer(
            4.5 / self.update_frequency,
            self.update_robot_status)

        self.initialized = True

        # Check if current map is not target_map.
        # If true, call switch_map
        target_map_name = self.map_name
        if self.map_name != self.api.current_map():
            response = self.api.switch_map(target_map_name)
            if response:
                self.node.get_logger().info("Robot switch map successful")
                self.is_localization_called = False
                self.is_map_switch_triggered = True
            else:
                self.node.get_logger().error("Failed to call robot to switch map")

        # Wait for map on Robot to be correct
        if target_map_name != self.api.current_map():
            self.node.get_logger().info(f"SWITCHING MAP to [{target_map_name}] " +
                                        "from [{self.api.current_map()}]")
            while target_map_name != self.api.current_map():
                time.sleep(1)

        # Localize to initial waypoint
        if not self.api.is_localised:
            self.node.get_logger().info(f"start.waypoint = {start.waypoint}")
            self.api.localize(
                self.name,
                [self.position[0], self.position[1], self.position[2]])

        # Wait for robot status to return localization true.
        while not self.api.is_localised():
            time.sleep(1)
        self.node.get_logger().info("ROBOT LOCALIZATION --> SUCCESS")

    def sleep_for(self, seconds):
        goal_time =\
          self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def clear(self):
        with self._lock:
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.path_finished_callback = None
            self.next_arrival_estimator = None
            self.docking_finished_callback = None
            self.state = RobotState.IDLE

    def stop(self):
        # Stop the robot. Tracking variables should remain unchanged.
        while True:
            self.node.get_logger().info("Requesting robot to stop...")
            if self.api.stop(self.name):
                break
            self.sleep_for(0.1)
        if self._follow_path_thread is not None:
            self._quit_path_event.set()
            if self._follow_path_thread.is_alive():
                self._follow_path_thread.join()
            self._follow_path_thread = None
            self.clear()

    def follow_new_path(
            self,
            waypoints,
            next_arrival_estimator,
            path_finished_callback):

        self.stop()
        self._quit_path_event.clear()

        self.node.get_logger().info("[follow_new_path] - Received new path to follow...")

        for x in range(len(waypoints)):
            is_waypoint = False
            for waypoint_name in self.waypoint_network:
                if(waypoints[x].position[0] == self.waypoint_network[waypoint_name][0] and
                   waypoints[x].position[1] == self.waypoint_network[waypoint_name][1]):
                    is_waypoint = True
                    self.node.get_logger().info(f'{x}. waypoint received = {waypoint_name}')
            if not is_waypoint:
                self.node.get_logger().info(f'{x}. waypoint received = {waypoints[x].position}')

        self.remaining_waypoints = self.get_remaining_waypoints(waypoints)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None
        self.next_arrival_estimator = next_arrival_estimator
        self.path_finished_callback = path_finished_callback

        self.node.get_logger().info("Waypoints: ")
        for waypoint in self.remaining_waypoints:
            tagwyp = waypoint[1]
            [o, p] = tagwyp.position[:2]
            try:
                tgmap = self.graph.get_waypoint(tagwyp.graph_index).map_name
                self.node.get_logger().info(
                    self.graph.get_waypoint(tagwyp.graph_index).waypoint_name)
                self.node.get_logger().info(
                    f"--- {o}, {p}, {tgmap}")
            except Exception:
                self.node.get_logger().error(
                    "Error getting map_name from waypoint")
                tgmap = None

        def _follow_path():
            target_pose = []
            while(self.remaining_waypoints or
                  self.state == RobotState.MOVING or
                  self.state == RobotState.WAITING):
                # Check if we need to abort
                if self._quit_path_event.is_set():
                    self.node.get_logger().info("Aborting previously followed "
                                                "path")
                    return
                # State machine
                if self.state == RobotState.IDLE:
                    # Assign the next waypoint
                    self.target_waypoint = self.remaining_waypoints[0][1]
                    self.path_index = self.remaining_waypoints[0][0]
                    # Move robot to next waypoint
                    target_pose = self.target_waypoint.position
                    # Try getting map name
                    target_map = None
                    try:
                        target_map = self.graph.get_waypoint(
                            self.target_waypoint.graph_index).map_name
                        self.node.get_logger().info(
                            "Identified [target_map]:", str(target_map))
                    except Exception:
                        if target_map is None:
                            target_map = self.map_name

                    self.node.get_logger().info(
                        f"str(target_map) = {str(target_map)}")

                    if self.is_switch_map_called:
                        self.node.get_logger().info(
                            "Robot is in switch map state")
                        self.state = RobotState.MOVING
                    elif (str(target_map) != self.api.current_map() and
                          self.map_transition is None):
                        self.node.get_logger().info(
                            "Target map differs from current map")

                        if not self.is_switch_map_called:
                            self.node.get_logger().info("Calling API to switch map to: " +
                                                        str(target_map))
                            # To get the respective map exit point
                            # get the next lift exit waypoint name
                            self.localize_waypoint = self.remaining_waypoints[1][1]
                            try:
                                localize_waypoint_name = self.graph.get_waypoint(
                                    self.localize_waypoint.graph_index).waypoint_name
                                self.node.get_logger().info(
                                    "Calling API to switch map to: " + str(target_map))
                                self.node.get_logger().info(
                                    f"localise_waypoint_name: {localize_waypoint_name}")
                            except Exception:
                                self.node.get_logger().info(
                                    "problem getting localize_waypoint name")

                            self.node.get_logger().info(
                                f"localize_waypoint name {localize_waypoint_name}")

                            self.node.get_logger().info(f"target_map: {target_map}")

                            exit_waypoint = None
                            for _, floors in self.config["map_exit_point"].items():
                                rmf_target_map_name = target_map
                                if(rmf_target_map_name in floors and
                                   floors[rmf_target_map_name] == localize_waypoint_name):
                                    exit_waypoint_name = floors[self.map_name]
                                    exit_waypoint = self.graph.find_waypoint(
                                        exit_waypoint_name)
                                    break
                            if exit_waypoint is None:
                                self.node.get_logger().error(
                                    "Waypoint not found in exit points")
                            exit_pose = exit_waypoint.location  # 2D Vector
                            # Call the robot to move out from the lift
                            [x, y] = self.transforms[self.map_name]["rmf_to_robot"].transform(
                                exit_pose[:2])
                            theta = 0.01
                            for waypoint_name in self.waypoint_network:
                                if(target_pose[0] == self.waypoint_network[waypoint_name][0] and
                                   target_pose[1] == self.waypoint_network[waypoint_name][1]):
                                    self.node.get_logger().info(
                                        f"Robot {self.name} about to call " +
                                        f"navigate() for RMF Waypoint {waypoint_name}...")
                                    self.node.get_logger().info(
                                        f"Robot {self.name} about to call navigate() for" +
                                        f" robot pose {x}, {y}, {theta}")

                            response = self.api.navigate(
                                self.name,
                                [x, y, theta])
                            self.map_transition = target_map
                            self.is_switch_map_called = True
                            self.is_mov_out_from_lift = False
                            if response:
                                self.node.get_logger().info(
                                    "Moving out from Lift")
                            else:
                                self.node.get_logger().error(
                                    "Failed to move out from lift")
                            # Update robot status for updateRobotHandling override error
                            self.node.get_logger().info(
                                "RobotState is switch to MOVING for switch map process")
                            self.on_waypoint = self.target_waypoint.graph_index
                            self.state = RobotState.MOVING
                        else:
                            self.node.get_logger().info(
                                "Already called api to switch map previously.")
                            self.node.get_logger().info(
                                "RobotState is switch to MOVING for switch map process")
                            self.state = RobotState.MOVING
                    elif (self.api.is_initialized_finished() and
                          self.api.current_map() == str(target_map)):
                        # if the target map same as current map, doing navigation
                        [x, y] = self.transforms[self.map_name]["rmf_to_robot"].transform(
                            target_pose[:2])
                        theta = (target_pose[2] +
                                 self.transforms[self.map_name]['orientation_offset'])
                        for waypoint_name in self.waypoint_network:
                            if(target_pose[0] == self.waypoint_network[waypoint_name][0] and
                               target_pose[1] == self.waypoint_network[waypoint_name][1]):
                                self.node.get_logger().info(
                                    f"Robot {self.name} about to call " +
                                    f"navigate() for RMF Waypoint {waypoint_name}...")
                        self.node.get_logger().info(
                                f"Robot {self.name} about to " +
                                f"call navigate() for robot pose {x}, {y}, {theta}")
                        response = self.api.navigate(self.name,
                                                     [x, y, theta])
                        if response:
                            self.node.get_logger().info(
                                f"Robot {self.name} is navigating to rmf_pose {target_pose[:2]} ")
                            self.remaining_waypoints = self.remaining_waypoints[1:]
                            self.state = RobotState.MOVING
                        else:
                            self.node.get_logger().info(
                                f"Robot {self.name} failed to navigate to " +
                                f"target [{x:.2f}, {y:.2f}] " +
                                f"pose_index {self.target_waypoint.graph_index} "
                                f"Retrying...")
                            self.sleep_for(0.5)
                    else:
                        self.node.get_logger().warn(
                            "Robot will not move due to some state is not fulfill")
                        self.node.get_logger().warn(
                            f"target_map:{target_map} str(): " +
                            f"{str(target_map)} " +
                            f"robot_map:{self.api.current_map()} {self.map_name}")
                        self.node.get_logger().warn(
                            f"localization status:{self.api.is_initialized_finished()}")
                        self.sleep_for(0.5)
                elif self.state == RobotState.WAITING:
                    self.sleep_for(0.1)
                    time_now = self.adapter.now()
                    with self._lock:
                        if self.target_waypoint is not None:
                            waypoint_wait_time = self.target_waypoint.time
                            if (waypoint_wait_time < time_now):
                                self.state = RobotState.IDLE
                            else:
                                if self.path_index is not None:
                                    self.node.get_logger().info(
                                        f"Waiting for "
                                        f"{(waypoint_wait_time - time_now).seconds}s")
                                    self.next_arrival_estimator(
                                        self.path_index, timedelta(seconds=0.0))

                elif self.state == RobotState.MOVING:
                    self.sleep_for(0.1)
                    self.node.get_logger().info("RobotState : MOVING begin")
                    # Check if we have reached the target
                    with self._lock:
                        # Update the lane the robot is on
                        lane = self.get_current_lane()

                        # If is_switch_map called is true, it is in switch map process
                        if self.is_switch_map_called:
                            self.node.get_logger().info("Switch map was called")
                            current_robot_map = self.api.current_map()

                            # update rmf that the robot is the waypoint outside of Lift
                            # By this point will keep monitoring to
                            # see whether it reach lift access point
                            if not self.is_mov_out_from_lift:
                                if self.api.navigation_completed(
                                    self.name,
                                    self.transforms[self.map_name]["rmf_to_robot"].transform(
                                        target_pose[:2])):
                                    try:
                                        self.node.get_logger().info(
                                            "Robot reach lift access point")
                                        next_waypoint = self.remaining_waypoints[1][1]
                                        self.on_waypoint = next_waypoint.graph_index
                                        self.last_known_waypoint_index = self.on_waypoint
                                    except Exception as e:
                                        self.node.get_logger().warn(e)
                                        self.node.get_logger().warn(
                                            "Assigning on_waypoint to rmf pose index" +
                                            f" {self.target_waypoint.graph_index}")
                                        self.on_waypoint = self.target_waypoint.graph_index
                                    self.state = RobotState.WAITING
                                    self.is_mov_out_from_lift = True
                                    # After Robot is move out from lift,
                                    # Clear the remaining waypoint
                                    # inside lift and outside lift waypoint
                                    # By doing this, it will trigger
                                    # self.path_finished_callback()
                                    # to inform RMF robot is already out of lift already
                                    self.remaining_waypoints = self.remaining_waypoints[2:]
                                else:
                                    self.node.get_logger().info("robot is moving out from Lift")
                                    self.on_waypoint = None
                                    self.on_lane = lane
                                    duration = self.api.navigation_remaining_duration(self.name)
                                    if self.path_index is not None:
                                        self.next_arrival_estimator(
                                            self.path_index, timedelta(seconds=duration))
                            # it is still switching map/ call to switch map
                            elif not self.is_map_switch_triggered:
                                self.node.get_logger().info(
                                    f"robot switch map to {self.map_transition} " +
                                    f"- current robot map {current_robot_map}")
                                response = self.api.switch_map(self.map_transition)
                                if response:
                                    self.node.get_logger().info("Robot switch map successful")
                                    self.is_localization_called = False
                                    self.is_map_switch_triggered = True
                                else:
                                    self.node.get_logger().error(
                                        "Failed to call robot to switch map")

                                self.on_waypoint = None
                                self.on_lane = lane

                            elif not self.is_localization_called:
                                loc_pose = self.localize_waypoint.position
                                # Call the robot to localize
                                [x, y] = (
                                    self.transforms
                                    [self.map_transition]["rmf_to_robot"].transform(
                                        loc_pose[:2]))
                                theta = loc_pose[2] + \
                                    self.transforms[self.map_transition]['orientation_offset']
                                self.node.get_logger().info(
                                        f"Robot {self.name} about to call initialize" +
                                        f" for rmf_pose {loc_pose[:2]} ")
                                self.node.get_logger().info(
                                        f"Robot {self.name} about to call initialize" +
                                        f" for robot pose {x}, {y}, {theta}")
                                res = self.api.localize(self.name,
                                                        [x, y, theta])
                                if res:
                                    self.node.get_logger().info(
                                        f"Robot {self.name} successfully called to " +
                                        f"initialize for rmf_pose {loc_pose[:2]} ")
                                    self.is_localization_called = True
                                else:
                                    self.node.get_logger().warn(
                                        f"Robot {self.name} fail to call to initialize for " +
                                        f"rmf_pose {loc_pose[:2]} ")

                                self.sleep_for(3)

                                self.on_waypoint = None
                                self.on_lane = lane

                            # If Robot is localized, switch map process has completed.
                            # Proceed to RobotState.Waiting to accept the next RMF waypoint
                            elif not self.api.is_initialized_finished():
                                if self.localization_time_count == 0:
                                    self.node.get_logger().warn(
                                        f"Robot {self.name} not localize " +
                                        "more than 30 seconds, recall localiation")
                                    self.is_localization_called = False
                                    self.localization_time_count = 30
                                self.sleep_for(1)
                                self.localization_time_count = self.localization_time_count - 1
                                self.on_waypoint = \
                                    self.target_waypoint.graph_index
                                self.last_known_waypoint_index = \
                                    self.on_waypoint
                                self.node.get_logger().info("Localising Robot...")
                            elif (self.api.is_initialized_finished() and
                                  current_robot_map == self.map_transition):
                                self.localization_waypoint = None
                                self.map_name = self.map_transition
                                self.map_transition = None
                                self.is_switch_map_called = False
                                self.is_map_switch_triggered = False
                                self.is_localization_called = False
                                self.localization_time_count = 30
                                self.state = RobotState.WAITING
                                if (self.target_waypoint.graph_index is not None):
                                    self.on_waypoint = \
                                        self.target_waypoint.graph_index
                                    self.last_known_waypoint_index = \
                                        self.on_waypoint
                                    self.remaining_waypoints = self.remaining_waypoints[1:]
                                else:
                                    self.on_waypoint = None  # still on a lane
                            else:
                                self.node.get_logger().warn(
                                    f"target_map:{target_map} str(): " +
                                    f"{str(target_map)} " +
                                    f"robot_map:{self.api.current_map()} {self.map_name}")
                                self.node.get_logger().warn(
                                    "localization status:" +
                                    f"{self.api.is_initialized_finished()}")
                                self.on_waypoint = None
                            duration = self.api.navigation_remaining_duration(self.name)
                            if self.path_index is not None:
                                self.next_arrival_estimator(
                                    self.path_index, timedelta(seconds=duration))

                        # Normal Navigate process
                        elif (self.api.navigation_completed(
                                self.name,
                                self.transforms[self.map_name]["rmf_to_robot"].transform(
                                    target_pose[:2]))):
                            self.node.get_logger().info(
                                f"Robot {self.name} has reached its target " +
                                f"rmf_pose {target_pose[:2]} " +
                                f"pose_index {self.target_waypoint.graph_index}")
                            self.state = RobotState.WAITING
                            if (self.target_waypoint.graph_index is not None):
                                self.on_waypoint = \
                                    self.target_waypoint.graph_index
                                self.last_known_waypoint_index = \
                                    self.on_waypoint
                            else:
                                self.on_waypoint = None  # still on a lane
                        elif lane is not None:
                            self.node.get_logger().info("Lane")
                            self.on_waypoint = None
                            self.on_lane = lane
                        # The robot may either be on the previous
                        # waypoint or the target one
                        elif (self.target_waypoint.graph_index is not None and
                              self.dist(self.position, target_pose) < 0.5):
                            self.on_waypoint = self.target_waypoint.graph_index
                            self.node.get_logger().info("Setting on_waypoint to" +
                                                        f" {self.on_waypoint}")
                        elif (self.last_known_waypoint_index is not None and
                              self.dist(
                                self.position,
                                (self.graph.get_waypoint(
                                    self.last_known_waypoint_index).location)
                                ) < 0.5):
                            self.on_waypoint = self.last_known_waypoint_index
                            self.node.get_logger().info("Setting on_waypoint to " +
                                                        f"last known waypoint: {self.on_waypoint}")
                        else:
                            self.on_lane = None  # update_off_grid()
                            self.on_waypoint = None
                            self.node.get_logger().info("Not on lane nor waypoint")
                        duration = self.api.navigation_remaining_duration(self.name)
                        if self.path_index is not None:
                            self.next_arrival_estimator(
                                self.path_index, timedelta(seconds=duration))
                    self.node.get_logger().info("RobotState : MOVING end\n")
            self.path_finished_callback()
            self.node.get_logger().info(
                f"Robot {self.name} has successfully navigated along "
                f"requested path.")

        self._follow_path_thread = threading.Thread(
            target=_follow_path)
        self._follow_path_thread.start()

    def dock(
            self,
            dock_name,
            docking_finished_callback):

        self.node.get_logger().info("[dock] - Docking triggered...")
        self._quit_dock_event.clear()
        if self._dock_thread is not None:
            self._dock_thread.join()

        self.dock_name = dock_name
        assert docking_finished_callback is not None
        self.docking_finished_callback = docking_finished_callback

        # Get the waypoint that the robot is trying to dock into
        dock_waypoint = self.graph.find_waypoint(self.dock_name)
        assert(dock_waypoint)
        self.dock_waypoint_index = dock_waypoint.index

        def _dock():
            # Request the robot to start the relevant process
            self.node.get_logger().info(
                f"Requesting robot {self.name} to dock at {self.dock_name}")

            with self._lock:
                self.on_waypoint = None
                self.on_lane = None
            self.sleep_for(0.1)

            if dock_name == self.charger_waypoint:
                self.api.trigger_docking()
            else:
                self.node.get_logger().info("Dock point assigned but " +
                                            "it is not same as charger waypoint")

            while (not self.api.is_charging()):
                # Check if we need to abort
                if self._quit_dock_event.is_set():
                    self.node.get_logger().info("Aborting docking")
                    return
                self.node.get_logger().info("Robot is docking...")
                self.sleep_for(0.1)

            with self._lock:
                self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
                self.docking_finished_callback()
                self.node.get_logger().info("Docking completed")

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    def default_docking(self):
        if self.dock_name not in self.docks:
            self.node.get_logger().info(
                f"Requested dock {self.dock_name} not found, "
                "ignoring docking request"
            )
            # TODO(MXG): This should open an issue ticket for the robot
            # to tell the operator that the robot cannot proceed
            return False

    def get_position(self):
        """Return the live position of the robot in Robot Frame in [x,y,theta] meters."""
        if self.is_localization_called and not self.api.is_initialized_finished:
            if self.localization_waypoint is not None:
                return self.localization_waypoint.position
            else:
                self.node.get_logger().error(
                    "Unable to retrieve position from robot.")
                return self.position
        else:
            position = self.api.position(self.name)
            if position is not None:
                x, y = self.transforms[self.map_name]['robot_to_rmf'].transform(
                    [position[0], position[1]])
                theta = math.radians(position[2]) - \
                    self.transforms[self.map_name]['orientation_offset']

                theta = math.degrees(theta)
                return [x, y, theta]
            else:
                self.node.get_logger().error(
                    "Unable to retrieve position from robot.")
                return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        else:
            self.node.get_logger().error(
                "Unable to retrieve battery data from robot.")
            return self.battery_soc

    def update(self):
        self.position = self.get_position()
        self.battery_soc = self.get_battery_soc()
        if self.update_handle is not None:
            self.update_state()

    def update_next_arrival_time(self):
        if self.path_index is not None:
            self.next_arrival_estimator(
                self.path_index, timedelta(seconds=5.0))

    # Update robot state's status
    def update_robot_status(self):
        """
        Update the status of the robot.

        This function calls the `update_robot_status` method of the `api` object
        to update the robot's status.
        """
        self.api.update_robot_status()
        if self.api.is_charging():
            self.node.get_logger().warn(f"Robot {self.name} is charging")
        elif not self.api.check_connection():
            self.node.get_logger().warn(f"Robot {self.name} is offline")
        #     self.update_handle.override_status("offline")

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        if not self.charger_is_set:
            if ("max_delay" in self.config.keys()):
                max_delay = self.config["max_delay"]
                self.node.get_logger().info(
                    f"Setting max delay to {max_delay}s")
                self.update_handle.set_maximum_delay(max_delay)
            if (self.charger_waypoint_index < self.graph.num_waypoints):
                self.update_handle.set_charger_waypoint(
                    self.charger_waypoint_index)
            else:
                self.node.get_logger().warn(
                    "Invalid waypoint supplied for charger. "
                    "Using default nearest charger in the map")
            self.charger_is_set = True
        # Update position
        with self._lock:
            if (self.on_waypoint is not None):  # if robot is on a waypoint
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif (self.on_lane is not None):  # if robot is on a lane
                # We only keep track of the forward lane of the robot.
                # However, when calling this update it is recommended to also
                # pass in the reverse lane so that the planner does not assume
                # the robot can only head forwards. This would be helpful when
                # the robot is still rotating on a waypoint.
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:  # Unidirectional graph
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif (self.dock_waypoint_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            # if robot is merging into a waypoint
            elif (self.target_waypoint is not None and
                  self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:  # if robot is lost
                self.update_handle.update_lost_position(
                    self.map_name, self.position)

    def get_current_lane(self):
        """
        Determine which approach lane the robot is currently positioned in.

        This method iterates through all approach lanes associated with the target
        waypoint and checks if the current position of the robot lies within each lane's
        bounds. The first lane that contains the robot's position is returned as
        the current lane index.
        """
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):
            px, py, _ = current_position
            p = np.array([px, py])
            t = np.array(target_position)
            entry = np.array(lane_entry)
            exit = np.array(lane_exit)
            return np.dot(p - t, exit - entry)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        # Spin on the spot
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        # Determine which lane the robot is currently on
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:  # The robot is on this lane
                return lane_index
        return None

    def dist(self, A, B):
        """Return Euclidian distance between A(x,y) and B(x,y)."""
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_remaining_waypoints(self, waypoints: list):
        """Return a list of RMF waypoints."""
        assert(len(waypoints) > 0)
        remaining_waypoints = []

        for i in range(len(waypoints)):
            remaining_waypoints.append((i, waypoints[i]))
        return remaining_waypoints
