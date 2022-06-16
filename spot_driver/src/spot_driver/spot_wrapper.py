import time
import math
import sys
from typing import List
import numpy as np
import statistics
import rospy

from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client import robot_command
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.geometry import EulerZXY

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import manipulation_api_pb2
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.frame_helpers import get_odom_tform_body, ODOM_FRAME_NAME
from bosdyn.client.power import safe_power_off, PowerClient, power_on
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import estop_pb2, image_pb2
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client import power
from bosdyn.client import frame_helpers
from bosdyn.client import math_helpers
from bosdyn.client.exceptions import InternalServerError

from . import graph_nav_util
from spot_driver.arm.arm_wrapper import ArmWrapper

from bosdyn.api import arm_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
from bosdyn.api import basic_command_pb2
from bosdyn.api import synchronized_command_pb2
from bosdyn.api import robot_command_pb2
from bosdyn.api import geometry_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.util import seconds_to_duration
from google.protobuf.duration_pb2 import Duration
from google.protobuf.timestamp_pb2 import Timestamp

front_image_sources = ['frontleft_fisheye_image', 'frontright_fisheye_image', 'frontleft_depth', 'frontright_depth']
"""List of image sources for front image periodic query"""
side_image_sources = ['left_fisheye_image', 'right_fisheye_image', 'left_depth', 'right_depth']
"""List of image sources for side image periodic query"""
rear_image_sources = ['back_fisheye_image', 'back_depth']
"""List of image sources for rear image periodic query"""
hand_image_sources = ['hand_image', 'hand_depth', 'hand_color_image', 'hand_depth_in_hand_color_frame']
"""List of image sources for hand image periodic query"""


class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__("robot-state", client, logger,
                                           period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_state_async()
            callback_future.add_done_callback(self._callback)
            return callback_future

class AsyncMetrics(AsyncPeriodicQuery):
    """Class to get robot metrics at regular intervals.  get_robot_metrics_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncMetrics, self).__init__("robot-metrics", client, logger,
                                           period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_metrics_async()
            callback_future.add_done_callback(self._callback)
            return callback_future

class AsyncLease(AsyncPeriodicQuery):
    """Class to get lease state at regular intervals.  list_leases_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncLease, self).__init__("lease", client, logger,
                                           period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.list_leases_async()
            callback_future.add_done_callback(self._callback)
            return callback_future

class AsyncImageService(AsyncPeriodicQuery):
    """Class to get images at regular intervals.  get_image_from_sources_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback, image_requests):
        super(AsyncImageService, self).__init__("robot_image_service", client, logger,
                                           period_sec=1.0/max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_image_async(self._image_requests)
            callback_future.add_done_callback(self._callback)
            return callback_future

class AsyncIdle(AsyncPeriodicQuery):
    """Class to check if the robot is moving, and if not, command a stand with the set mobility parameters

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            spot_wrapper: A handle to the wrapper library
    """
    def __init__(self, client, logger, rate, spot_wrapper):
        super(AsyncIdle, self).__init__("idle", client, logger,
                                           period_sec=1.0/rate)

        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if self._spot_wrapper._last_stand_command != None:
            hold_position = False
            if self._spot_wrapper._hold_position_time is not None and time.time() < self._spot_wrapper._hold_position_time:
                hold_position = True
            try:
                response = self._client.robot_command_feedback(self._spot_wrapper._last_stand_command)
                self._spot_wrapper._is_sitting = False
                if (response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status ==
                        basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING):
                    if hold_position:
                        return
                    self._spot_wrapper._is_standing = True
                    self._spot_wrapper._last_stand_command = None
                    self._hold_position_time = None
                else:
                    self._spot_wrapper._is_standing = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_stand_command = None

        if self._spot_wrapper._last_sit_command != None:
            try:
                self._spot_wrapper._is_standing = False
                response = self._client.robot_command_feedback(self._spot_wrapper._last_sit_command)
                if (response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status ==
                        basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING):
                    self._spot_wrapper._is_sitting = True
                    self._spot_wrapper._last_sit_command = None
                else:
                    self._spot_wrapper._is_sitting = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_sit_command = None

        is_moving = False

        if self._spot_wrapper._last_velocity_command_time != None:
            if time.time() < self._spot_wrapper._last_velocity_command_time:
                is_moving = True
            else:
                self._spot_wrapper._last_velocity_command_time = None

        if self._spot_wrapper._last_trajectory_command != None:
            try:
                response = self._client.robot_command_feedback(self._spot_wrapper._last_trajectory_command)
                status = response.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
                # STATUS_AT_GOAL always means that the robot reached the goal. If the trajectory command did not
                # request precise positioning, then STATUS_NEAR_GOAL also counts as reaching the goal
                if status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL or \
                    (status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL and
                     not self._spot_wrapper._last_trajectory_command_precise):
                    self._spot_wrapper._at_goal = True
                    # Clear the command once at the goal
                    self._spot_wrapper._last_trajectory_command = None
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_GOING_TO_GOAL:
                    is_moving = True
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL:
                    is_moving = True
                    self._spot_wrapper._near_goal = True
                else:
                    self._spot_wrapper._last_trajectory_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_trajectory_command = None

        self._spot_wrapper._is_moving = is_moving

        if self._spot_wrapper.is_standing and not self._spot_wrapper.is_moving:
            self._spot_wrapper.stand(False)

class SpotWrapper():
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""
    def __init__(self, username, password, hostname, logger, estop_timeout=9.0, rates = {}, callbacks = {}):
        self._username = username
        self._password = password
        self._hostname = hostname
        self._logger = logger
        self._rates = rates
        self._callbacks = callbacks
        self._estop_timeout = estop_timeout
        self._keep_alive = True
        self._valid = True

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._is_standing = False
        self._is_sitting = True
        self._is_moving = False
        self._at_goal = False
        self._near_goal = False
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_trajectory_command_precise = None
        self._last_velocity_command_time = None
        self._hold_position_time = None
        self._get_world_tfrom_object = None

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        self._hand_image_requests = []
        for source in hand_image_sources:
            self._hand_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        try:
            self._sdk = create_standard_sdk('ros_spot')
        except Exception as e:
            self._logger.error("Error creating SDK object: %s", e)
            self._valid = False
            return

        self._robot = self._sdk.create_robot(self._hostname)

        try:
            self._robot.authenticate(self._username, self._password)
            self._robot.start_time_sync()
        except RpcError as err:
            self._logger.error("Failed to communicate with robot: %s", err)
            self._valid = False
            return

        if self._robot:
            # Clients
            try:
                self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
                self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
                self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
                self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
                self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
                self._lease_wallet = self._lease_client.lease_wallet
                self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
                self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
                # Start Arm wrapper
                self._arm = ArmWrapper(self._robot, self, self._logger)
            except Exception as e:
                self._logger.error("Unable to create client service: %s", e)
                self._valid = False
                return

            # Store the most recent knowledge of the state of the robot based on rpc calls.
            self._current_graph = None
            self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
            self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
            self._current_edge_snapshots = dict()  # maps id to edge snapshot
            self._current_annotation_name_to_wp_id = dict()

            # Async Tasks
            self._async_task_list = []
            self._robot_state_task = AsyncRobotState(self._robot_state_client, self._logger, max(0.0, self._rates.get("robot_state", 0.0)), self._callbacks.get("robot_state", lambda:None))
            self._robot_metrics_task = AsyncMetrics(self._robot_state_client, self._logger, max(0.0, self._rates.get("metrics", 0.0)), self._callbacks.get("metrics", lambda:None))
            self._lease_task = AsyncLease(self._lease_client, self._logger, max(0.0, self._rates.get("lease", 0.0)), self._callbacks.get("lease", lambda:None))
            self._front_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("front_image", 0.0)), self._callbacks.get("front_image", lambda:None), self._front_image_requests)
            self._side_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("side_image", 0.0)), self._callbacks.get("side_image", lambda:None), self._side_image_requests)
            self._rear_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("rear_image", 0.0)), self._callbacks.get("rear_image", lambda:None), self._rear_image_requests)
            self._hand_image_task = AsyncImageService(self._image_client, self._logger, max(0.0, self._rates.get("hand_image", 0.0)), self._callbacks.get("hand_image", lambda:None), self._hand_image_requests)
            self._idle_task = AsyncIdle(self._robot_command_client, self._logger, 10.0, self)

            self._estop_endpoint = None

            self._async_tasks = AsyncTasks(
                [self._robot_state_task, self._robot_metrics_task, self._lease_task, self._front_image_task, self._side_image_task, self._rear_image_task, self._hand_image_task, self._idle_task])

            self._robot_id = None
            self._lease = None

    @property
    def logger(self):
        """Return logger instance of the SpotWrapper"""
        return self._logger

    @property
    def is_valid(self):
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._valid

    @property
    def id(self):
        """Return robot's ID"""
        return self._robot_id

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def metrics(self):
        """Return latest proto from the _robot_metrics_task"""
        return self._robot_metrics_task.proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def front_images(self):
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self):
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self):
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto
    
    @property
    def hand_images(self):
        """Return latest proto from the _hand_image_task"""
        return self._hand_image_task.proto

    @property
    def is_standing(self):
        """Return boolean of standing state"""
        return self._is_standing

    @property
    def is_sitting(self):
        """Return boolean of standing state"""
        return self._is_sitting

    @property
    def is_moving(self):
        """Return boolean of walking state"""
        return self._is_moving

    @property
    def near_goal(self):
        return self._near_goal

    @property
    def at_goal(self):
        return self._at_goal

    @property
    def time_skew(self):
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def resetMobilityParams(self):
        """
        Resets the mobility parameters used for motion commands to the default values provided by the bosdyn api.
        Returns:
        """
        self._mobility_params = RobotCommandBuilder.mobility_params()

    def robotToLocalTime(self, timestamp):
        """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """

        rtime = Timestamp()

        rtime.seconds = timestamp.seconds - self.time_skew.seconds
        rtime.nanos = timestamp.nanos - self.time_skew.nanos
        if rtime.nanos < 0:
            rtime.nanos = rtime.nanos + 1000000000
            rtime.seconds = rtime.seconds - 1

        # Workaround for timestamps being incomplete
        if rtime.seconds < 0:
            rtime.seconds = 0
            rtime.nanos = 0

        return rtime

    def claim(self):
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self._robot_id = self._robot.get_id()
            self.getLease()
            self.resetEStop()
            return True, "Success"
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False, str(err)

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        try:
            self._async_tasks.update()
        except Exception as e:
            print(f"Update tasks failed with error: {str(e)}")

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', self._estop_timeout)
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe=True):
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_keepalive.stop()
            else:
                self._estop_keepalive.settle_then_cut()

            return True, "Success"
        except:
            return False, "Error"

    def disengageEStop(self):
        """Disengages the E-Stop"""
        try:
            self._estop_keepalive.allow()
            return True, "Success"
        except:
            return False, "Error"


    def releaseEStop(self):
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self):
        """Get a lease for the robot and keep the lease alive automatically."""
        self._lease = self._lease_client.acquire()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def releaseLease(self):
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self):
        """Return the lease on the body and the eStop handle."""
        try:
            self.releaseLease()
            self.releaseEStop()
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def disconnect(self):
        """Release control of robot as gracefully as posssible."""
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.releaseLease()
        self.releaseEStop()

    def _robot_command(self, command_proto, end_time_secs=None, timesync_endpoint=None):
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
            timesync_endpoint: (optional) Time sync endpoint
        """
        try:
            id = self._robot_command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs, timesync_endpoint=timesync_endpoint)
            return True, "Success", id
        except Exception as e:
            return False, str(e), None

    def stop(self):
        """Stop the robot's motion."""
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    def self_right(self):
        """Have the robot self-right itself."""
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self):
        """Stop the robot's motion and sit down if able."""
        self.arm_stow() # Arm needs to be stowed before sitting!!
        time.sleep(0.5)
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    def stand(self, monitor_command=True):
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up."""
        response = self._robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    def safe_power_off(self):
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        response = self._robot_command(RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def clear_behavior_fault(self, id):
        """Clear the behavior fault defined by id."""
        try:
            rid = self._robot_command_client.clear_behavior_fault(behavior_fault_id=id, lease=None)
            return True, "Success", rid
        except Exception as e:
            return False, str(e), None

    def power_on(self):
        """Enble the motor power if e-stop is enabled."""
        try:
            power.power_on(self._power_client)
            return True, "Success"
        except Exception as e:
            return False, str(e)

    def set_mobility_params(self, mobility_params):
        """Set Params for mobility and movement

        Args:
            mobility_params: spot.MobilityParams, params for spot mobility commands.
        """
        self._mobility_params = mobility_params

    def get_mobility_params(self):
        """Get mobility params
        """
        return self._mobility_params

    def velocity_cmd(self, v_x, v_y, v_rot, cmd_duration=0.125):
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        end_time=time.time() + cmd_duration
        response = self._robot_command(RobotCommandBuilder.synchro_velocity_command(
                                      v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params),
                                      end_time_secs=end_time, timesync_endpoint=self._robot.time_sync.endpoint)
        self._last_velocity_command_time = end_time
        return response[0], response[1]

    def trajectory_cmd(self, goal_x, goal_y, goal_heading, cmd_duration, frame_name='odom', precise_position=False):
        """Send a trajectory motion command to the robot.

        Args:
            goal_x: Position X coordinate in meters
            goal_y: Position Y coordinate in meters
            goal_heading: Pose heading in radians
            cmd_duration: Time-to-live for the command in seconds.
            frame_name: frame_name to be used to calc the target position. 'odom' or 'vision'
            precise_position: if set to false, the status STATUS_NEAR_GOAL and STATUS_AT_GOAL will be equivalent. If
            true, the robot must complete its final positioning before it will be considered to have successfully
            reached the goal.
        """
        self._at_goal = False
        self._near_goal = False
        self._last_trajectory_command_precise = precise_position
        self._logger.info("got command duration of {}".format(cmd_duration))
        end_time=time.time() + cmd_duration
        if frame_name == 'vision':
            vision_tform_body = frame_helpers.get_vision_tform_body(
                    self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
            body_tform_goal = math_helpers.SE3Pose(x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading))
            vision_tform_goal = vision_tform_body * body_tform_goal
            response = self._robot_command(
                            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                                goal_x=vision_tform_goal.x,
                                goal_y=vision_tform_goal.y,
                                goal_heading=vision_tform_goal.rot.to_yaw(),
                                frame_name=frame_helpers.VISION_FRAME_NAME,
                                params=self._mobility_params),
                            end_time_secs=end_time
                            )
        elif frame_name == 'odom':
            odom_tform_body = frame_helpers.get_odom_tform_body(
                    self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
            body_tform_goal = math_helpers.SE3Pose(x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading))
            odom_tform_goal = odom_tform_body * body_tform_goal
            response = self._robot_command(
                            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                                goal_x=odom_tform_goal.x,
                                goal_y=odom_tform_goal.y,
                                goal_heading=odom_tform_goal.rot.to_yaw(),
                                frame_name=frame_helpers.ODOM_FRAME_NAME,
                                params=self._mobility_params),
                            end_time_secs=end_time
                            )
        else:
            raise ValueError('frame_name must be \'vision\' or \'odom\'')
        if response[0]:
            self._last_trajectory_command = response[2]
        return response[0], response[1]

    def list_graph(self, upload_path):
        """List waypoint ids of garph_nav
        Args:
          upload_path : Path to the root directory of the map.
        """
        ids, eds = self._list_graph_waypoint_and_edge_ids()
        # skip waypoint_ for v2.2.1, skip waypiont for < v2.2
        return [v for k, v in sorted(ids.items(), key=lambda id : int(id[0].replace('waypoint_','')))]

    def navigate_to(self, upload_path,
                    navigate_to,
                    initial_localization_fiducial=True,
                    initial_localization_waypoint=None):
        """ navigate with graph nav.

        Args:
           upload_path : Path to the root directory of the map.
           navigate_to : Waypont id string for where to goal
           initial_localization_fiducial : Tells the initializer whether to use fiducials
           initial_localization_waypoint : Waypoint id string of current robot position (optional)
        """
        # Filepath for uploading a saved graph's and snapshots too.
        if upload_path[-1] == "/":
            upload_filepath = upload_path[:-1]
        else:
            upload_filepath = upload_path

        # Boolean indicating the robot's power state.
        power_state = self._robot_state_client.get_robot_state().power_state
        self._started_powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        self._powered_on = self._started_powered_on

        # FIX ME somehow,,,, if the robot is stand, need to sit the robot before starting garph nav
        if self.is_standing and not self.is_moving:
            self.sit()

        # TODO verify estop  / claim / power_on
        self._clear_graph()
        self._upload_graph_and_snapshots(upload_filepath)
        if initial_localization_fiducial:
            self._set_initial_localization_fiducial()
        if initial_localization_waypoint:
            self._set_initial_localization_waypoint([initial_localization_waypoint])
        self._list_graph_waypoint_and_edge_ids()
        self._get_localization_state()
        resp = self._navigate_to([navigate_to])

        return resp
    
    def spot_position(self, euler_x, euler_y, euler_z, timeout_sec=0.0, monitor_command=True):
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up.
            This is for Spot to hold a specific position for a certain amount of time"""
        mobility_param = RobotCommandBuilder().mobility_params(footprint_R_body=EulerZXY(euler_z, euler_x, euler_y))
        response = self._robot_command(RobotCommandBuilder.synchro_stand_command(params=mobility_param), end_time_secs=timeout_sec)
        self._hold_position_time = time.time() + timeout_sec
        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    # Arm ############################################
    def ensure_arm_power_and_stand(self):
        if not self._robot.has_arm():
            return False, "Spot with an arm is required for this service"
        
        try:
            self._logger.info("Spot is powering on within the timeout of 20 secs")
            self._robot.power_on(timeout_sec=20)
            assert self._robot.is_powered_on(), "Spot failed to power on"
            self._logger.info("Spot is powered on")
        except Exception as e:
            return False, "Exception occured while Spot was trying to power on or stand"

        if not self._is_standing:
            robot_command.blocking_stand(command_client=self._robot_command_client, timeout_sec=10.0)
            self._logger.info("Spot is standing")
        else:
            self._logger.info("Spot is already standing")

        return True, "Spot has an arm, is powered on, and standing"

    def arm_stow(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Stow Arm
                stow = RobotCommandBuilder.arm_stow_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(stow)
                self._logger.info("Command stow issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while trying to stow"

        return True, "Stow arm success"

    def arm_unstow(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:                
                # Unstow Arm
                unstow = RobotCommandBuilder.arm_ready_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(unstow)
                self._logger.info("Command unstow issued")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while trying to unstow"

        return True, "Unstow arm success"

    def arm_carry(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Get arm in carry mode
                carry = RobotCommandBuilder.arm_carry_command()
                self._robot_command_client.robot_command(carry)
                self._logger.info("Command carry issued")
                time.sleep(1.0)
        except Exception as e:
            return False, "Exception occured while carry mode was issued"

        return True, "Carry mode success"
    
    
    def make_arm_trajectory_command(self, arm_joint_trajectory):
        """ Helper function to create a RobotCommand from an ArmJointTrajectory. 
            Copy from 'spot-sdk/python/examples/arm_joint_move/arm_joint_move.py' """

        joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_trajectory)
        arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
        sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
        arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
        return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)

    def arm_joint_move(self, joint_targets):
        """Move the Arm by giving an angle for each joints of the arm"""
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Joint1: 0.0 arm points to the front. RANGE: 0.0 -> 5.75959 (positive: turn left, negative: turn right)
                # Joint2: 0.0 arm points to the front. RANGE: 0.0 -> 3.66519)
                # Joint3: 0.0 arm straight. RANGE: 0.0 -> 3.1415
                # Joint4: 0.0 middle position. RANGE: -2.79253 -> 2.79253
                # Joint5: 0.0 gripper points to the front. RANGE: -1.8326 -> 1.8326
                # Joint6: 0.0 Moving finger on top of stationary finger. RANGE: -2.87979 -> 2.87979)

                trajectory_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
                    joint_targets[0], joint_targets[1], joint_targets[2],
                    joint_targets[3], joint_targets[4], joint_targets[5])
                arm_joint_trajectory = arm_command_pb2.ArmJointTrajectory(points = [trajectory_point])
                arm_command = self.make_arm_trajectory_command(arm_joint_trajectory)

                # Send the request
                cmd_id = self._robot_command_client.robot_command(arm_command)

                # Query for feedback to determine how long it will take
                feedback_resp = self._robot_command_client.robot_command_feedback(cmd_id)
                joint_move_feedback = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
                time_to_goal : Duration = joint_move_feedback.time_to_goal
                time_to_goal_in_seconds: float = time_to_goal.seconds + (float(time_to_goal.nanos) / float(10**9))
                time.sleep(time_to_goal_in_seconds)
                return True, "Spot Arm moved successfully"
        
        except Exception as e:
            return False, "Exception occured during arm movement: " + str(e)

    def force_trajectory(self, forces_torques):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Unstow arm
                unstow = RobotCommandBuilder.arm_ready_command()

                # Send command via the RobotCommandClient
                self._robot_command_client.robot_command(unstow)

                self._logger.info("Unstow command issued.")
                time.sleep(2.0)

                # Demonstrate an example force trajectory by ramping up and down a vertical force over
                # 10 seconds

                f_x0 = forces_torques[0]  # Newtons
                f_y0 = forces_torques[1]
                f_z0 = forces_torques[2]

                f_x1 = forces_torques[3]  # Newtons
                f_y1 = forces_torques[4]
                f_z1 = forces_torques[5]  # -10 push down

                # We won't have any rotational torques
                torque_x = forces_torques[6]
                torque_y = forces_torques[7]
                torque_z = forces_torques[8]

                # Duration in seconds.
                traj_duration = 5

                # First point of trajectory
                force_vector0 = geometry_pb2.Vec3(x=f_x0, y=f_y0, z=f_z0)
                torque_vector0 = geometry_pb2.Vec3(x=torque_x, y=torque_y, z=torque_z)
                
                wrench0 = geometry_pb2.Wrench(force=force_vector0, torque=torque_vector0)
                t0 = seconds_to_duration(0)
                traj_point0 = trajectory_pb2.WrenchTrajectoryPoint(wrench=wrench0,
                                                                time_since_reference=t0)
                
                # Second point on the trajectory
                force_vector1 = geometry_pb2.Vec3(x=f_x1, y=f_y1, z=f_z1)
                torque_vector1 = geometry_pb2.Vec3(x=torque_x, y=torque_y, z=torque_z)

                wrench1 = geometry_pb2.Wrench(force=force_vector1, torque=torque_vector1)
                t1 = seconds_to_duration(traj_duration)
                traj_point1 = trajectory_pb2.WrenchTrajectoryPoint(wrench=wrench1,
                                                                time_since_reference=t1)

                # Build the trajectory
                self._logger.info("Building the trajectory")
                trajectory = trajectory_pb2.WrenchTrajectory(points=[traj_point0, traj_point1])
                
                # Build the trajectory request, putting all axes into force mode
                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=ODOM_FRAME_NAME, wrench_trajectory_in_task=trajectory,
                    x_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    y_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    z_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rx_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    ry_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rz_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE)
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command)
                synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
                    arm_command=arm_command)
                robot_command = robot_command_pb2.RobotCommand(
                    synchronized_command=synchronized_command)

                # Send the request
                self._robot_command_client.robot_command(robot_command)
                self._logger.info('Force trajectory command sent')

                time.sleep(10.0)

        except Exception as e:
            return False, "Exception occured during arm movement" + str(e)
        
    def gripper_open(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Open gripper
                command = RobotCommandBuilder.claw_gripper_open_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open sent")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Open gripper success"

    def gripper_close(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Close gripper
                command = RobotCommandBuilder.claw_gripper_close_command()

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper close sent")
                time.sleep(2.0)
                
        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Closed gripper successfully"
    
    
    def gripper_angle_open(self, gripper_ang):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Open gripper at an angle
                command = RobotCommandBuilder.claw_gripper_open_angle_command(gripper_q)

                # Command issue with RobotCommandClient
                self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open angle sent")
                time.sleep(2.0)

        except Exception as e:
            return False, "Exception occured while gripper was moving"

        return True, "Opened gripper successfully"
    
    def body_follow_arm(self):
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Move the arm to a spot in front of the robot, and command the body to follow the hand.
                # Build a position to move the arm to (in meters, relative to the body frame origin.)
                x = 1.25
                y = 0
                z = 0.25
                hand_pos_rt_body = geometry_pb2.Vec3(x=x, y=y, z=z)

                # Rotation as a quaternion.
                qw = 1
                qx = 0
                qy = 0
                qz = 0
                body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

                # Build the SE(3) pose of the desired hand position in the moving body frame.
                body_T_hand = geometry_pb2.SE3Pose(position=hand_pos_rt_body, rotation=body_Q_hand)

                # Transform the desired from the moving body frame to the odom frame.
                robot_state = self._robot_state_client.get_robot_state()

                odom_T_body = frame_helpers.get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                            ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
                odom_T_hand = odom_T_body * math_helpers.SE3Pose.from_obj(body_T_hand)

                # duration in seconds
                seconds = 5

                # Create the arm command.
                arm_command = RobotCommandBuilder.arm_pose_command(
                    odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
                    odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)
                self._logger.info("Create arm command")

                # Tell the robot's body to follow the arm
                follow_arm_command = RobotCommandBuilder.follow_arm_command()
                
                command = self._robot_command(RobotCommandBuilder.build_synchro_command(follow_arm_command, arm_command))
                self._logger.info("After building command")

                # Send the request
                self._robot_command_client.robot_command(command)
                robot.logger.info('Moving arm to position.')

                time.sleep(6.0)

        except Exception as e:
            return False, "Exception occured while arm was moving"

        return True, "Moved arm successfully"

    def arm_pose_cmd(self, pose_x, pose_y, pose_z, q_x, q_y, q_z, q_w):
        """Give a position for the arm to achieve"""
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                self.gripper_open()

                # Position in meters to move arm (aligned to body frame)
                position = geometry_pb2.Vec3(x=pose_x, y=pose_y, z=pose_z)
                # Rotation as a quaternion
                rotation = geometry_pb2.Quaternion(w=q_w, x=q_x, y=q_y, z=q_z)

                flat_body_T_hand = geometry_pb2.SE3Pose(position=position,
                                                        rotation=rotation)
                
                robot_state = self._robot_state_client.get_robot_state()
                odom_T_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                                ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
                odom_T_hand = odom_T_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)

                end_time = 2
                arm_command = RobotCommandBuilder.arm_pose_command(
                    odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w,
                    odom_T_hand.rot.x, odom_T_hand.rot.y, odom_T_hand.rot.z,
                    ODOM_FRAME_NAME, end_time
                )

                command = RobotCommandBuilder.build_synchro_command(arm_command)
                cmd_id = self._robot_command(command)
                
        except Exception as e:
            return False, "Exception occured while arm was moving"

        return True

    
    def hand_pose(self, pose_points):
        """By giving a position and orientation in space to the gripper, the arm will move to achieve it """
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Move the arm to a spot in front of the robot given a pose for the gripper.
                # Build a position to move the arm to (in meters, relative to the body frame origin.)
                x = pose_points[0] #1.45
                y = pose_points[1] #0.2
                z = pose_points[2] #0.85
                position = geometry_pb2.Vec3(x=x, y=y, z=z)

                # Rotation as a quaternion.
                qw = pose_points[3]
                qx = pose_points[4]
                qy = pose_points[5]
                qz = pose_points[6]
                rotation = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

                seconds = 5.0
                duration = seconds_to_duration(seconds)

                # Build the SE(3) pose of the desired hand position in the moving body frame.
                hand_pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
                hand_pose_traj_point = trajectory_pb2.SE3TrajectoryPoint(pose=hand_pose, time_since_reference=duration)                
                hand_trajectory = trajectory_pb2.SE3Trajectory(points=[hand_pose_traj_point])
                
                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=ODOM_FRAME_NAME, pose_trajectory_in_task=hand_trajectory)
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command)
                synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
                    arm_command=arm_command)
                                
                #robot_command = self._robot_command(RobotCommandBuilder.build_synchro_command(synchronized_command))
                robot_command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

                command = self._robot_command(RobotCommandBuilder.build_synchro_command(robot_command))

                self._logger.info("After building command")

                # Send the request
                self._robot_command_client.robot_command(command)
                self._logger.info('Moving arm to position.')

                time.sleep(6.0)

        except Exception as e:
            return False, "Exception occured while arm was moving"

        return True, "Moved arm successfully"


    def grasp_object_image(self, y_min, x_min, y_max, x_max):
        print("Starting grasp object in image")
        try:
            success, msg = self.ensure_arm_power_and_stand()
            self.gripper_open()
            self.arm_unstow()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                manipulation_api_client = self._robot.ensure_client(ManipulationApiClient.default_service_name)

                visual_image = self._image_client.get_image_from_sources(["hand_image"])[0]
                print("visual_image being used: " + str(visual_image))
                
                # Find centre pixel of the rotated bounding box in the original frame
                center_px_x = math.fabs(x_max - x_min) / 2.0 + x_min
                center_px_y = math.fabs(y_max - y_min) / 2.0 + y_min
                # Request Pick up on that pixel
                pick_vec = geometry_pb2.Vec2(x=center_px_x, y=center_px_y)
                grasp = manipulation_api_pb2.PickObjectInImage(
                    pixel_xy=pick_vec,
                    transforms_snapshot_for_camera=visual_image.shot.transforms_snapshot,
                    frame_name_image_sensor=visual_image.shot.frame_name_image_sensor,
                    camera_model=visual_image.source.pinhole,
                    walk_gaze_mode=False)

                print("grasp is " + str(grasp))

                # We can specify where in the gripper we want to grasp. About halfway is generally good for
                # small objects like this. For a bigger object like a shoe, 0 is better (use the entire
                # gripper)
                grasp.grasp_params.grasp_palm_to_fingertip = 0.6

                # Tell the grasping system that we want a top-down grasp.

                # Add a constraint that requests that the x-axis of the gripper is pointing in the
                # negative-z direction in the vision frame.

                # The axis on the gripper is the x-axis.
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

                # The axis in the vision frame is the negative z-axis
                axis_to_align_with_ewrt_vision = geometry_pb2.Vec3(x=0, y=0, z=-1)

                # Add the vector constraint to our proto.
                constraint = grasp.grasp_params.allowable_orientation.add()
                constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
                    axis_on_gripper_ewrt_gripper)
                constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
                    axis_to_align_with_ewrt_vision)

                # We'll take anything within about 15 degrees for top-down or horizontal grasps.
                constraint.vector_alignment_with_tolerance.threshold_radians = 0.25

                # Specify the frame we're using.
                grasp.grasp_params.grasp_params_frame_name = frame_helpers.VISION_FRAME_NAME

                # Build the proto
                grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                    pick_object_in_image=grasp)

                # Send the request
                print('Sending grasp request...')
                cmd_response = manipulation_api_client.manipulation_api_command(
                    manipulation_api_request=grasp_request)

                # Wait for the grasp to finish
                grasp_done = False
                failed = False
                time_start = time.time()
                timeout_sec = 8
                end_time = time_start + timeout_sec
                now = time.time()

                while now < end_time or not grasp_done:
                    feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                        manipulation_cmd_id=cmd_response.manipulation_cmd_id)

                    # Send a request for feedback
                    response = manipulation_api_client.manipulation_api_feedback_command(
                        manipulation_api_feedback_request=feedback_request)

                    current_state = response.current_state
                    current_time = time.time()
                    print('Current state ({time:.1f} sec): {state}'.format(
                        time=current_time - time_start,
                        state=manipulation_api_pb2.ManipulationFeedbackState.Name(
                            current_state)),
                        end='                \r')
                    #sys.stdout.flush()

                    failed_states = [manipulation_api_pb2.MANIP_STATE_GRASP_FAILED,
                                    manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION,
                                    manipulation_api_pb2.MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP,
                                    manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE]

                    failed = current_state in failed_states
                    grasp_done = current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or failed

                    time.sleep(0.1)
                    now = time.time()
                    
        finally:
            print("grasp done")

    
    def walk_to_object_image(self, object_point):
        print("In wrapper")
        print(object_point)

        try:
            success, msg = self.ensure_arm_power_and_stand()
            self.gripper_open()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                walk_vec = geometry_pb2.Vec2(x=object_point[0], y=object_point[1])

                manipulation_api_client = self._robot.ensure_client(ManipulationApiClient.default_service_name)
                print("manipulation_api_client")

                data = self.front_images
                image_used = data[1]

                offset_distance = None

                # Build proto
                walk_to = manipulation_api_pb2.WalkToObjectInImage(
                    pixel_xy=walk_vec, transforms_snapshot_for_camera=image_used.shot.transforms_snapshot,
                    frame_name_image_sensor=image_used.shot.frame_name_image_sensor,
                    camera_model=image_used.source.pinhole, offset_distance=offset_distance)

                print("Built proto")

                # Ask Spot to pick up the object
                walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
                    walk_to_object_in_image=walk_to)

                print("Walk to request")

                # Send the request
                cmd_response = manipulation_api_client.manipulation_api_command( 
                    manipulation_api_request=walk_to_request)

                print("Sent request")

                # Get feedback from robot
                while True:
                    time.sleep(0.25)
                    feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                        manipulation_cmd_id=cmd_response.manipulation_cmd_id)

                    print("Feedback request")

                    # Send the request
                    response = manipulation_api_client.manipulation_api_feedback_command(
                        manipulation_api_feedback_request=feedback_request)

                    print("Feedback response")

                    print('Current state: ', manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

                    if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
                        break

                self._logger.info('Finished')
                time.sleep(4.0)

        finally:
            print("done")

    # BEGIN: METHODS FOR FETCH ############################################################################
    def block_for_trajectory_cmd(self, cmd_id, timeout_sec=None):
        """Helper that blocks until a trajectory command reaches STATUS_AT_GOAL or a timeout is
            exceeded.
        Args:
            cmd_id: command ID returned by the robot when the trajectory command was sent
            timeout_sec: optional number of seconds after which we'll return no matter what the
                            robot's state is.
        Return values:
            True if reaches STATUS_AT_GOAL, False otherwise.
        """
        start_time = time.time()
        verbose = True

        if timeout_sec is not None:
            end_time = start_time + timeout_sec
            now = time.time()

        while timeout_sec is None or now < end_time:
            feedback_resp = self._robot_command_client.robot_command_feedback(cmd_id)
            current_state = feedback_resp.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
            print('current_state ' + str(current_state))

            if verbose:
                current_state_str = basic_command_pb2.SE2TrajectoryCommand.Feedback.Status.Name(current_state)

                current_time = time.time()
                print('Walking: ({time:.1f} sec): {state}'.format(
                    time=current_time - start_time, state=current_state_str),
                    end='                \r')

            if current_state == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL:
                return True

            time.sleep(0.3)
            now = time.time()

        if verbose:
            print('block_for_trajectory_cmd: timeout exceeded.')

        return False
    
    
    def _get_heading(self, xhat):
        """ Get heading direction"""
        zhat = [0.0, 0.0, 1.0]
        yhat = np.cross(zhat, xhat)
        mat = np.array([xhat, yhat, zhat]).transpose()
        return math_helpers.Quat.from_matrix(mat).to_yaw()
    
    
    def get_go_to(self, world_tform_object, mobility_params, dist_margin=0.5):
        """Gets trajectory command to a goal location

        Args:
            world_tform_object (SE3Pose): Transform from vision frame to target object
            robot_state (RobotState): Current robot state
            mobility_params (MobilityParams): Mobility parameters
            dist_margin (float): Distance margin to target
        """
        robot_state = self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        vo_tform_robot = frame_helpers.get_vision_tform_body(robot_state)

        delta_ewrt_vo = np.array(
            [world_tform_object.x - vo_tform_robot.x, world_tform_object.y - vo_tform_robot.y, 0])
        norm = np.linalg.norm(delta_ewrt_vo)
        if norm == 0:
            return None
        delta_ewrt_vo_norm = delta_ewrt_vo / norm
        heading = self._get_heading(delta_ewrt_vo_norm)
        vo_tform_goal = np.array([
            world_tform_object.x - delta_ewrt_vo_norm[0] * dist_margin,
            world_tform_object.y - delta_ewrt_vo_norm[1] * dist_margin
        ])

        position = geometry_pb2.Vec2(x=vo_tform_goal[0], y=vo_tform_goal[1])
        goal_se2 = geometry_pb2.SE2Pose(position=position, angle=heading)
        tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_command(
                                goal_se2=goal_se2,
                                frame_name=frame_helpers.VISION_FRAME_NAME,
                                params=mobility_params)

        """
        There are many move command posiible in the SDK apart from synchro_se2_trajectory_command.
        Below are two other examples that could be used instead:

        tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                                goal_x=vo_tform_goal[0],
                                goal_y=vo_tform_goal[1],
                                goal_heading=heading,
                                frame_name=frame_helpers.VISION_FRAME_NAME,
                                params=mobility_params)

        tag_cmd = RobotCommandBuilder.trajectory_command(goal_x=vo_tform_goal[0],
                                                        goal_y=vo_tform_goal[1], goal_heading=heading,
                                                        frame_name=frame_helpers.VISION_FRAME_NAME,
                                                        params=mobility_params)
        """
        return tag_cmd
    
    
    
    def get_distance_to_closest_object_depth(self, x_min, x_max, y_min, y_max, depth_scale, raw_depth_image,
                                         histogram_bin_size=0.20, minimum_number_of_points=100,
                                         max_distance=4.0):
        """Make a histogram of distances to points in the cloud and take the closest distance with
        enough points.

        Args:
            origin (tuple): Origin to rotate the point around
            x_min (int): minimum x coordinate (column) of object to find
            x_max (int): maximum x coordinate (column) of object to find
            y_min (int): minimum y coordinate (row) of object to find
            y_max (int): maximum y coordinate (row) of object to find
            depth_scale (float): depth scale of the image to convert from sensor value to meters
            raw_depth_image (np.array): matrix of depth pixels
            histogram_bin_size (float): size of each bin of distances
            minimum_number_of_points (int): minimum number of points before returning depth
            max_distance (float): maximum distance to object in meters

        Default:
            histogram_bin_size=0.20
            minimum_number_of_points=100
            max_distance=4.0
        """'frontright_depth_in_visual_frame'
        print("In get_distance_to_closest_object_depth")
        num_bins = math.ceil(max_distance / histogram_bin_size)
        depths = []

        for row in range(y_min, y_max):
            for col in range(x_min, x_max):
                raw_depth = raw_depth_image[row][col]
                if raw_depth != 0 and raw_depth is not None:
                    depth = raw_depth / depth_scale
                    depths.append(depth)

        hist, hist_edges = np.histogram(depths, bins=num_bins, range=(0, max_distance))

        edges_zipped = zip(hist_edges[:-1], hist_edges[1:])
        # Iterate over the histogram and return the first distance with enough points.
        for entry, edges in zip(hist, edges_zipped):
            if entry > minimum_number_of_points:
                return statistics.mean([d for d in depths if d > edges[0] and d > edges[1]])

        return max_distance

    def rotate_about_origin_degrees(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        Args:
            origin (tuple): Origin to roFutureWrappertate the point around
            point (tuple): Point to rotate
            angle (float): Angle in degrees
        """
        return self.rotate_about_origin(origin, point, math.radians(angle))

    def rotate_about_origin(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        Args:
            origin (tuple): Origin to rotate the point around
            point (tuple): Point to rotate
            angle (float): Angle in radians
        """
        orig_x, orig_y = origin
        pnt_x, pnt_y = point

        ret_x = orig_x + math.cos(angle) * (pnt_x - orig_x) - math.sin(angle) * (pnt_y - orig_y)
        ret_y = orig_y + math.sin(angle) * (pnt_x - orig_x) + math.cos(angle) * (pnt_y - orig_y)
        return int(ret_x), int(ret_y)
    
    
    def get_object_position(self, world_tfrom_cam, visual_image, depth_image, y_min, y_max, x_min, x_max):
        """
        Function to get the object to be grasped position uing visual and depth data
        world_tform_cam (SE3Pose): SE3 transform from world to camera frame
        visual_image (ImageResponse): From a visual camera
        depth_image (ImageResponse): From a depth camera corresponding to the visual_image
        """
        if visual_image is None or depth_image is None:
            print("Fail: Visual or depth image is none")
            return

        print("world_tfrom_cam is " + str(world_tfrom_cam) + " type " + str(type(world_tfrom_cam)))

        # Get width and height of bounding box
        height = y_max - y_min
        width = x_max - x_min
        print("height of bouding box is " + str(height) + " width is " + str(width))

        # Rotate bounding box back to original frame
        rotation_angle = -102 # FOR FRONTRIGHT_FISHEYE_IMAGE
        points = [(y_min, x_min), (height, x_min), (height, width), (y_min, width)]
        origin = (visual_image.shot.image.cols / 2, visual_image.shot.image.rows / 2)
        points_rot = [self.rotate_about_origin_degrees(origin, point, rotation_angle) for point in points]

        # Get bounding box corners in right frame
        ymin = max(0, min([point[1] for point in points_rot]))
        xmin = max(0, min([point[0] for point in points_rot]))
        ymax = min(visual_image.shot.image.rows, max([point[1] for point in points_rot]))
        xmax = min(visual_image.shot.image.cols, max([point[0] for point in points_rot]))

        print("new points: ymin, " + str(ymin) + " xmin " + str(xmin) + " ymax " + str(ymax) + " xmax " + str(xmax))
        
        # Check that the bounding box is valid
        if (xmin < 0 or ymin < 0 or xmax > visual_image.shot.image.cols or
                ymax > visual_image.shot.image.rows):
            print(f'Bounding box is invalid: ({xmin}, {ymin}) | ({xmax}, {ymax})')
            print(f'Bounds: ({visual_image.shot.image.cols}, {visual_image.shot.image.rows})')
            return
        
        try:
            if depth_image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                dtype = np.uint16
            else:
                dtype = np.uint8
            img = np.fromstring(depth_image.shot.image.data, dtype = dtype)
            if depth_image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                img = img.reshape(depth_image.shot.image.rows, depth_image.shot.image.cols)
            else:
                img = cv2.imdecode(img, -1)
            
            depth_image_pixels = img

            # Get depth data from the region in the bounding box
            depth = self.get_distance_to_closest_object_depth(xmin, xmax, ymin, ymax,
                                                     depth_image.source.depth_scale,
                                                     depth_image_pixels)

            print("depth is " + str(depth))

            if depth >= 4.0:
                print("Not enough depth data")
                return False
            
            # Calculate the transform to the center point of the object using camera intrinsics
            # and depth calculated earlier in the function
            focal_x = depth_image.source.pinhole.intrinsics.focal_length.x
            principal_x = depth_image.source.pinhole.intrinsics.principal_point.x

            focal_y = depth_image.source.pinhole.intrinsics.focal_length.y
            principal_y = depth_image.source.pinhole.intrinsics.principal_point.y

            center_x = round((xmax - xmin) / 2.0 + xmin)
            center_y = round((ymax - ymin) / 2.0 + ymin)

            tform_x = depth * (center_x - principal_x) / focal_x
            tform_y = depth * (center_y - principal_y) / focal_y
            tform_z = depth
            obj_tform_camera = math_helpers.SE3Pose(tform_x, tform_y, tform_z, math_helpers.Quat())

            return world_tfrom_cam * obj_tform_camera
        except Exception as exc:  # pylint: disable=broad-except
            print(f'Error getting object position: {exc}')
            return

    def get_world_tfrom_object(self):
        """Helper to return world_tfrom_object detected"""
        if self._get_world_tfrom_object is not None:
            return self._get_world_tfrom_object
    
    def fetch_obj(self, center_px, y_min, x_min, y_max, x_max):
        """Fetch function to get the distance from the object to be grasped to the robot, and walk to it"""
        try:
            success, msg = self.ensure_arm_power_and_stand()
            self.gripper_open()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                data = self.front_images
                
                image_used = data[1]   # image
                print("image_used type " + str(type(image_used)))
                # Needing this depth ImageResponse so size is the same than visual frame
                depth_used = self._image_client.get_image_from_sources(["frontright_depth_in_visual_frame"])[0]
                print("depth_used type " + str(type(depth_used)))

                # Get a tf from vision to sensor/camera
                vision_tfrom_cam = frame_helpers.get_a_tform_b(
                    image_used.shot.transforms_snapshot,
                    frame_helpers.VISION_FRAME_NAME,
                    image_used.shot.frame_name_image_sensor
                )

                if vision_tfrom_cam is None:
                    print("vision_tfrom_cam is NONE")
                else:
                    print("vision_tfrom_cam is " + str(vision_tfrom_cam) + " and type is " + str(type(vision_tfrom_cam)))

                    # Find transform to object using depth sensor
                    world_tfrom_object = self.get_object_position(
                        world_tfrom_cam=vision_tfrom_cam,
                        visual_image=image_used,
                        depth_image=depth_used,
                        y_min=y_min, y_max=y_max, x_min=x_min, x_max=x_max
                    )

                    if world_tfrom_object is False:
                        return False

                    # Create a TF from the object detected to the robot
                    self._get_world_tfrom_object = world_tfrom_object
                    print("world_tfrom_object " + str(world_tfrom_object) + " type " + str(type(world_tfrom_object)))

                    # Create move command
                    params_set = self._mobility_params

                    # Get trajectory tag command
                    move_cmd = self.get_go_to(world_tform_object=world_tfrom_object,
                                            mobility_params=params_set,
                                            dist_margin=0.5)
                    print("move_cmd " + str(move_cmd) + " type is " + str(type(move_cmd)))

                    end_time = time.time() + 15.0
                    if move_cmd is not None:
                        # Move to the detected object
                        cmd_id = self._robot_command_client.robot_command(
                            command=move_cmd, end_time_secs=end_time
                            )
                        self.block_for_trajectory_cmd(cmd_id=cmd_id, timeout_sec=15)
                        
        except Exception as exc:
            print(f'Error while doing fetch: {exc}')
            return
    # END: METHODS FOR FETCH ############################################################################
            

    ###################################################################

    ## copy from spot-sdk/python/examples/graph_nav_command_line/graph_nav_command_line.py
    def _get_localization_state(self, *args):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state()
        self._logger.info('Got localization: \n%s' % str(state.localization))
        odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot)
        self._logger.info('Got robot state in kinematic odometry frame: \n%s' % str(odom_tform_body))

    def _set_initial_localization_fiducial(self, *args):
        """Trigger localization when near a fiducial."""
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)

    def _set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            self._logger.error("No waypoint specified to initialize to.")
            return
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0], self._current_graph, self._current_annotation_name_to_wp_id, self._logger)
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
            return

        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance = 0.2,
            max_yaw = 20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body)

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Empty graph.")
            return
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id

        # Update and print waypoints and edges
        self._current_annotation_name_to_wp_id, self._current_edges = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id, self._logger)
        return self._current_annotation_name_to_wp_id, self._current_edges


    def _upload_graph_and_snapshots(self, upload_filepath):
        """Upload the graph and snapshots to the robot."""
        self._logger.info("Loading the graph from disk into local storage...")
        with open(upload_filepath + "/graph", "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            self._logger.info("Loaded graph has {} waypoints and {} edges".format(
                len(self._current_graph.waypoints), len(self._current_graph.edges)))
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(upload_filepath + "/waypoint_snapshots/{}".format(waypoint.snapshot_id),
                      "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
        for edge in self._current_graph.edges:
            # Load the edge snapshots from disk.
            with open(upload_filepath + "/edge_snapshots/{}".format(edge.snapshot_id),
                      "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        self._logger.info("Uploading the graph and snapshots to the robot...")
        self._graph_nav_client.upload_graph(lease=self._lease.lease_proto,
                                            graph=self._current_graph)
        # Upload the snapshots to the robot.
        for waypoint_snapshot in self._current_waypoint_snapshots.values():
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            self._logger.info("Uploaded {}".format(waypoint_snapshot.id))
        for edge_snapshot in self._current_edge_snapshots.values():
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            self._logger.info("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and it if is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            self._logger.info(
                   "Upload complete! The robot is currently not localized to the map; please localize", \
                   "the robot using commands (2) or (3) before attempting a navigation command.")

    def _navigate_to(self, *args):
        """Navigate to a specific waypoint."""
        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            self._logger.info("No waypoint provided as a destination for navigate to.")
            return

        self._lease = self._lease_wallet.get_lease()
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0], self._current_graph, self._current_annotation_name_to_wp_id, self._logger)
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        if not self.toggle_power(should_power_on=True):
            self._logger.info("Failed to power on the robot, and cannot complete navigate to request.")
            return

        # Stop the lease keepalive and create a new sublease for graph nav.
        self._lease = self._lease_wallet.advance()
        sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Navigate to the destination waypoint.
        is_finished = False
        nav_to_cmd_id = -1
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 1.0,
                                                               leases=[sublease.lease_proto])
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        self._lease = self._lease_wallet.advance()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

        # Update the lease and power off the robot if appropriate.
        if self._powered_on and not self._started_powered_on:
            # Sit the robot down + power off after the navigation command is complete.
            self.toggle_power(should_power_on=False)

        status = self._graph_nav_client.navigation_feedback(nav_to_cmd_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            return True, "Successfully completed the navigation commands!"
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            return False, "Robot got lost when navigating the route, the robot will now sit down."
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            return False, "Robot got stuck when navigating the route, the robot will now sit down."
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            return False, "Robot is impaired."
        else:
            return False, "Navigation command is not complete yet."

    def _navigate_route(self, *args):
        """Navigate through a specific route of waypoints."""
        if len(args) < 1:
            # If no waypoint ids are given as input, then return without requesting navigation.
            self._logger.error("No waypoints provided for navigate route.")
            return
        waypoint_ids = args[0]
        for i in range(len(waypoint_ids)):
            waypoint_ids[i] = graph_nav_util.find_unique_waypoint_id(
                waypoint_ids[i], self._current_graph, self._current_annotation_name_to_wp_id, self._logger)
            if not waypoint_ids[i]:
                # Failed to find the unique waypoint id.
                return

        edge_ids_list = []
        all_edges_found = True
        # Attempt to find edges in the current graph that match the ordered waypoint pairs.
        # These are necessary to create a valid route.
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                all_edges_found = False
                self._logger.error("Failed to find an edge between waypoints: ", start_wp, " and ", end_wp)
                self._logger.error(
                    "List the graph's waypoints and edges to ensure pairs of waypoints has an edge."
                )
                break

        self._lease = self._lease_wallet.get_lease()
        if all_edges_found:
            if not self.toggle_power(should_power_on=True):
                self._logger.error("Failed to power on the robot, and cannot complete navigate route request.")
                return

            # Stop the lease keepalive and create a new sublease for graph nav.
            self._lease = self._lease_wallet.advance()
            sublease = self._lease.create_sublease()
            self._lease_keepalive.shutdown()

            # Navigate a specific route.
            route = self._graph_nav_client.build_route(waypoint_ids, edge_ids_list)
            is_finished = False
            while not is_finished:
                # Issue the route command about twice a second such that it is easy to terminate the
                # navigation command (with estop or killing the program).
                nav_route_command_id = self._graph_nav_client.navigate_route(
                    route, cmd_duration=1.0, leases=[sublease.lease_proto])
                time.sleep(.5)  # Sleep for half a second to allow for command execution.
                # Poll the robot for feedback to determine if the route is complete. Then sit
                # the robot down once it is finished.
                is_finished = self._check_success(nav_route_command_id)

            self._lease = self._lease_wallet.advance()
            self._lease_keepalive = LeaseKeepAlive(self._lease_client)

            # Update the lease and power off the robot if appropriate.
            if self._powered_on and not self._started_powered_on:
                # Sit the robot down + power off after the navigation command is complete.
                self.toggle_power(should_power_on=False)

    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        return self._graph_nav_client.clear_graph(lease=self._lease.lease_proto)

    def toggle_power(self, should_power_on):
        """Power the robot on/off dependent on the current power state."""
        is_powered_on = self.check_is_powered_on()
        if not is_powered_on and should_power_on:
            # Power on the robot up before navigating when it is in a powered-off state.
            power_on(self._power_client)
            motors_on = False
            while not motors_on:
                future = self._robot_state_client.get_robot_state_async()
                state_response = future.result(timeout=10) # 10 second timeout for waiting for the state response.
                if state_response.power_state.motor_power_state == robot_state_pb2.PowerState.STATE_ON:
                    motors_on = True
                else:
                    # Motors are not yet fully powered on.
                    time.sleep(.25)
        elif is_powered_on and not should_power_on:
            # Safe power off (robot will sit then power down) when it is in a
            # powered-on state.
            safe_power_off(self._robot_command_client, self._robot_state_client)
        else:
            # Return the current power state without change.
            return is_powered_on
        # Update the locally stored power state.
        self.check_is_powered_on()
        return self._powered_on

    def check_is_powered_on(self):
        """Determine if the robot is powered on or off."""
        power_state = self._robot_state_client.get_robot_state().power_state
        self._powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        return self._powered_on

    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            self._logger.error("Robot got lost when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            self._logger.error("Robot got stuck when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            self._logger.error("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def _match_edge(self, current_edges, waypoint1, waypoint2):
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint2, to_waypoint=waypoint1)
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint1, to_waypoint=waypoint2)
        return None