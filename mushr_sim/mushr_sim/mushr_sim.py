#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

import time
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from mushr_sim.fake_urg import FakeURG
from mushr_base.motion_model import KinematicCarMotionModel
from mushr_sim_interfaces.srv import CarPose
from nav_msgs.srv import GetMap
from mushr_sim.simulated_car import SimulatedCar

class MushrSim(Node):
    """
    __init__: Initialize params, publishers, subscribers, and timer
    """

    def __init__(self):
        super().__init__("mushr_sim")
        self.declare_parameter("use_tf_prefix", True)
        self.use_tf_prefix = bool(self.get_parameter("use_tf_prefix").value)

        # speed (rpm) = SPEED_TO_ERPM_OFFSET + SPEED_TO_ERPM_GAIN * speed (m/s)
        self.declare_parameter("vesc.speed_to_erpm_offset", 0.0)
        self.declare_parameter("vesc.speed_to_erpm_gain", 4614.0)
        self.speed_to_erpm_offset = float(self.get_parameter("vesc.speed_to_erpm_offset").value)
        self.speed_to_erpm_gain = float(self.get_parameter("vesc.speed_to_erpm_gain").value)

        # servo angle = STEERING_TO_SERVO_OFFSET + STEERING_TO_SERVO_GAIN * steering_angle (rad)
        self.declare_parameter("vesc.steering_angle_to_servo_offset", 0.5304)
        self.declare_parameter("vesc.steering_angle_to_servo_gain", -1.2135)
        steering_to_servo_offset = float(self.get_parameter("vesc.steering_angle_to_servo_offset").value)
        steering_to_servo_gain = float(self.get_parameter("vesc.steering_angle_to_servo_gain").value)

        # Car geometry
        self.declare_parameter("vesc.chassis_length", 0.33)
        self.declare_parameter("vesc.wheelbase", 0.25)
        self.car_length = float(self.get_parameter("vesc.chassis_length").value)
        self.car_width = float(self.get_parameter("vesc.wheelbase").value)

        # Wheel radius (constant here; make a param if you want)
        self.car_wheel_radius = 0.0976 / 2.0

        # Rate at which to publish joints and tf
        self.declare_parameter("update_rate", 20.0)
        self.update_rate = float(self.get_parameter("update_rate").value)

        # Motion model parameters
        self.declare_parameter("motion_model.vel_bias", 0.0)
        self.declare_parameter("motion_model.vel_std", 0.0001)
        self.declare_parameter("motion_model.delta_bias", 0.0)
        self.declare_parameter("motion_model.delta_std", 0.000001)
        self.declare_parameter("motion_model.x_bias", 0.0)
        self.declare_parameter("motion_model.x_std", 0.0000001)
        self.declare_parameter("motion_model.x_vel_std", 0.001)
        self.declare_parameter("motion_model.y_bias", 0.0)
        self.declare_parameter("motion_model.y_std", 0.000001)
        self.declare_parameter("motion_model.y_vel_std", 0.001)
        self.declare_parameter("motion_model.theta_bias", 0.0)
        self.declare_parameter("motion_model.theta_std", 0.000001)

        # FakeURG sensor parameters
        self.declare_parameter("fake_urg.update_rate", 10.0)
        self.declare_parameter("fake_urg.theta_discretization", 656)
        self.declare_parameter("fake_urg.z_hit", 0.8)
        self.declare_parameter("fake_urg.z_short", 0.03)
        self.declare_parameter("fake_urg.z_max", 0.16)
        self.declare_parameter("fake_urg.z_blackout_max", 50)
        self.declare_parameter("fake_urg.z_rand", 0.01)
        self.declare_parameter("fake_urg.z_sigma", 0.03)
        self.declare_parameter("fake_urg.tf_prefix", "")
        self.declare_parameter("fake_urg.angle_min", -2.0943951)
        self.declare_parameter("fake_urg.angle_max", 2.0943951)
        self.declare_parameter("fake_urg.angle_step", 0.006396)
        self.declare_parameter("fake_urg.min_range_meters", 0.02)
        self.declare_parameter("fake_urg.max_range_meters", 0.16)
        self.declare_parameter("fake_urg.laser_yaw_offset", np.pi)

        # Extract motion model parameters
        self.motion_params = {
            "vel_bias": float(self.get_parameter("motion_model.vel_bias").value),
            "vel_std": float(self.get_parameter("motion_model.vel_std").value),
            "delta_bias": float(self.get_parameter("motion_model.delta_bias").value),
            "delta_std": float(self.get_parameter("motion_model.delta_std").value),
            "x_bias": float(self.get_parameter("motion_model.x_bias").value),
            "x_std": float(self.get_parameter("motion_model.x_std").value),
            "x_vel_std": float(self.get_parameter("motion_model.x_vel_std").value),
            "y_bias": float(self.get_parameter("motion_model.y_bias").value),
            "y_std": float(self.get_parameter("motion_model.y_std").value),
            "y_vel_std": float(self.get_parameter("motion_model.y_vel_std").value),
            "theta_bias": float(self.get_parameter("motion_model.theta_bias").value),
            "theta_std": float(self.get_parameter("motion_model.theta_std").value),
            "steering_to_servo_offset": steering_to_servo_offset,
            "steering_to_servo_gain": steering_to_servo_gain,
            "car_length": self.car_length,
            "car_width": self.car_width,
            "car_wheel_radius": self.car_wheel_radius,
        }

        # Extract FakeURG sensor parameters
        self.sensor_params = {
            "update_rate": float(self.get_parameter("fake_urg.update_rate").value),
            "theta_discretization": int(self.get_parameter("fake_urg.theta_discretization").value),
            "z_hit": float(self.get_parameter("fake_urg.z_hit").value),
            "z_short": float(self.get_parameter("fake_urg.z_short").value),
            "z_max": float(self.get_parameter("fake_urg.z_max").value),
            "z_blackout_max": int(self.get_parameter("fake_urg.z_blackout_max").value),
            "z_rand": float(self.get_parameter("fake_urg.z_rand").value),
            "z_sigma": float(self.get_parameter("fake_urg.z_sigma").value),
            "tf_prefix": str(self.get_parameter("fake_urg.tf_prefix").value),
            "angle_min": float(self.get_parameter("fake_urg.angle_min").value),
            "angle_max": float(self.get_parameter("fake_urg.angle_max").value),
            "angle_step": float(self.get_parameter("fake_urg.angle_step").value),
            "min_range_meters": float(self.get_parameter("fake_urg.min_range_meters").value),
            "max_range_meters": float(self.get_parameter("fake_urg.max_range_meters").value),
            "laser_yaw_offset": float(self.get_parameter("fake_urg.laser_yaw_offset").value),
        }

        # The map and map params
        self.permissible_region = None
        self.map_info = None


        self.map_service_name = '/map_server/map'

        self.map_client = self.create_client(GetMap, self.map_service_name)

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map Server not available, waiting again...')

        def get_map():
            req = GetMap.Request()
            future = self.map_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                map_msg = future.result().map  # full nav_msgs/OccupancyGrid

                array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
                permissible_region = np.zeros_like(array_255, dtype=bool)
                permissible_region[array_255 == 0] = 1

                return permissible_region, map_msg.info, map_msg
            else:
                self.get_logger().error('Service call failed %r' % (future.exception(),))
                return None, None, None
                        
        # Get the map - retry until map_server is active and returns valid data
        while True:
            self.permissible_region, self.map_info, self.raw_map_msg = get_map()
            if self.raw_map_msg is not None and self.raw_map_msg.info.resolution > 0:
                break
            self.get_logger().info("Map not ready (resolution=0), retrying...")
            time.sleep(1.0)

        # Publishes joint messages
        self.br = tf2_ros.TransformBroadcaster(self)

        # Duration param controls how often to publish default map to odom tf
        # if no other nodes are publishing it
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._cars = []
        
        self._car_reposition_srv = self.create_service(CarPose, "reposition", self._car_reposition_cb)
        self._car_spawn_srv = self.create_service(CarPose, "spawn", self._spawn_cb)

        self.last_stamp = None
        # Timer to updates joints and tf
        period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(period, self.simulate_cb)

        self.default_motion_model = KinematicCarMotionModel(**self.motion_params)
        static_br = tf2_ros.StaticTransformBroadcaster(self)

        # Declare and read list of car names
        self.declare_parameter("car_names", [""])
        # pass in the list of car names in logger

        car_names = list(self.get_parameter("car_names").value)

        self._pending_spawns = []

        for car_name in car_names:
            self.declare_parameter(f"{car_name}.initial_x", 0.0)
            self.declare_parameter(f"{car_name}.initial_y", 0.0)
            self.declare_parameter(f"{car_name}.initial_theta", 0.0)

            initial_x = float(self.get_parameter(f"{car_name}.initial_x").value)
            initial_y = float(self.get_parameter(f"{car_name}.initial_y").value)
            initial_theta = float(self.get_parameter(f"{car_name}.initial_theta").value)

            self._pending_spawns.append((car_name, initial_x, initial_y, initial_theta))

        # timer that will run once we are spinning
        self._spawn_timer = self.create_timer(0.1, self._try_spawn_pending)

    def _try_spawn_pending(self):
        if not self._pending_spawns:
            # stop the timer once all cars are spawned
            self._spawn_timer.cancel()
            return

        car_name, x, y, theta = self._pending_spawns[0]
        car_tf_prefix = car_name + "/" if self.use_tf_prefix else ""

        # quick readiness check (don’t block forever)
        if not self.tf_buffer.can_transform(
            car_tf_prefix + "base_link",
            car_tf_prefix + "base_footprint",
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.0),
        ):
            self.get_logger().info(f"Waiting for TF: {car_tf_prefix}base_footprint -> {car_tf_prefix}base_link")
            return

        # now safe to spawn
        req = CarPose.Request()
        req.car_name = car_name
        req.x = x
        req.y = y
        req.theta = theta
        resp = CarPose.Response()

        ok = self.spawn_car(req, car_name, x, y, theta, resp)

        if ok:
            self._pending_spawns.pop(0)
        else:
            # TF exists but spawn failed for another reason — log and retry next tick
            self.get_logger().warn(f"Spawn attempt failed for {car_name}, retrying...")


    def _spawn_cb(self, request, response):
        return self.spawn_car(request, request.car_name, request.x, request.y, request.theta, response)

    def spawn_car(self, request, car_name, x, y, theta, response):
        response.success = False
        if any(map(lambda car: car.car_name == car_name, self._cars)):
            return False
        sensor_params = self.sensor_params.copy()
        car_tf_prefix = car_name + "/" if self.use_tf_prefix else ""
        self.get_logger().info("Spawning car named '{}' at ({}, {}, {})".format(car_tf_prefix , x, y, theta))
        # self.get_logger().info("transform: {}".format())
        sensor_params["tf_prefix"] = car_tf_prefix
        try:
            transform = self.tf_buffer.lookup_transform(
                        car_tf_prefix + "laser_link",
                        car_tf_prefix + "base_link",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=10)
                        )
            # Drop stamp header
            transform = transform.transform

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn("Failed to spawn new car named '{}' because no TF information was found. Exception: {} - {}".format(car_name, type(e).__name__, str(e)))
            return False

        sensor = FakeURG(self, self.raw_map_msg, topic_namespace=car_name + "/scan", x_offset=transform.translation.x,
                         **sensor_params)
        
        new_car = SimulatedCar(self, car_name, x, y, theta, speed_to_erpm_gain=self.speed_to_erpm_gain,
                               speed_to_erpm_offset=self.speed_to_erpm_offset,
                               permissible_region=self.permissible_region, map_info=self.map_info, sensor_model=sensor,
                               motion_model=self.default_motion_model, tf_prefix=car_tf_prefix)
        self._cars.append(new_car)

        response.success = True

        return response

    def simulate_cb(self):
        """
        Callback occurring at a rate of self.UPDATE_RATE. Updates the car joint
                  angles and tf of the base_footprint w.r.t odom. Also publishes robot state as a PoseStamped msg.
          event: Information about when this callback occurred
        """
        # timeout = rclpy.duration.Duration(seconds=2.0)
        now = self.get_clock().now()
        # Get the time since the last update
        if self.last_stamp is None:
            self.last_stamp = now
        dt = (now - self.last_stamp).nanoseconds

        # self.get_logger().info("Simulating with dt =" + str(dt))

        # NOTE(nickswalker5-6-21): There's more stuff that could optionally
        # happen here. All the state updates could be calculated at once
        # because the motion model is vectorized (ROS operations would
        # need to be looped over after). Collision checking between
        # vehicles.
        for car in self._cars:
            car.simulate(dt, now)
            # self.get_logger().info(f"car {car.transform}")
            # Publish the tf from odom to base_footprint
            self.br.sendTransform(car.transform)

        self.last_stamp = now

    def _car_reposition_cb(self, request, response):
        response.success = False
        for car in self._cars:
            if car.car_name == request.car_name:
                car.reposition(request.x, request.y, request.theta)
                response.success = True
                break
        return response


def check_position_in_bounds(x, y, permissible_region):
    if permissible_region is None:
        return True
    return (
            0 <= x < permissible_region.shape[1] and \
            0 <= y < permissible_region.shape[0] and \
            permissible_region[
                int(y + 0.5), int(x + 0.5)
            ]
    )

def main(args=None):
    rclpy.init(args=args)
    node = MushrSim()
    node.get_logger().info("Starting MushrSim node.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
