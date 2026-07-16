#!/usr/bin/env python3

"""Simulation-safe Move HRI that snapshots odometry for every command."""

import math
import threading
import time

import actionlib
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rois_env.msg import completed, executeAction, executeResult
from rois_env.srv import (
    component_status,
    component_statusResponse,
    move_get_param,
    move_get_paramResponse,
    move_set_param,
    move_set_paramResponse,
)


class MoveHriSimulation:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "SEED_Noid")
        self.move_timeout = max(1.0, float(rospy.get_param("~move_timeout", 20.0)))
        self.position_tolerance = max(
            0.005, float(rospy.get_param("~position_tolerance", 0.03))
        )
        self.kp = max(0.05, float(rospy.get_param("~kp", 0.5)))
        self.maximum_speed = max(0.05, float(rospy.get_param("~maximum_speed", 0.3)))
        self.minimum_speed = max(
            0.01, min(self.maximum_speed, float(rospy.get_param("~minimum_speed", 0.05)))
        )

        self.condition = threading.Condition()
        self.current_xy = None
        self.relative_line = [0, 0, 0]
        self.state = "READY"
        self.worker = None
        self.cancel_event = threading.Event()

        prefix = "/" + self.robot_name
        self.status_service = rospy.Service(
            prefix + "/get_state/Move", component_status, self.handle_status
        )
        self.set_service = rospy.Service(
            prefix + "/move_set_param", move_set_param, self.handle_set
        )
        self.get_service = rospy.Service(
            prefix + "/move_get_param", move_get_param, self.handle_get
        )
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.completion_publisher = rospy.Publisher(
            prefix + "/completed_command", completed, queue_size=1
        )
        self.odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.handle_odom, queue_size=1
        )
        self.action_server = actionlib.SimpleActionServer(
            prefix + "/execute/Move",
            executeAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.action_server.start()
        rospy.loginfo(
            "Move HRI simulation is ready: odom=/odom cmd_vel=/cmd_vel "
            "snapshot_each_start=true"
        )

    def handle_odom(self, message):
        with self.condition:
            self.current_xy = [
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            ]
            self.condition.notify_all()

    def handle_status(self, request):
        if request.component_name != "Move":
            return component_statusResponse("ERROR")
        with self.condition:
            return component_statusResponse(self.state)

    def handle_set(self, request):
        values = list(request.line)
        if len(values) != 3 or any(isinstance(value, bool) for value in values):
            return move_set_paramResponse("BAD PARAMETER")
        with self.condition:
            if self.state == "BUSY":
                return move_set_paramResponse("BUSY")
            self.relative_line = [int(value) for value in values]
        rospy.loginfo("Move relative target set: %s mm", self.relative_line)
        return move_set_paramResponse("OK")

    def handle_get(self, _request):
        with self.condition:
            return move_get_paramResponse(list(self.relative_line))

    def wait_for_position(self):
        deadline = time.monotonic() + 10.0
        with self.condition:
            while self.current_xy is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out waiting for /odom")
                self.condition.wait(remaining)
            return list(self.current_xy)

    def execute(self, request):
        result = executeResult()
        command = request.command_name
        if command == "start":
            try:
                start_xy = self.wait_for_position()
            except RuntimeError as error:
                rospy.logerr("Move start rejected: %s", error)
                result.success = "False"
                self.action_server.set_aborted(result)
                return
            with self.condition:
                if self.state == "BUSY":
                    result.success = "False"
                    self.action_server.set_aborted(result)
                    return
                relative_m = [
                    self.relative_line[0] / 1000.0,
                    self.relative_line[1] / 1000.0,
                ]
                self.state = "BUSY"
                self.cancel_event.clear()
            self.worker = threading.Thread(
                target=self.run_move, args=(start_xy, relative_m), daemon=True
            )
            self.worker.start()
            result.success = "True"
            self.action_server.set_succeeded(result)
            return
        if command in {"stop", "suspend"}:
            self.cancel_event.set()
            self.publish_zero_velocity()
            with self.condition:
                self.state = "STOPPED" if command == "stop" else "SUSPENDED"
            result.success = "True"
            self.action_server.set_succeeded(result)
            return
        result.success = "False"
        self.action_server.set_aborted(result)

    def run_move(self, start_xy, relative_m):
        target_xy = [start_xy[0] + relative_m[0], start_xy[1] + relative_m[1]]
        rospy.loginfo(
            "Move execution started: start=[%.3f, %.3f] target=[%.3f, %.3f]",
            start_xy[0], start_xy[1], target_xy[0], target_xy[1]
        )
        deadline = time.monotonic() + self.move_timeout
        rate = rospy.Rate(10)
        status = "timeup"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.cancel_event.is_set():
                status = "stopped"
                break
            with self.condition:
                current = list(self.current_xy) if self.current_xy else None
            if current is None:
                rate.sleep()
                continue
            error_x = target_xy[0] - current[0]
            error_y = target_xy[1] - current[1]
            if math.hypot(error_x, error_y) <= self.position_tolerance:
                status = "completed"
                break
            command = Twist()
            command.linear.x = self.velocity_for_error(error_x)
            command.linear.y = self.velocity_for_error(error_y)
            self.velocity_publisher.publish(command)
            rate.sleep()
        self.publish_zero_velocity()
        with self.condition:
            final_xy = list(self.current_xy) if self.current_xy else [math.nan, math.nan]
            self.state = "READY" if status == "completed" else "ERROR"
        message = completed(command_id="Move", status=status)
        self.completion_publisher.publish(message)
        rospy.loginfo(
            "Move completed: result=%s final=[%.3f, %.3f]",
            status, final_xy[0], final_xy[1]
        )

    def velocity_for_error(self, error):
        if abs(error) <= self.position_tolerance:
            return 0.0
        value = max(-self.maximum_speed, min(self.maximum_speed, self.kp * error))
        if abs(value) < self.minimum_speed:
            value = math.copysign(self.minimum_speed, error)
        return value

    def publish_zero_velocity(self):
        self.velocity_publisher.publish(Twist())


def main():
    rospy.init_node("move_hri_sim")
    MoveHriSimulation()
    rospy.spin()


if __name__ == "__main__":
    main()
