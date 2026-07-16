#!/usr/bin/env python3

"""Simulation-safe Navigation HRI component backed by move_base."""

import math
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from rois_env.msg import completed, executeAction, executeResult
from rois_env.srv import (
    component_status,
    component_statusResponse,
    navi_get_param,
    navi_get_paramResponse,
    navi_set_param,
    navi_set_paramResponse,
)


class NavigationHriSimulation:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "SEED_Noid")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.server_timeout = max(
            1.0, float(rospy.get_param("~move_base_server_timeout", 30.0))
        )
        self.navigation_timeout = max(
            5.0, float(rospy.get_param("~navigation_timeout", 75.0))
        )
        self.lock = threading.Lock()
        self.state = "UNINITIALIZED"
        self.goal_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.current_goal = None
        self.completion_sent = False

        prefix = "/" + self.robot_name
        self.status_service = rospy.Service(
            prefix + "/get_state/Navigation",
            component_status,
            self.handle_status,
        )
        self.set_service = rospy.Service(
            prefix + "/navi_set_param", navi_set_param, self.handle_set
        )
        self.get_service = rospy.Service(
            prefix + "/navi_get_param", navi_get_param, self.handle_get
        )
        self.completion_publisher = rospy.Publisher(
            prefix + "/completed_command", completed, queue_size=1
        )
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo(
            "Waiting up to %.1fs for move_base action server", self.server_timeout
        )
        if not self.move_base.wait_for_server(rospy.Duration(self.server_timeout)):
            raise RuntimeError("move_base action server did not become ready")
        self.action_server = actionlib.SimpleActionServer(
            prefix + "/execute/Navigation",
            executeAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.action_server.start()
        self.state = "READY"
        rospy.loginfo(
            "Navigation HRI simulation is ready: frame=%s backend=move_base",
            self.frame_id,
        )

    def handle_status(self, request):
        if request.component_name != "Navigation":
            return component_statusResponse("ERROR")
        with self.lock:
            return component_statusResponse(self.state)

    def handle_set(self, request):
        values = [float(value) for value in request.target_position]
        if len(values) != 7:
            return navi_set_paramResponse("BAD PARAMETER")
        if not all(math.isfinite(value) for value in values):
            return navi_set_paramResponse("BAD PARAMETER")
        quaternion_norm = sum(value * value for value in values[3:7]) ** 0.5
        if quaternion_norm < 0.5:
            return navi_set_paramResponse("BAD PARAMETER")
        with self.lock:
            if self.state == "BUSY":
                return navi_set_paramResponse("BUSY")
            self.goal_position = values
        rospy.loginfo("Navigation target set: %s", values)
        return navi_set_paramResponse("OK")

    def handle_get(self, _request):
        with self.lock:
            return navi_get_paramResponse(list(self.goal_position))

    def make_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        values = self.goal_position
        goal.target_pose.pose.position.x = values[0]
        goal.target_pose.pose.position.y = values[1]
        goal.target_pose.pose.position.z = values[2]
        goal.target_pose.pose.orientation.x = values[3]
        goal.target_pose.pose.orientation.y = values[4]
        goal.target_pose.pose.orientation.z = values[5]
        goal.target_pose.pose.orientation.w = values[6]
        return goal

    def execute(self, request):
        result = executeResult()
        command = request.command_name
        if command == "start":
            with self.lock:
                if self.state == "BUSY":
                    result.success = "False"
                    self.action_server.set_aborted(result)
                    return
                self.current_goal = self.make_goal()
                self.completion_sent = False
                self.state = "BUSY"
            self.move_base.send_goal(
                self.current_goal,
                done_cb=self.navigation_done,
                active_cb=lambda: rospy.loginfo("Navigation route execution started"),
            )
            threading.Thread(target=self.cancel_on_timeout, daemon=True).start()
            result.success = "True"
            self.action_server.set_succeeded(result)
            return
        if command in {"stop", "suspend"}:
            self.move_base.cancel_goal()
            with self.lock:
                self.state = "STOPPED" if command == "stop" else "SUSPENDED"
            result.success = "True"
            self.action_server.set_succeeded(result)
            return
        if command == "resume":
            with self.lock:
                goal = self.current_goal
                resumable = self.state == "SUSPENDED" and goal is not None
                if resumable:
                    self.state = "BUSY"
            if not resumable:
                result.success = "False"
                self.action_server.set_aborted(result)
                return
            self.move_base.send_goal(goal, done_cb=self.navigation_done)
            result.success = "True"
            self.action_server.set_succeeded(result)
            return
        result.success = "False"
        self.action_server.set_aborted(result)

    def publish_completion(self, status):
        message = completed()
        message.command_id = "Navigation"
        message.status = status
        self.completion_publisher.publish(message)

    def cancel_on_timeout(self):
        deadline = time.monotonic() + self.navigation_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.completion_sent or self.state != "BUSY":
                    return
            time.sleep(0.1)
        with self.lock:
            if self.completion_sent or self.state != "BUSY":
                return
            self.completion_sent = True
            self.state = "ERROR"
        self.move_base.cancel_goal()
        self.publish_completion("timeout")
        rospy.logerr("Navigation timed out after %.1fs", self.navigation_timeout)

    def navigation_done(self, status, _result):
        with self.lock:
            if self.completion_sent:
                return
            self.completion_sent = True
            succeeded = status == GoalStatus.SUCCEEDED
            self.state = "READY" if succeeded else "ERROR"
        completion_status = "completed" if succeeded else "failed"
        self.publish_completion(completion_status)
        rospy.loginfo(
            "Navigation completed: move_base_status=%d result=%s",
            status,
            completion_status,
        )


def main():
    rospy.init_node("navigation_hri_sim")
    NavigationHriSimulation()
    rospy.spin()


if __name__ == "__main__":
    main()
