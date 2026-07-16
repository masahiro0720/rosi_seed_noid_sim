#!/usr/bin/env python3

"""Estimate the detected Gazebo person's 3-D position from PointCloud2."""

import math
import statistics
import threading
import time

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_geometry_msgs  # noqa: F401  Registers PointStamped conversions.
import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2

from rois_env.srv import GetPosition, GetPositionResponse
from rosi_seed_noid_sim.msg import PersonRoi


def sample_pixels(roi, grid_size=7):
    """Return a centered grid inside a person ROI, biased toward the torso."""
    if roi.image_width <= 0 or roi.image_height <= 0:
        return []
    if roi.width <= 0 or roi.height <= 0:
        return []

    center_x = roi.x_offset + roi.width * 0.5
    center_y = roi.y_offset + roi.height * 0.45
    half_width = max(2.0, roi.width * 0.12)
    half_height = max(2.0, roi.height * 0.10)
    grid_size = max(3, int(grid_size))
    pixels = []
    for row in range(grid_size):
        y = center_y - half_height + (2.0 * half_height * row / (grid_size - 1))
        for column in range(grid_size):
            x = center_x - half_width + (
                2.0 * half_width * column / (grid_size - 1)
            )
            u = min(roi.image_width - 1, max(0, int(round(x))))
            v = min(roi.image_height - 1, max(0, int(round(y))))
            pixels.append((u, v))
    return pixels


def cloud_uvs(pixels, image_width, image_height, cloud_width, cloud_height):
    """Map RGB pixels to organized or Gazebo-flattened PointCloud2 indices."""
    if cloud_width == image_width and cloud_height == image_height:
        return pixels
    if cloud_height == 1 and cloud_width == image_width * image_height:
        return [(v * image_width + u, 0) for u, v in pixels]
    raise ValueError(
        "PointCloud layout does not match the RGB image: "
        f"image={image_width}x{image_height}, cloud={cloud_width}x{cloud_height}"
    )


def median_xyz(points, minimum_depth=0.5, maximum_depth=5.0):
    """Reject invalid optical-frame points and return their coordinate median."""
    valid = [
        (float(x), float(y), float(z))
        for x, y, z in points
        if all(math.isfinite(value) for value in (x, y, z))
        and minimum_depth <= float(z) <= maximum_depth
    ]
    if not valid:
        raise ValueError("person ROI contains no finite depth points")
    return tuple(statistics.median(point[index] for point in valid) for index in range(3))


class DepthPersonLocalizer:
    def __init__(self):
        self.roi_topic = rospy.get_param(
            "~roi_topic", "/rosi_seed_noid_sim/person_roi"
        )
        self.cloud_topic = rospy.get_param("~cloud_topic", "/camera/points")
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.wait_timeout = max(0.1, float(rospy.get_param("~wait_timeout", 5.0)))
        self.maximum_roi_age = max(
            0.1, float(rospy.get_param("~maximum_roi_age", 10.0))
        )
        self.maximum_pair_delta = max(
            0.1, float(rospy.get_param("~maximum_pair_delta", 2.0))
        )
        self.minimum_valid_points = max(
            1, int(rospy.get_param("~minimum_valid_points", 5))
        )
        self.grid_size = max(3, int(rospy.get_param("~grid_size", 7)))

        self.condition = threading.Condition()
        self.latest_roi = None
        self.latest_cloud = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.position_publisher = rospy.Publisher(
            "/rosi_seed_noid_sim/person_position",
            PointStamped,
            queue_size=1,
            latch=True,
        )
        self.roi_subscriber = rospy.Subscriber(
            self.roi_topic, PersonRoi, self.handle_roi, queue_size=1
        )
        self.cloud_subscriber = rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self.handle_cloud,
            queue_size=1,
            buff_size=2**25,
        )
        self.position_service = rospy.Service(
            "/get_position", GetPosition, self.handle_get_position
        )
        rospy.loginfo(
            "Depth person localizer is ready: roi=%s cloud=%s target=%s",
            self.roi_topic,
            self.cloud_topic,
            self.target_frame,
        )

    def handle_roi(self, message):
        with self.condition:
            self.latest_roi = message
            self.condition.notify_all()
        rospy.loginfo(
            "Person ROI received: x=%d y=%d width=%d height=%d score=%.3f",
            message.x_offset,
            message.y_offset,
            message.width,
            message.height,
            message.score,
        )

    def handle_cloud(self, message):
        with self.condition:
            self.latest_cloud = message
            self.condition.notify_all()

    def wait_for_inputs(self):
        deadline = time.monotonic() + self.wait_timeout
        with self.condition:
            while self.latest_roi is None or self.latest_cloud is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out waiting for person ROI and PointCloud2")
                self.condition.wait(remaining)
            return self.latest_roi, self.latest_cloud

    def validate_timestamps(self, roi, cloud):
        now = rospy.Time.now()
        if roi.header.stamp != rospy.Time(0):
            roi_age = (now - roi.header.stamp).to_sec()
            if roi_age < 0 or roi_age > self.maximum_roi_age:
                raise RuntimeError(f"person ROI is stale: age={roi_age:.3f}s")
        if roi.header.stamp != rospy.Time(0) and cloud.header.stamp != rospy.Time(0):
            delta = abs((cloud.header.stamp - roi.header.stamp).to_sec())
            if delta > self.maximum_pair_delta:
                raise RuntimeError(
                    f"person ROI and PointCloud2 are not synchronized: delta={delta:.3f}s"
                )

    def estimate_position(self):
        roi, cloud = self.wait_for_inputs()
        self.validate_timestamps(roi, cloud)
        pixels = sample_pixels(roi, self.grid_size)
        uvs = cloud_uvs(
            pixels,
            roi.image_width,
            roi.image_height,
            cloud.width,
            cloud.height,
        )
        points = list(
            point_cloud2.read_points(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=False,
                uvs=uvs,
            )
        )
        finite = [
            point
            for point in points
            if all(math.isfinite(value) for value in point)
            and 0.5 <= float(point[2]) <= 5.0
        ]
        if len(finite) < self.minimum_valid_points:
            raise RuntimeError(
                f"person ROI has only {len(finite)} valid depth points; "
                f"required={self.minimum_valid_points}"
            )
        x, y, z = median_xyz(finite)
        source = PointStamped()
        source.header = cloud.header
        if not source.header.frame_id:
            source.header.frame_id = roi.header.frame_id or "camera_optical_frame"
        source.point.x = x
        source.point.y = y
        source.point.z = z
        transformed = self.tf_buffer.transform(
            source, self.target_frame, rospy.Duration(self.wait_timeout)
        )
        values = [
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        ]
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"transformed person position is invalid: {values}")
        self.position_publisher.publish(transformed)
        rospy.loginfo(
            "Depth localization succeeded: frame=%s position=[%.3f, %.3f, %.3f] "
            "valid_points=%d",
            self.target_frame,
            values[0],
            values[1],
            values[2],
            len(finite),
        )
        return values

    def handle_get_position(self, _request):
        try:
            return GetPositionResponse(self.estimate_position())
        except (RuntimeError, ValueError, tf2_ros.TransformException) as error:
            rospy.logerr("Depth localization failed: %s", error)
            return GetPositionResponse([0.0, 0.0, 0.0])


def main():
    rospy.init_node("person_localization_depth")
    DepthPersonLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
