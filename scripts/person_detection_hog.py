#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Detect a Gazebo person from the SEED-Noid RGB camera using OpenCV HOG."""

import threading
import time
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from rois_env.srv import (
    JudgeParam,
    JudgeParamResponse,
    detection,
    detectionResponse,
)
from rosi_seed_noid_sim.msg import PersonRoi


class HogPersonDetector:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.detection_rate = max(
            0.1, float(rospy.get_param("~detection_rate", 5.0))
        )
        self.required_consecutive = max(
            1, int(rospy.get_param("~required_consecutive", 3))
        )
        self.minimum_weight = float(rospy.get_param("~minimum_weight", 0.0))
        self.detection_scale = max(
            1.0, min(2.0, float(rospy.get_param("~detection_scale", 1.5)))
        )
        self.publish_debug_image = bool(
            rospy.get_param("~publish_debug_image", True)
        )

        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.run_id = 0
        self.active = False
        self.detected = False
        self.detected_at = ""
        self.person_count = 0
        self.consecutive_count = 0
        self.processed_frames = 0
        self.last_process_time = 0.0

        self.judge_publisher = rospy.Publisher(
            "/judge_param", Bool, queue_size=1, latch=True
        )
        self.debug_publisher = rospy.Publisher(
            "~debug_image", Image, queue_size=1
        )
        self.roi_publisher = rospy.Publisher(
            "/rosi_seed_noid_sim/person_roi", PersonRoi, queue_size=1, latch=True
        )
        self.detection_service = rospy.Service(
            "/detection", detection, self.handle_detection
        )
        self.start_service = rospy.Service(
            "/start", JudgeParam, self.handle_start
        )
        self.stop_service = rospy.Service(
            "/stop", JudgeParam, self.handle_stop
        )
        self.image_subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.handle_image,
            queue_size=1,
            buff_size=2**24,
        )

        self.judge_publisher.publish(Bool(data=False))
        rospy.loginfo(
            "HOG person detector is ready: topic=%s, rate=%.1f Hz, "
            "required_consecutive=%d, minimum_weight=%.2f, scale=%.2f",
            self.image_topic,
            self.detection_rate,
            self.required_consecutive,
            self.minimum_weight,
            self.detection_scale,
        )

    def handle_start(self, _request):
        self.start_detection()
        return JudgeParamResponse(result=True)

    def handle_stop(self, _request):
        self.stop_detection()
        return JudgeParamResponse(result=True)

    def handle_detection(self, request):
        trigger = request.trigger.strip().lower()
        if trigger == "start":
            self.start_detection()
        elif trigger == "stop":
            self.stop_detection()
        elif trigger not in ("", "status"):
            rospy.logwarn("Unsupported /detection trigger: %s", request.trigger)
            return detectionResponse(
                result="error",
                timestamp=self.detected_at,
                number=0,
            )

        with self.lock:
            result = "success" if self.detected else "waiting"
            timestamp = self.detected_at
            number = self.person_count if self.detected else 0

        return detectionResponse(
            result=result,
            timestamp=timestamp,
            number=number,
        )

    def start_detection(self):
        with self.lock:
            self.run_id += 1
            self.active = True
            self.detected = False
            self.detected_at = ""
            self.person_count = 0
            self.consecutive_count = 0
            self.processed_frames = 0
            self.last_process_time = 0.0

        self.judge_publisher.publish(Bool(data=False))
        rospy.loginfo(
            "HOG detection started; waiting for %d consecutive frames.",
            self.required_consecutive,
        )

    def stop_detection(self):
        with self.lock:
            self.run_id += 1
            self.active = False
            self.consecutive_count = 0
        rospy.loginfo("HOG detection stopped.")

    def handle_image(self, message):
        now = time.monotonic()
        with self.lock:
            if not self.active:
                return
            minimum_interval = 1.0 / self.detection_rate
            if now - self.last_process_time < minimum_interval:
                return
            self.last_process_time = now
            current_run_id = self.run_id

        if not self.processing_lock.acquire(blocking=False):
            return

        started_at = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            detections = self.detect_people(image)
            elapsed_ms = (time.perf_counter() - started_at) * 1000.0
            self.handle_detection_result(
                message,
                image,
                detections,
                elapsed_ms,
                current_run_id,
            )
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "cv_bridge conversion failed: %s", error)
        except cv2.error as error:
            rospy.logerr_throttle(5.0, "OpenCV HOG detection failed: %s", error)
        finally:
            self.processing_lock.release()

    def detect_people(self, image):
        detection_image = image
        if self.detection_scale != 1.0:
            detection_image = cv2.resize(
                image,
                None,
                fx=self.detection_scale,
                fy=self.detection_scale,
                interpolation=cv2.INTER_LINEAR,
            )
        rectangles, weights = self.hog.detectMultiScale(
            detection_image,
            hitThreshold=0.0,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
            groupThreshold=2,
        )

        detections = []
        for rectangle, weight in zip(rectangles, weights):
            score = float(weight)
            if score < self.minimum_weight:
                continue
            x, y, width, height = (int(value) for value in rectangle)
            if self.detection_scale != 1.0:
                x = int(round(x / self.detection_scale))
                y = int(round(y / self.detection_scale))
                width = int(round(width / self.detection_scale))
                height = int(round(height / self.detection_scale))
            detections.append((x, y, width, height, score))
        return detections

    def handle_detection_result(
        self,
        source_message,
        image,
        detections,
        elapsed_ms,
        current_run_id,
    ):
        with self.lock:
            if not self.active or current_run_id != self.run_id:
                return

            self.processed_frames += 1
            if detections:
                self.consecutive_count += 1
            else:
                self.consecutive_count = 0

            processed_frames = self.processed_frames
            consecutive_count = self.consecutive_count
            confirmed = consecutive_count >= self.required_consecutive

            if confirmed:
                self.active = False
                self.detected = True
                self.detected_at = datetime.now().isoformat(timespec="milliseconds")
                self.person_count = min(127, len(detections))

        if self.publish_debug_image:
            self.publish_debug(
                source_message,
                image,
                detections,
                consecutive_count,
            )

        rospy.loginfo(
            "HOG frame=%d detections=%d consecutive=%d/%d elapsed=%.1f ms",
            processed_frames,
            len(detections),
            consecutive_count,
            self.required_consecutive,
            elapsed_ms,
        )

        if confirmed:
            selected = max(detections, key=lambda detection: detection[4])
            roi = PersonRoi()
            roi.header = source_message.header
            roi.image_width = image.shape[1]
            roi.image_height = image.shape[0]
            roi.x_offset = selected[0]
            roi.y_offset = selected[1]
            roi.width = selected[2]
            roi.height = selected[3]
            roi.score = selected[4]
            self.roi_publisher.publish(roi)
            self.judge_publisher.publish(Bool(data=True))
            rospy.loginfo(
                "Published person ROI and /judge_param: true after %d consecutive "
                "detections.",
                consecutive_count,
            )

    def publish_debug(
        self,
        source_message,
        image,
        detections,
        consecutive_count,
    ):
        annotated = image.copy()
        for x, y, width, height, score in detections:
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                annotated,
                "person %.3f" % score,
                (x, max(18, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            "consecutive %d/%d" % (
                consecutive_count,
                self.required_consecutive,
            ),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        try:
            debug_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_message.header = source_message.header
            self.debug_publisher.publish(debug_message)
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "Debug image conversion failed: %s", error)


if __name__ == "__main__":
    rospy.init_node("person_detection_hog")
    HogPersonDetector()
    rospy.spin()
