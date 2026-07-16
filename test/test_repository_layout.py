#!/usr/bin/env python3

import unittest
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class RepositoryLayoutTests(unittest.TestCase):
    def test_package_metadata_matches_repository(self):
        package = ET.parse(str(PACKAGE_ROOT / "package.xml")).getroot()
        self.assertEqual(package.findtext("name"), "rosi_seed_noid_sim")
        dependencies = {node.text for node in package.findall("depend")}
        self.assertTrue(
            {
                "cv_bridge",
                "gazebo_ros",
                "geometry_msgs",
                "rois_env",
                "rospy",
                "seed_r7_gazebo",
                "sensor_msgs",
                "std_msgs",
                "tf2_geometry_msgs",
                "tf2_ros",
            }.issubset(dependencies)
        )

    def test_launch_uses_overlay_world_and_never_starts_real_driver(self):
        launch = (PACKAGE_ROOT / "launch" / "seed_noid_person_world.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn("seed_r7_empty_world.launch", launch)
        self.assertIn("$(find rosi_seed_noid_sim)/worlds/person_detection.world", launch)
        self.assertNotIn("seed_r7_bringup", launch)

    def test_world_contains_one_stationary_person_at_three_metres(self):
        world_path = PACKAGE_ROOT / "worlds" / "person_detection.world"
        root = ET.parse(str(world_path)).getroot()
        actors = root.findall("./world/actor")
        self.assertEqual([actor.get("name") for actor in actors], ["sim_person"])
        poses = root.findall("./world/actor/script/trajectory/waypoint/pose")
        self.assertEqual(len(poses), 2)
        self.assertTrue(all(pose.text.strip().startswith("3 0 0 ") for pose in poses))

    def test_hog_node_uses_camera_frames_and_not_a_success_timer(self):
        source = (PACKAGE_ROOT / "scripts" / "person_detection_hog.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('rospy.get_param("~image_topic", "/camera/image_raw")', source)
        self.assertIn("cv2.HOGDescriptor_getDefaultPeopleDetector()", source)
        self.assertIn("self.required_consecutive", source)
        self.assertIn('"/start", JudgeParam, self.handle_start', source)
        self.assertIn('"/stop", JudgeParam, self.handle_stop', source)
        self.assertIn('"/detection", detection, self.handle_detection', source)
        self.assertIn('"/judge_param", Bool', source)
        self.assertIn('"/rosi_seed_noid_sim/person_roi", PersonRoi', source)
        self.assertNotIn("rospy.Timer", source)

    def test_depth_localizer_uses_roi_flat_cloud_tf_and_real_service(self):
        source = (
            PACKAGE_ROOT / "scripts" / "person_localization_depth.py"
        ).read_text(encoding="utf-8")
        self.assertIn('rospy.get_param("~cloud_topic", "/camera/points")', source)
        self.assertIn("v * image_width + u", source)
        self.assertIn('self.target_frame = rospy.get_param("~target_frame", "base_link")', source)
        self.assertIn('"/get_position", GetPosition', source)
        self.assertIn('"/rosi_seed_noid_sim/person_position"', source)
        self.assertNotIn("ROSI_HRI_MOCK_POSITION", source)


if __name__ == "__main__":
    unittest.main()
