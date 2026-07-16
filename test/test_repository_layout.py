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
        self.assertIn("gazebo_ros", dependencies)
        self.assertIn("seed_r7_gazebo", dependencies)

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


if __name__ == "__main__":
    unittest.main()
