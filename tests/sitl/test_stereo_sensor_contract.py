#!/usr/bin/env python3
"""Regression tests for the Gazebo stereo sensor timing contract."""

from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as element_tree

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = (
    REPO_ROOT / "sim" / "px4_gz" / "models" /
    "smartdrone_x500_stereo" / "model.sdf"
)
SIM_CONFIG_PATH = (
    REPO_ROOT / "sim" / "px4_gz" / "config" / "smartdrone_sim.yaml"
)
WORLD_PATH = (
    REPO_ROOT / "sim" / "px4_gz" / "worlds" / "smartdrone_indoor.sdf"
)
CAMERA_NAMES = (
    "smartdrone_left_camera",
    "smartdrone_right_camera",
)


def _load_sim_config() -> dict[str, object]:
    lines = SIM_CONFIG_PATH.read_text(encoding="utf-8").splitlines()
    if lines and lines[0].startswith("%YAML:"):
        lines = lines[1:]
    payload = yaml.safe_load("\n".join(lines))
    if not isinstance(payload, dict):
        raise ValueError("simulation configuration root must be a mapping")
    return payload


class StereoSensorContractTest(unittest.TestCase):
    def setUp(self) -> None:
        root = element_tree.parse(MODEL_PATH).getroot()
        self.sensors = {
            sensor.attrib["name"]: sensor
            for sensor in root.findall(".//sensor")
            if sensor.attrib.get("name") in CAMERA_NAMES
        }
        self.config = _load_sim_config()

    def test_both_eyes_share_stream_and_rendering_contract(self) -> None:
        self.assertEqual(set(self.sensors), set(CAMERA_NAMES))
        expected_topics = {
            "smartdrone_left_camera": self.config["left_image_topic"],
            "smartdrone_right_camera": self.config["right_image_topic"],
        }
        for name, sensor in self.sensors.items():
            with self.subTest(camera=name):
                self.assertEqual(sensor.attrib.get("type"), "camera")
                self.assertEqual(
                    float(sensor.findtext("update_rate", "0")), 30.0,
                )
                self.assertEqual(sensor.findtext("topic"), expected_topics[name])
                image = sensor.find("camera/image")
                self.assertIsNotNone(image)
                assert image is not None
                self.assertEqual(image.findtext("width"), "640")
                self.assertEqual(image.findtext("height"), "480")
                self.assertEqual(image.findtext("format"), "L_INT8")
                self.assertEqual(image.findtext("anti_aliasing"), "0")

    def test_pairing_tolerance_remains_stricter_than_frame_period(self) -> None:
        pair_tolerance_ns = int(self.config["pair_tolerance_ns"])
        frame_period_ns = int(1_000_000_000 / 30)

        self.assertEqual(pair_tolerance_ns, 2_000_000)
        self.assertLess(pair_tolerance_ns, frame_period_ns)

    def test_scene_keeps_one_feature_preserving_shadow_source(self) -> None:
        world = element_tree.parse(WORLD_PATH).getroot().find("world")
        self.assertIsNotNone(world)
        assert world is not None
        self.assertEqual(world.findtext("scene/shadows"), "true")
        lights = {
            light.attrib.get("name"): light
            for light in world.findall("light")
        }
        self.assertEqual(set(lights), {"ceiling_key", "front_fill"})
        self.assertEqual(lights["ceiling_key"].attrib.get("type"), "spot")
        self.assertEqual(lights["ceiling_key"].findtext("cast_shadows"), "true")
        self.assertEqual(lights["ceiling_key"].findtext("direction"), "0.25 0 -1")
        self.assertEqual(lights["ceiling_key"].findtext("spot/outer_angle"), "1.5")
        self.assertEqual(lights["front_fill"].attrib.get("type"), "point")
        self.assertEqual(lights["front_fill"].findtext("cast_shadows"), "false")

if __name__ == "__main__":
    unittest.main()
