from __future__ import annotations

from dora_lerobot.adapters.calibration import CalibrationProfile


def test_calibration_profile_round_trips_and_rejects_wrong_device(tmp_path) -> None:
    path = tmp_path / "follower.json"
    profile = CalibrationProfile.create(
        robot_id="so-arm-a", role="follower", calibration_dir=tmp_path, device_id="follower-a"
    )
    profile.save(path)

    assert CalibrationProfile.load(path, robot_id="so-arm-a", role="follower") == profile
