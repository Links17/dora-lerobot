from pathlib import Path

import yaml


def test_every_so_arm_workflow_is_a_dora_graph():
    workflow_directory = Path("workflows/so_arm")
    paths = sorted(workflow_directory.glob("*.yaml"))
    assert {path.stem for path in paths} == {
        "teleoperate",
        "record",
        "replay",
        "inference_local",
        "inference_cloud",
    }
    for path in paths:
        graph = yaml.safe_load(path.read_text())
        assert isinstance(graph["nodes"], list)
        assert graph["nodes"]
