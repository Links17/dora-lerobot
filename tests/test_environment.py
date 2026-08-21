def test_runtime_versions():
    import sys

    import lerobot

    assert sys.version_info[:2] == (3, 12)
    assert lerobot.__version__ == "0.6.1"
