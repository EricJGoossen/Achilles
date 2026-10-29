"""
Runs Project Otto against the sentry co-simulation.

This is Otto's own application -- MainApplication, its main loop, autonav, odometry and ArUco
localization, unmodified -- started with the sentry's configs plus a small override that points
its devices at the sim:

  - mcb_comms.port_name: the sim's pseudo-terminal for the MCB's UART (run-dir/mcb_uart);
  - autonav.lidars[*].port: the sim's LD19 pseudo-terminals (run-dir/lidar_front, lidar_rear);
  - logging.root_dir: run-dir/otto_logs;
  - no USB/driver resets or device listings, no Arducams.

and hardware factories (MainApplication's `hardware` argument) that open a blank-frame camera
(blank_camera.py; the RealSense isn't modelled yet) instead of a real one, record no video, and
stand in for the GPU plate detector with one that finds nothing.

Usage (from Otto's virtualenv; scripts/run-sentry-sim.sh does this):
    python run_otto.py --otto-dir ../project-otto --run-dir /tmp/sentry-sim
"""

import argparse
import logging
import os
import sys
from typing import Any, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
SENTRY_DIR = os.path.dirname(HERE)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--otto-dir", required=True, help="the project-otto checkout")
    parser.add_argument("--run-dir", default="/tmp/sentry-sim", help="the sim's --run-dir")
    parser.add_argument(
        "--config",
        action="append",
        default=None,
        help="Otto config files, highest precedence first, as Otto's loader takes them "
        "(default: sentry_name_config.yaml, base_config.yaml)",
    )
    parser.add_argument("--silent", action="store_true", help="suppress logging to the console")
    return parser.parse_args()


def _write_override(run_dir: str, sentry_config_lidars: List[Any]) -> str:
    """The sim's device paths, merged over the robot's configs."""
    import yaml

    lidar_names = ["lidar_front", "lidar_rear"]
    lidars = []
    for name, lidar in zip(lidar_names, sentry_config_lidars, strict=True):
        lidar = dict(lidar)
        lidar["port"] = os.path.join(run_dir, name)
        lidars.append(lidar)
    override = {
        "mcb_comms": {"port_name": os.path.join(run_dir, "mcb_uart")},
        "autonav": {"lidars": lidars},
        "logging": {"root_dir": os.path.join(run_dir, "otto_logs")},
        "realsense_reset": {
            "reset_usb": False,
            "reload_drivers": False,
            "reset_hardware": False,
            "reset_usb_reload_drivers_timer": 0,
            "reset_hardware_timer": 0,
        },
        "arducam_aruco": False,
        "arducam_cameras": [],
    }
    path = os.path.join(run_dir, "otto_sim_override.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(override, f)
    return path


def main():
    args = _parse_args()
    otto_dir = os.path.abspath(args.otto_dir)
    sys.path.insert(0, otto_dir)
    sys.path.insert(0, HERE)
    os.chdir(otto_dir)  # Otto resolves its assets (detector weights) relative to the checkout
    os.makedirs(os.path.join(args.run_dir, "otto_logs"), exist_ok=True)

    import yaml

    from project_otto.application_config import ApplicationConfig
    from project_otto.core.config_deserialization import load_merged_yaml_from_files
    from project_otto.core.data_logging import VideoDumper
    from project_otto.main import CommandlineOptions, HardwareFactories, MainApplication
    from project_otto.vision.target_detector import TargetDetector

    from blank_camera import BlankRealsenseImagesetSource

    configs = args.config or [
        os.path.join(otto_dir, "sentry_name_config.yaml"),
        os.path.join(otto_dir, "base_config.yaml"),
    ]
    with open(configs[0]) as f:
        robot_lidars = (yaml.safe_load(f).get("autonav") or {}).get("lidars") or []
    if len(robot_lidars) != 2:
        raise SystemExit(f"expected two LiDARs in {configs[0]}, found {len(robot_lidars)}")
    # Otto's loader gives the first file precedence.
    configs = [_write_override(args.run_dir, robot_lidars)] + configs
    config = load_merged_yaml_from_files(configs, ApplicationConfig)

    def create_camera(_serial: Optional[str], _stream_config: Any) -> BlankRealsenseImagesetSource:
        return BlankRealsenseImagesetSource()

    class NoVideo(VideoDumper[Any]):
        """Records nothing: the sim's frames can be regenerated from its pose log."""

        def add_imageset(self, imageset: Any):
            pass

        def _save_imageset(self, color_image: Any, depth_image: Any, timestamp: Any):
            pass

    class NoPlates(TargetDetector):
        """The detector's place in the loop, without its GPU: no robots are simulated."""

        def __init__(self, _weights_path: str):
            pass

        def detect_targets(self, imagesets: Any) -> Any:
            return [set() for _ in imagesets]

    hardware = HardwareFactories(
        create_camera=create_camera,
        create_video_dumper=NoVideo,
        create_detector=NoPlates,
        manage_devices=False,
    )
    cli = CommandlineOptions(argparse.Namespace(config_paths=configs, silent=args.silent))
    try:
        with MainApplication(config, cli, hardware) as app:
            app.run_forever()
    except Exception:
        logging.critical("Unhandled exception in main", exc_info=True)
        raise


if __name__ == "__main__":
    main()
