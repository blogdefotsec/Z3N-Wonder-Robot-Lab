"""Unitree SDK2 bridge for Isaac Sim / Isaac Lab.

This script mirrors the behavior of `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`
inside Isaac Sim. It subscribes to `rt/lowcmd`, applies the received motor commands to an
Isaac Lab articulation, and publishes `rt/lowstate`, `rt/sportmodestate`, and
`rt/wirelesscontroller`.

Example:

    ./isaaclab.sh -p scripts/demos/unitree_sdk2_bridge.py --robot go2 --interface lo
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

ISAACLAB_ROOT = Path(__file__).resolve().parents[2]
for relative_path in (
    "source/isaaclab",
    "source/isaaclab_assets",
    "source/isaaclab_rl",
    "source/isaaclab_tasks",
    "source/isaaclab_mimic",
):
    package_path = str(ISAACLAB_ROOT / relative_path)
    if package_path not in sys.path:
        sys.path.append(package_path)

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Bridge Unitree SDK2 DDS topics to an Isaac Sim robot.")
parser.add_argument(
    "--robot",
    type=str,
    default="h2",
    choices=["h2", "go2", "h1", "g1", "g1_29dof"],
    help="Robot model to spawn.",
)
parser.add_argument("--domain-id", type=int, default=0, help="Cyclone DDS domain id.")
parser.add_argument("--interface", type=str, default="", help="Network interface passed to ChannelFactoryInitialize.")
parser.add_argument("--physics-dt", type=float, default=0.002, help="Physics step used by the simulator.")
parser.add_argument(
    "--physx-enable-external-forces-every-iteration",
    action="store_true",
    default=False,
    help="Enable PhysX TGS external forces every position iteration to reduce noisy velocities.",
)
parser.add_argument(
    "--physx-min-velocity-iterations",
    type=int,
    default=0,
    help="Minimum PhysX solver velocity iterations; set to 1 or 2 to improve velocity accuracy.",
)
parser.add_argument(
    "--velocity-source",
    type=str,
    default="sim",
    choices=["sim", "fd"],
    help="Joint velocity source used for the D term: sim uses PhysX joint_vel; fd uses finite-difference of joint positions.",
)
parser.add_argument(
    "--velocity-fd-lpf-alpha",
    type=float,
    default=0.0,
    help="Optional IIR low-pass for fd velocity: v = a*v_prev + (1-a)*v_raw. Set 0 to disable.",
)
parser.add_argument(
    "--velocity-fd-max-abs",
    type=float,
    default=0.0,
    help="Optional clamp for fd velocity magnitude in rad/s (0 disables). Useful to prevent rare FD spikes from exploding the D term.",
)
parser.add_argument(
    "--support-preset",
    type=str,
    default="none",
    choices=["none", "light_support", "full_hoist", "wrist_test"],
    help="Apply a reusable support preset for upper-body tests.",
)
parser.add_argument(
    "--support-constraint",
    type=str,
    default="none",
    choices=["none", "fixed", "prismatic_z"],
    help="Add a physics joint constraint between a robot rigid body and the world frame.",
)
parser.add_argument(
    "--support-constraint-body",
    type=str,
    default="pelvis",
    help="Body name used as the support constraint attachment point.",
)
parser.add_argument(
    "--support-prismatic-lower",
    type=float,
    default=-10.0,
    help="Lower limit for prismatic Z support (meters).",
)
parser.add_argument(
    "--support-prismatic-upper",
    type=float,
    default=10.0,
    help="Upper limit for prismatic Z support (meters).",
)
parser.add_argument(
    "--support-prismatic-drive-stiffness",
    type=float,
    default=8000.0,
    help="Drive stiffness for prismatic Z support.",
)
parser.add_argument(
    "--support-prismatic-drive-damping",
    type=float,
    default=1200.0,
    help="Drive damping for prismatic Z support.",
)
parser.add_argument(
    "--support-prismatic-drive-max-force",
    type=float,
    default=2e5,
    help="Drive maxForce for prismatic Z support.",
)
parser.add_argument(
    "--render-interval",
    type=int,
    default=4,
    help="Number of physics steps per rendered frame. Higher values improve GUI throughput.",
)
parser.add_argument(
    "--mode-machine",
    type=int,
    default=7,
    help="Value filled into LowState.mode_machine (used by H2 controllers to mirror into LowCmd.mode_machine).",
)
parser.add_argument(
    "--use-lowcmd-kp-kd",
    action="store_true",
    default=False,
    help="Use kp/kd from incoming LowCmd when computing joint torques (Unitree-style). If disabled, kp/kd are ignored and the simulation's implicit actuator stiffness/damping is used instead.",
)
parser.add_argument(
    "--h2-usd",
    type=str,
    default="",
    help="USD used for H2 simulation.",
)
parser.add_argument(
    "--h2-articulation-root",
    type=str,
    default="",
    help="Articulation root prim path relative to the loaded H2 USD root (starts with '/').",
)
parser.add_argument(
    "--hold-default-pose",
    action="store_true",
    default=True,
    help="Hold the startup pose with a light PD controller until the first lowcmd arrives.",
)
parser.add_argument(
    "--no-hold-default-pose",
    dest="hold_default_pose",
    action="store_false",
    help="Disable the startup pose hold controller.",
)
parser.add_argument(
    "--startup-hold-kp",
    type=float,
    default=120.0,
    help="Startup pose hold proportional gain (used until the first lowcmd arrives).",
)
parser.add_argument(
    "--startup-hold-kd",
    type=float,
    default=6.0,
    help="Startup pose hold derivative gain (used until the first lowcmd arrives).",
)
parser.add_argument(
    "--startup-hold-mode",
    type=str,
    default="asset",
    choices=["asset", "lock_current"],
    help="Startup hold controller mode: asset uses default joint stiffness/damping; lock_current uses startup-hold-kp/kd to lock current pose.",
)
parser.add_argument(
    "--startup-hold-max-tau",
    type=float,
    default=120.0,
    help="Clamp magnitude for startup hold joint torques.",
)
parser.add_argument(
    "--status-interval-steps",
    type=int,
    default=1000,
    help="Print basic bridge status every N simulation steps (0 disables).",
)
parser.add_argument(
    "--debug-lowcmd-interval-steps",
    type=int,
    default=0,
    help="Print detailed lowcmd diagnostics every N simulation steps (0 disables).",
)
parser.add_argument(
    "--debug-lowcmd-topk",
    type=int,
    default=8,
    help="Top-k joints printed in lowcmd diagnostics.",
)
parser.add_argument(
    "--debug-lowcmd-joints",
    type=str,
    default="",
    help="Comma-separated sdk joint indices to always print in diagnostics (e.g., '0,1,12').",
)
parser.add_argument(
    "--enable-hoist",
    action="store_true",
    default=False,
    help="Enable the hoist tool (disabled by default).",
)
parser.add_argument(
    "--disable-hoist",
    dest="enable_hoist",
    action="store_false",
    help="Disable the hoist tool.",
)
parser.add_argument(
    "--hoist-model",
    type=str,
    default="elastic",
    choices=["elastic", "hard"],
    help="Hoist model: elastic applies a wrench; hard teleports the robot root (debug-only, unphysical).",
)
parser.add_argument("--hoist-body", type=str, default="torso_link", help="Body name used as the hoist attachment point.")
parser.add_argument("--hoist-stiffness", type=float, default=220.0, help="Elastic hoist stiffness.")
parser.add_argument("--hoist-damping", type=float, default=90.0, help="Elastic hoist damping.")
parser.add_argument(
    "--hoist-planar-stiffness",
    type=float,
    default=80.0,
    help="Planar centering stiffness used to suppress astronaut-like drifting while hoisted.",
)
parser.add_argument(
    "--hoist-planar-damping",
    type=float,
    default=120.0,
    help="Planar damping used to suppress sideways swinging while hoisted.",
)
parser.add_argument("--hoist-rest-length", type=float, default=0.0, help="Elastic hoist cable rest length.")
parser.add_argument(
    "--hoist-height-offset",
    type=float,
    default=0.8,
    help="Initial height offset between the attachment body and the hoist anchor point.",
)
parser.add_argument(
    "--hoist-height-rate",
    type=float,
    default=0.4,
    help="Anchor height adjustment speed in meters per second while holding the keyboard key.",
)
parser.add_argument(
    "--hoist-height-step",
    type=float,
    default=0.05,
    help="Anchor height increment used by single key presses.",
)
parser.add_argument(
    "--hoist-max-force",
    type=float,
    default=450.0,
    help="Clamp magnitude for the hoist force to keep the simulation stable.",
)
parser.add_argument(
    "--hoist-preload-force",
    type=float,
    default=0.0,
    help="Constant upward preload added while the hoist cable is taut.",
)
parser.add_argument(
    "--hoist-auto-preload",
    action="store_true",
    default=False,
    help="Auto-set hoist preload based on robot weight (sum mass * |gravity|).",
)
parser.add_argument(
    "--hoist-auto-preload-scale",
    type=float,
    default=1.0,
    help="Scale applied to the robot weight when auto preload is enabled.",
)
parser.add_argument(
    "--hoist-debug-interval",
    type=int,
    default=0,
    help="Print hoist debug every N simulation steps (0 disables).",
)
parser.add_argument("--h2-linear-damping", type=float, default=0.2, help="Rigid-body linear damping applied to the H2 USD.")
parser.add_argument("--h2-angular-damping", type=float, default=0.4, help="Rigid-body angular damping applied to the H2 USD.")
parser.add_argument(
    "--enable-virtual-hand",
    action="store_true",
    default=False,
    help="Enable the virtual hand tool used to lift or stabilize a specific robot body.",
)
parser.add_argument(
    "--disable-virtual-hand",
    dest="enable_virtual_hand",
    action="store_false",
    help="Disable the virtual hand tool.",
)
parser.add_argument(
    "--hand-body",
    type=str,
    default="right_hand_link",
    help="Preferred body name used by the virtual hand tool.",
)
parser.add_argument(
    "--hand-stiffness",
    type=float,
    default=700.0,
    help="Virtual hand Cartesian stiffness.",
)
parser.add_argument(
    "--hand-damping",
    type=float,
    default=120.0,
    help="Virtual hand Cartesian damping.",
)
parser.add_argument(
    "--hand-max-force",
    type=float,
    default=1200.0,
    help="Clamp magnitude for the virtual hand force.",
)
parser.add_argument(
    "--hand-position-step",
    type=float,
    default=0.03,
    help="Position increment used by the virtual hand GUI buttons and hotkeys.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


def apply_support_preset(args):
    if args.support_preset == "none":
        return
    if args.support_preset in ("light_support", "full_hoist"):
        args.support_constraint = "prismatic_z"
        args.support_constraint_body = "pelvis"
        args.enable_hoist = False
        args.render_interval = max(args.render_interval, 6)
        args.h2_linear_damping = max(args.h2_linear_damping, 0.35)
        args.h2_angular_damping = max(args.h2_angular_damping, 0.7)
        print(
            f"[INFO] Applied support preset '{args.support_preset}': "
            "prismatic joint support to world."
        )
        return
    if args.support_preset == "wrist_test":
        args.enable_hoist = True
        args.enable_virtual_hand = False
        args.hoist_body = "pelvis"
        args.hoist_stiffness = 420.0
        args.hoist_damping = 130.0
        args.hoist_planar_stiffness = 140.0
        args.hoist_planar_damping = 200.0
        args.hoist_rest_length = 0.0
        args.hoist_height_offset = 0.72
        args.hoist_max_force = 900.0
        args.hoist_preload_force = 180.0
        args.render_interval = max(args.render_interval, 6)
        args.h2_linear_damping = max(args.h2_linear_damping, 0.4)
        args.h2_angular_damping = max(args.h2_angular_damping, 0.8)
        print(
            "[INFO] Applied support preset 'wrist_test': "
            "pelvis hoist with light preload, moderate support for upper-body tests, virtual hand disabled."
        )


apply_support_preset(args_cli)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.markers import SPHERE_MARKER_CFG, VisualizationMarkers

from isaaclab_assets.robots.unitree import G1_29DOF_CFG, H1_CFG, UNITREE_GO2_CFG
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_, unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as GoLowStateDefault
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__IMUState_ as HgImuStateDefault
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as HgLowStateDefault
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as GoLowCmd
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as GoLowState
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_ as HgImuState
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HgLowCmd
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState
from unitree_sdk2py.utils.crc import CRC

try:
    import carb
    import omni.appwindow
    import omni.usd
    import omni.ui as ui
    from omni.kit.viewport.utility import get_viewport_from_window_name
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    import omni.physx.scripts.utils as physx_utils
    from pxr import Gf, Sdf, UsdPhysics
except Exception:
    carb = None
    omni = None
    physx_utils = None
    ui = None
    get_viewport_from_window_name = None
    ViewportCameraState = None
    Gf = None
    Sdf = None
    UsdPhysics = None


TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"
TOPIC_SECONDARY_IMU = "rt/secondary_imu"


@dataclass(frozen=True)
class RobotSpec:
    name: str
    asset_cfg: object | None
    use_usd: bool
    joint_sdk_names: list[str]
    low_cmd_type: object
    low_state_type: object
    low_state_default_factory: object
    secondary_imu_type: object | None
    secondary_imu_default_factory: object | None


ROBOT_SPECS: dict[str, RobotSpec] = {
    "h2": RobotSpec(
        name="h2",
        asset_cfg=None,
        use_usd=True,
        joint_sdk_names=[
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_roll_joint",
            "left_ankle_pitch_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_roll_joint",
            "right_ankle_pitch_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            "head_pitch_joint",
            "head_yaw_joint",
        ],
        low_cmd_type=HgLowCmd,
        low_state_type=HgLowState,
        low_state_default_factory=HgLowStateDefault,
        secondary_imu_type=HgImuState,
        secondary_imu_default_factory=HgImuStateDefault,
    ),
    "go2": RobotSpec(
        name="go2",
        asset_cfg=UNITREE_GO2_CFG,
        use_usd=False,
        joint_sdk_names=[
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ],
        low_cmd_type=GoLowCmd,
        low_state_type=GoLowState,
        low_state_default_factory=GoLowStateDefault,
        secondary_imu_type=None,
        secondary_imu_default_factory=None,
    ),
    "h1": RobotSpec(
        name="h1",
        asset_cfg=H1_CFG,
        use_usd=False,
        joint_sdk_names=[
            "right_hip_roll_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "left_hip_roll_joint",
            "left_hip_pitch_joint",
            "left_knee_joint",
            "torso_joint",
            "left_hip_yaw_joint",
            "right_hip_yaw_joint",
            "",
            "left_ankle_joint",
            "right_ankle_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
        ],
        low_cmd_type=GoLowCmd,
        low_state_type=GoLowState,
        low_state_default_factory=GoLowStateDefault,
        secondary_imu_type=None,
        secondary_imu_default_factory=None,
    ),
    "g1": RobotSpec(
        name="g1_29dof",
        asset_cfg=G1_29DOF_CFG,
        use_usd=False,
        joint_sdk_names=[
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ],
        low_cmd_type=HgLowCmd,
        low_state_type=HgLowState,
        low_state_default_factory=HgLowStateDefault,
        secondary_imu_type=HgImuState,
        secondary_imu_default_factory=HgImuStateDefault,
    ),
    "g1_29dof": RobotSpec(
        name="g1_29dof",
        asset_cfg=G1_29DOF_CFG,
        use_usd=False,
        joint_sdk_names=[
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ],
        low_cmd_type=HgLowCmd,
        low_state_type=HgLowState,
        low_state_default_factory=HgLowStateDefault,
        secondary_imu_type=HgImuState,
        secondary_imu_default_factory=HgImuStateDefault,
    ),
}


class UnitreeSdk2IsaacBridge:
    """Bridge between Unitree SDK2 DDS channels and an Isaac Lab articulation."""

    def __init__(self, robot: Articulation, spec: RobotSpec):
        self.robot = robot
        self.spec = spec
        self.device = robot.device
        self.lock = threading.Lock()
        self.latest_low_cmd = None
        self.command_received = False
        self.low_cmd_count = 0
        self.low_state_publish_count = 0
        self.joint_id_by_sdk_index = self._resolve_joint_mapping()
        self.last_effort_target = torch.zeros((1, robot.num_joints), device=self.device, dtype=torch.float32)
        self.startup_hold_target_joint_pos: torch.Tensor | None = None
        self.hold_default_pose_enabled = bool(args_cli.hold_default_pose)
        self.hold_override_enabled = False
        self._position_gains_enabled = (not bool(args_cli.use_lowcmd_kp_kd)) and bool(self.hold_default_pose_enabled)
        self._fd_prev_joint_pos: torch.Tensor | None = None
        self._fd_prev_joint_vel: torch.Tensor | None = None
        self._last_joint_vel_sim: torch.Tensor | None = None
        self._last_joint_vel_used: torch.Tensor | None = None
        self.last_low_cmd_wall_time: float | None = None
        self._last_diag_wall_time: float | None = None
        self._last_diag_low_cmd_count = 0
        self.crc = CRC()

        self.low_state = spec.low_state_default_factory()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, spec.low_state_type)
        self.low_state_puber.Init()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(TOPIC_WIRELESS_CONTROLLER, WirelessController_)
        self.wireless_controller_puber.Init()

        self.secondary_imu = None
        self.secondary_imu_puber = None
        if spec.secondary_imu_default_factory is not None and spec.secondary_imu_type is not None:
            self.secondary_imu = spec.secondary_imu_default_factory()
            self.secondary_imu_puber = ChannelPublisher(TOPIC_SECONDARY_IMU, spec.secondary_imu_type)
            self.secondary_imu_puber.Init()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, spec.low_cmd_type)
        self.low_cmd_suber.Init(self.low_cmd_handler, 10)

    def _set_position_gains_enabled(self, enabled: bool):
        if args_cli.use_lowcmd_kp_kd:
            return
        enabled = bool(enabled)
        if enabled == self._position_gains_enabled:
            return
        self._position_gains_enabled = enabled
        if enabled:
            self.robot.write_joint_stiffness_to_sim(float(args_cli.startup_hold_kp))
            self.robot.write_joint_damping_to_sim(float(args_cli.startup_hold_kd))
        else:
            self.robot.write_joint_stiffness_to_sim(0.0)
            self.robot.write_joint_damping_to_sim(0.0)

    def _hold_active(self) -> bool:
        return bool(self.hold_override_enabled) or (not bool(self.command_received) and bool(self.hold_default_pose_enabled))

    def toggle_hold_override(self):
        self.hold_override_enabled = not bool(self.hold_override_enabled)
        if self.hold_override_enabled:
            self.startup_hold_target_joint_pos = self.robot.data.joint_pos.clone()
        if not self.command_received:
            self._set_position_gains_enabled(self._hold_active())
        state = "enabled" if self.hold_override_enabled else "disabled"
        note = " (overrides lowcmd)" if self.command_received else ""
        print(f"[INFO] Startup hold override {state}{note}.")

    def _resolve_joint_mapping(self) -> list[int | None]:
        joint_name_to_id = {name: index for index, name in enumerate(self.robot.joint_names)}
        mapping: list[int | None] = []
        missing_names: list[str] = []
        for name in self.spec.joint_sdk_names:
            if not name:
                mapping.append(None)
                continue
            joint_id = joint_name_to_id.get(name)
            if joint_id is None:
                mapping.append(None)
                missing_names.append(name)
            else:
                mapping.append(joint_id)
        if missing_names:
            missing = ", ".join(missing_names)
            raise RuntimeError(f"Robot joint mapping failed, missing Isaac joints: {missing}")
        return mapping

    def low_cmd_handler(self, msg):
        with self.lock:
            self.latest_low_cmd = msg
            self.command_received = True
            self.low_cmd_count += 1
            self.last_low_cmd_wall_time = time.time()
            if self.low_cmd_count == 1:
                self._set_position_gains_enabled(True)
                print("[INFO] Received first rt/lowcmd message.")

    def reset_robot(self):
        root_state = self.robot.data.default_root_state.clone()
        self.robot.write_root_pose_to_sim(root_state[:, :7])
        self.robot.write_root_velocity_to_sim(root_state[:, 7:])
        joint_pos = self.robot.data.default_joint_pos.clone()
        joint_vel = self.robot.data.default_joint_vel.clone()
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel)
        if args_cli.use_lowcmd_kp_kd:
            self.robot.write_joint_stiffness_to_sim(0.0)
            self.robot.write_joint_damping_to_sim(0.0)
        else:
            self._set_position_gains_enabled(self.command_received or self._hold_active())
        self.robot.reset()
        self.last_effort_target.zero_()
        self.startup_hold_target_joint_pos = None
        self._fd_prev_joint_pos = None
        self._fd_prev_joint_vel = None
        self._last_joint_vel_sim = None
        self._last_joint_vel_used = None

    def _estimate_joint_vel(self, sim_dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        joint_pos = self.robot.data.joint_pos[0]
        joint_vel_sim = self.robot.data.joint_vel[0]

        if args_cli.velocity_source != "fd":
            return joint_vel_sim, joint_vel_sim

        if self._fd_prev_joint_pos is None:
            self._fd_prev_joint_pos = joint_pos.clone()
            joint_vel_fd = torch.zeros_like(joint_pos)
            self._fd_prev_joint_vel = joint_vel_fd.clone()
            return joint_vel_fd, joint_vel_sim

        joint_vel_raw = (joint_pos - self._fd_prev_joint_pos) / float(sim_dt)
        alpha = float(args_cli.velocity_fd_lpf_alpha)
        if alpha > 0.0 and self._fd_prev_joint_vel is not None:
            alpha = max(0.0, min(0.999, alpha))
            joint_vel_fd = alpha * self._fd_prev_joint_vel + (1.0 - alpha) * joint_vel_raw
        else:
            joint_vel_fd = joint_vel_raw

        max_abs = float(args_cli.velocity_fd_max_abs)
        if max_abs > 0.0:
            joint_vel_fd = torch.clamp(joint_vel_fd, min=-max_abs, max=max_abs)

        self._fd_prev_joint_pos = joint_pos.clone()
        self._fd_prev_joint_vel = joint_vel_fd.clone()
        return joint_vel_fd, joint_vel_sim

    def _startup_hold_command(self, sim_dt: float) -> torch.Tensor:
        joint_pos = self.robot.data.joint_pos
        joint_vel_used, joint_vel_sim = self._estimate_joint_vel(sim_dt)
        joint_vel = joint_vel_used.unsqueeze(0)
        self._last_joint_vel_sim = joint_vel_sim.clone()
        self._last_joint_vel_used = joint_vel_used.clone()
        max_tau = float(args_cli.startup_hold_max_tau)
        mode = "lock_current" if self.hold_override_enabled else str(args_cli.startup_hold_mode)
        if mode == "asset":
            default_joint_pos = self.robot.data.default_joint_pos
            default_kp = self.robot.data.default_joint_stiffness
            default_kd = self.robot.data.default_joint_damping
            effort = default_kp * (default_joint_pos - joint_pos) - default_kd * joint_vel
        else:
            if self.startup_hold_target_joint_pos is None:
                self.startup_hold_target_joint_pos = joint_pos.clone()
            target_joint_pos = self.startup_hold_target_joint_pos
            kp = float(args_cli.startup_hold_kp)
            kd = float(args_cli.startup_hold_kd)
            effort = kp * (target_joint_pos - joint_pos) - kd * joint_vel
        return torch.clamp(effort, min=-max_tau, max=max_tau)

    def apply_low_cmd(self, sim_dt: float):
        if not self.command_received:
            hold_active = self._hold_active()
            self._set_position_gains_enabled(hold_active)
            if hold_active:
                if args_cli.use_lowcmd_kp_kd:
                    self.last_effort_target = self._startup_hold_command(sim_dt)
                    self.robot.set_joint_effort_target(self.last_effort_target)
                else:
                    if self.startup_hold_target_joint_pos is None:
                        self.startup_hold_target_joint_pos = self.robot.data.joint_pos.clone()
                    pos_target = self.startup_hold_target_joint_pos.to(torch.float32)
                    vel_target = torch.zeros((1, self.robot.num_joints), device=self.device, dtype=torch.float32)
                    effort_target = torch.zeros((1, self.robot.num_joints), device=self.device, dtype=torch.float32)
                    self.robot.set_joint_position_target(pos_target)
                    self.robot.set_joint_velocity_target(vel_target)
                    self.last_effort_target = effort_target
                    self.robot.set_joint_effort_target(effort_target)
            else:
                self.last_effort_target.zero_()
                self.robot.set_joint_effort_target(self.last_effort_target)
            return

        with self.lock:
            cmd = self.latest_low_cmd

        if self.hold_override_enabled:
            if args_cli.use_lowcmd_kp_kd:
                self.last_effort_target = self._startup_hold_command(sim_dt)
                self.robot.set_joint_effort_target(self.last_effort_target)
            else:
                if self.startup_hold_target_joint_pos is None:
                    self.startup_hold_target_joint_pos = self.robot.data.joint_pos.clone()
                pos_target = self.startup_hold_target_joint_pos.to(torch.float32)
                vel_target = torch.zeros((1, self.robot.num_joints), device=self.device, dtype=torch.float32)
                effort_target = torch.zeros((1, self.robot.num_joints), device=self.device, dtype=torch.float32)
                self.robot.set_joint_position_target(pos_target)
                self.robot.set_joint_velocity_target(vel_target)
                self.last_effort_target = effort_target
                self.robot.set_joint_effort_target(effort_target)
            return

        joint_pos = self.robot.data.joint_pos[0]
        joint_vel_used, joint_vel_sim = self._estimate_joint_vel(sim_dt)
        self._last_joint_vel_sim = joint_vel_sim.clone()
        self._last_joint_vel_used = joint_vel_used.clone()
        effort_target = torch.zeros((1, self.robot.num_joints), device=self.device, dtype=torch.float32)

        if not args_cli.use_lowcmd_kp_kd:
            pos_target = joint_pos.clone().unsqueeze(0).to(torch.float32)
            vel_target = torch.zeros((1, self.robot.num_joints), device=self.device, dtype=torch.float32)
            for sdk_index, joint_id in enumerate(self.joint_id_by_sdk_index):
                if joint_id is None:
                    continue
                motor_cmd = cmd.motor_cmd[sdk_index]
                pos_target[0, joint_id] = float(motor_cmd.q)
                vel_target[0, joint_id] = float(motor_cmd.dq)
                effort_target[0, joint_id] = float(motor_cmd.tau)
            self.robot.set_joint_position_target(pos_target)
            self.robot.set_joint_velocity_target(vel_target)
            self.last_effort_target = effort_target
            self.robot.set_joint_effort_target(effort_target)
            return

        for sdk_index, joint_id in enumerate(self.joint_id_by_sdk_index):
            if joint_id is None:
                continue
            motor_cmd = cmd.motor_cmd[sdk_index]
            effort = (
                motor_cmd.tau
                + motor_cmd.kp * (motor_cmd.q - float(joint_pos[joint_id]))
                + motor_cmd.kd * (motor_cmd.dq - float(joint_vel_used[joint_id]))
            )
            effort_target[0, joint_id] = effort

        self.last_effort_target = effort_target
        self.robot.set_joint_effort_target(effort_target)

    def lowcmd_diagnostics(self, sim_dt: float, step_count: int) -> str:
        if not self.command_received:
            return "[DIAG] lowcmd: not received yet."

        with self.lock:
            cmd = self.latest_low_cmd
            low_cmd_count = self.low_cmd_count
            last_low_cmd_wall_time = self.last_low_cmd_wall_time

        if cmd is None:
            return "[DIAG] lowcmd: received flag set but latest cmd is None."

        now = time.time()
        if self._last_diag_wall_time is None:
            self._last_diag_wall_time = now
            self._last_diag_low_cmd_count = low_cmd_count
        dt_wall = max(1e-6, now - self._last_diag_wall_time)
        recv_rate_hz = float(low_cmd_count - self._last_diag_low_cmd_count) / dt_wall
        self._last_diag_wall_time = now
        self._last_diag_low_cmd_count = low_cmd_count

        age_ms = None
        if last_low_cmd_wall_time is not None:
            age_ms = (now - last_low_cmd_wall_time) * 1000.0

        joint_pos = self.robot.data.joint_pos[0]
        joint_vel = self._last_joint_vel_used if self._last_joint_vel_used is not None else self.robot.data.joint_vel[0]
        joint_vel_sim = self._last_joint_vel_sim if self._last_joint_vel_sim is not None else self.robot.data.joint_vel[0]

        max_abs_q_err = 0.0
        max_abs_dq_err = 0.0
        max_abs_tau_ff = 0.0
        max_abs_effort = 0.0
        max_abs_dq_sim = 0.0
        max_abs_dq_used = 0.0
        max_kp = 0.0
        max_kd = 0.0

        by_effort: list[tuple[float, int, int, float, float, float, float, float, float, float, float, float, float]] = []

        if args_cli.use_lowcmd_kp_kd:
            kp_used_const = None
            kd_used_const = None
            kp_kd_mode = "rx"
        else:
            kp_used_const = float(args_cli.startup_hold_kp)
            kd_used_const = float(args_cli.startup_hold_kd)
            kp_kd_mode = "sim"

        for sdk_index, joint_id in enumerate(self.joint_id_by_sdk_index):
            if joint_id is None:
                continue
            motor_cmd = cmd.motor_cmd[sdk_index]
            q = float(joint_pos[joint_id])
            dq = float(joint_vel[joint_id])
            q_des = float(motor_cmd.q)
            dq_des = float(motor_cmd.dq)
            tau_ff = float(motor_cmd.tau)
            kp = float(motor_cmd.kp) if kp_used_const is None else float(kp_used_const)
            kd = float(motor_cmd.kd) if kd_used_const is None else float(kd_used_const)
            q_err = q_des - q
            dq_err = dq_des - dq
            effort = tau_ff + kp * q_err + kd * dq_err

            max_abs_q_err = max(max_abs_q_err, abs(q_err))
            max_abs_dq_err = max(max_abs_dq_err, abs(dq_err))
            max_abs_tau_ff = max(max_abs_tau_ff, abs(tau_ff))
            max_abs_effort = max(max_abs_effort, abs(effort))
            max_abs_dq_sim = max(max_abs_dq_sim, abs(float(joint_vel_sim[joint_id])))
            max_abs_dq_used = max(max_abs_dq_used, abs(dq))
            max_kp = max(max_kp, abs(kp))
            max_kd = max(max_kd, abs(kd))
            by_effort.append((abs(effort), sdk_index, joint_id, effort, q_err, dq_err, kp, kd, tau_ff, q_des, dq_des, q, dq))

        topk = max(0, int(args_cli.debug_lowcmd_topk))
        by_effort.sort(key=lambda x: x[0], reverse=True)
        by_effort = by_effort[:topk]

        forced_sdk_indices: list[int] = []
        if args_cli.debug_lowcmd_joints:
            try:
                forced_sdk_indices = [
                    int(x.strip())
                    for x in args_cli.debug_lowcmd_joints.split(",")
                    if x.strip()
                ]
            except ValueError:
                forced_sdk_indices = []

        forced_lines: list[str] = []
        forced_set = set(forced_sdk_indices)
        if forced_set:
            for sdk_index in sorted(forced_set):
                if sdk_index < 0 or sdk_index >= len(self.joint_id_by_sdk_index):
                    continue
                joint_id = self.joint_id_by_sdk_index[sdk_index]
                if joint_id is None:
                    continue
                motor_cmd = cmd.motor_cmd[sdk_index]
                q = float(joint_pos[joint_id])
                dq = float(joint_vel[joint_id])
                dq_sim = float(joint_vel_sim[joint_id])
                q_des = float(motor_cmd.q)
                dq_des = float(motor_cmd.dq)
                tau_ff = float(motor_cmd.tau)
                kp = float(motor_cmd.kp) if kp_used_const is None else float(kp_used_const)
                kd = float(motor_cmd.kd) if kd_used_const is None else float(kd_used_const)
                q_err = q_des - q
                dq_err = dq_des - dq
                effort = tau_ff + kp * q_err + kd * dq_err
                name = self.spec.joint_sdk_names[sdk_index] if sdk_index < len(self.spec.joint_sdk_names) else "<unknown>"
                forced_lines.append(
                    f"[DIAG] sdk[{sdk_index:02d}] -> isaac[{joint_id:02d}] {name}: "
                    f"q={q:+.4f} q_des={q_des:+.4f} q_err={q_err:+.4f} | "
                    f"dq={dq:+.4f} dq_sim={dq_sim:+.4f} dq_des={dq_des:+.4f} dq_err={dq_err:+.4f} | "
                    f"kp={kp:.1f} kd={kd:.1f} tau_ff={tau_ff:+.2f} effort={effort:+.2f}"
                )

        header = (
            "[DIAG] "
            f"step={step_count} sim_dt={sim_dt:.4f}s "
            f"lowcmd_rate={recv_rate_hz:.1f}Hz "
            + (f"lowcmd_age={age_ms:.1f}ms " if age_ms is not None else "lowcmd_age=<na> ")
            + f"kp_kd={kp_kd_mode} "
            + f"max|q_err|={max_abs_q_err:.4f} rad "
            f"max|dq_err|={max_abs_dq_err:.4f} rad/s "
            f"max|dq_sim|={max_abs_dq_sim:.4f} rad/s "
            f"max|dq_used|={max_abs_dq_used:.4f} rad/s "
            f"max|tau_ff|={max_abs_tau_ff:.2f} "
            f"max|effort|={max_abs_effort:.2f} "
            f"max|kp|={max_kp:.1f} max|kd|={max_kd:.1f}"
        )

        lines = [header]
        if forced_lines:
            lines.extend(forced_lines)

        for _abs_effort, sdk_index, joint_id, effort, q_err, dq_err, kp, kd, tau_ff, q_des, dq_des, q, dq in by_effort:
            name = self.spec.joint_sdk_names[sdk_index] if sdk_index < len(self.spec.joint_sdk_names) else "<unknown>"
            lines.append(
                f"[DIAG] top|effort| sdk[{sdk_index:02d}] -> isaac[{joint_id:02d}] {name}: "
                f"effort={effort:+.2f} tau_ff={tau_ff:+.2f} kp={kp:.1f} kd={kd:.1f} "
                f"q_err={q_err:+.4f} dq_err={dq_err:+.4f}"
            )

        return "\n".join(lines)

    @staticmethod
    def _quat_wxyz_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(roll), float(pitch), float(yaw)

    def publish_low_state(self):
        joint_pos = self.robot.data.joint_pos[0]
        joint_vel = self.robot.data.joint_vel[0]
        root_quat_w = self.robot.data.root_quat_w[0]
        root_ang_vel_b = self.robot.data.root_ang_vel_b[0]
        root_lin_acc_w = self.robot.data.body_lin_acc_w[0, 0].unsqueeze(0)
        root_lin_acc_b = math_utils.quat_apply_inverse(root_quat_w.unsqueeze(0), root_lin_acc_w)[0]

        self.low_state.mode_machine = int(args_cli.mode_machine)

        for sdk_index, joint_id in enumerate(self.joint_id_by_sdk_index):
            if joint_id is None:
                continue
            self.low_state.motor_state[sdk_index].q = float(joint_pos[joint_id])
            self.low_state.motor_state[sdk_index].dq = float(joint_vel[joint_id])
            self.low_state.motor_state[sdk_index].tau_est = float(self.last_effort_target[0, joint_id])

        self.low_state.imu_state.quaternion[0] = float(root_quat_w[0])
        self.low_state.imu_state.quaternion[1] = float(root_quat_w[1])
        self.low_state.imu_state.quaternion[2] = float(root_quat_w[2])
        self.low_state.imu_state.quaternion[3] = float(root_quat_w[3])

        r, p, y = self._quat_wxyz_to_rpy(
            float(root_quat_w[0]), float(root_quat_w[1]), float(root_quat_w[2]), float(root_quat_w[3])
        )
        if hasattr(self.low_state.imu_state, "rpy"):
            self.low_state.imu_state.rpy[0] = r
            self.low_state.imu_state.rpy[1] = p
            self.low_state.imu_state.rpy[2] = y

        self.low_state.imu_state.gyroscope[0] = float(root_ang_vel_b[0])
        self.low_state.imu_state.gyroscope[1] = float(root_ang_vel_b[1])
        self.low_state.imu_state.gyroscope[2] = float(root_ang_vel_b[2])

        self.low_state.imu_state.accelerometer[0] = float(root_lin_acc_b[0])
        self.low_state.imu_state.accelerometer[1] = float(root_lin_acc_b[1])
        self.low_state.imu_state.accelerometer[2] = float(root_lin_acc_b[2])

        self.low_state.crc = self.crc.Crc(self.low_state)
        self.low_state_puber.Write(self.low_state)
        self.low_state_publish_count += 1

    def publish_high_state(self):
        root_pos_w = self.robot.data.root_pos_w[0]
        root_lin_vel_w = self.robot.data.root_lin_vel_w[0]

        self.high_state.position[0] = float(root_pos_w[0])
        self.high_state.position[1] = float(root_pos_w[1])
        self.high_state.position[2] = float(root_pos_w[2])

        self.high_state.velocity[0] = float(root_lin_vel_w[0])
        self.high_state.velocity[1] = float(root_lin_vel_w[1])
        self.high_state.velocity[2] = float(root_lin_vel_w[2])

        self.high_state_puber.Write(self.high_state)

    def publish_wireless_controller(self):
        self.wireless_controller.keys = 0
        self.wireless_controller.lx = 0.0
        self.wireless_controller.ly = 0.0
        self.wireless_controller.rx = 0.0
        self.wireless_controller.ry = 0.0
        self.wireless_controller_puber.Write(self.wireless_controller)

    def publish_secondary_imu(self):
        if self.secondary_imu is None or self.secondary_imu_puber is None:
            return
        root_quat_w = self.robot.data.root_quat_w[0]
        root_ang_vel_b = self.robot.data.root_ang_vel_b[0]
        root_lin_acc_w = self.robot.data.body_lin_acc_w[0, 0].unsqueeze(0)
        root_lin_acc_b = math_utils.quat_apply_inverse(root_quat_w.unsqueeze(0), root_lin_acc_w)[0]

        self.secondary_imu.quaternion[0] = float(root_quat_w[0])
        self.secondary_imu.quaternion[1] = float(root_quat_w[1])
        self.secondary_imu.quaternion[2] = float(root_quat_w[2])
        self.secondary_imu.quaternion[3] = float(root_quat_w[3])
        r, p, y = self._quat_wxyz_to_rpy(
            float(root_quat_w[0]), float(root_quat_w[1]), float(root_quat_w[2]), float(root_quat_w[3])
        )
        self.secondary_imu.rpy[0] = r
        self.secondary_imu.rpy[1] = p
        self.secondary_imu.rpy[2] = y
        self.secondary_imu.gyroscope[0] = float(root_ang_vel_b[0])
        self.secondary_imu.gyroscope[1] = float(root_ang_vel_b[1])
        self.secondary_imu.gyroscope[2] = float(root_ang_vel_b[2])
        self.secondary_imu.accelerometer[0] = float(root_lin_acc_b[0])
        self.secondary_imu.accelerometer[1] = float(root_lin_acc_b[1])
        self.secondary_imu.accelerometer[2] = float(root_lin_acc_b[2])
        self.secondary_imu_puber.Write(self.secondary_imu)

    def publish_all(self):
        self.publish_low_state()
        self.publish_high_state()
        self.publish_wireless_controller()
        self.publish_secondary_imu()

    def print_mapping(self):
        print("[INFO] Isaac joint mapping:")
        for sdk_index, joint_name in enumerate(self.spec.joint_sdk_names):
            joint_id = self.joint_id_by_sdk_index[sdk_index]
            if joint_id is None:
                print(f"  sdk[{sdk_index:02d}] -> <unused>")
            else:
                print(f"  sdk[{sdk_index:02d}] -> isaac[{joint_id:02d}] {joint_name}")


def _resolve_body_with_fallback(robot: Articulation, preferred_name: str, fallback_names: tuple[str, ...]) -> tuple[int, str]:
    body_ids, body_names = robot.find_bodies(preferred_name)
    if body_ids:
        return body_ids[0], body_names[0]
    for fallback_name in fallback_names:
        body_ids, body_names = robot.find_bodies(fallback_name)
        if body_ids:
            print(f"[WARN] Body '{preferred_name}' not found, fallback to '{body_names[0]}'.")
            return body_ids[0], body_names[0]
    raise RuntimeError(f"Cannot resolve body '{preferred_name}' from robot body names.")


class NullTool:
    def step(self, _sim_dt: float):
        return


class SupportConstraintController:
    def __init__(self, robot: Articulation, joint_prim, mode: str, body_id: int, body_name: str, target_z: float = 0.0):
        self.robot = robot
        self.device = robot.device
        self.mode = mode
        self.body_id = int(body_id)
        self.body_name = body_name
        self.enabled = True
        self.height_rate = float(args_cli.hoist_height_rate)
        self.height_step = float(args_cli.hoist_height_step)
        self.vertical_input = 0.0

        self._joint_prim = joint_prim
        self._joint_api = UsdPhysics.Joint(joint_prim)
        self._joint_enabled_attr = self._joint_api.GetJointEnabledAttr()

        self.target_z = float(target_z)
        self._drive_api = None
        self._target_attr = None

        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        self._window = None
        self._ui_models: dict[str, object] = {}
        self._ui_labels: dict[str, object] = {}

        self._setup_prismatic_drive()
        self._setup_keyboard()
        self._setup_ui_window()
        self.print_help()

    def _setup_prismatic_drive(self):
        if self.mode != "prismatic_z":
            return
        self._drive_api = UsdPhysics.DriveAPI.Apply(self._joint_prim, "linear")
        try:
            self._drive_api.CreateTypeAttr().Set("force")
        except Exception:
            pass
        try:
            self._drive_api.CreateStiffnessAttr().Set(float(args_cli.support_prismatic_drive_stiffness))
            self._drive_api.CreateDampingAttr().Set(float(args_cli.support_prismatic_drive_damping))
        except Exception:
            pass
        try:
            self._drive_api.CreateMaxForceAttr().Set(float(args_cli.support_prismatic_drive_max_force))
        except Exception:
            pass
        try:
            self._target_attr = self._drive_api.CreateTargetPositionAttr()
        except Exception:
            self._target_attr = None
        if self._target_attr is None:
            attr = self._joint_prim.GetAttribute("physics:drive:linear:targetPosition")
            if not attr:
                attr = self._joint_prim.CreateAttribute("physics:drive:linear:targetPosition", Sdf.ValueTypeNames.Float)
            self._target_attr = attr
        self._target_attr.Set(float(self.target_z))

    def _setup_keyboard(self):
        if carb is None or omni is None or args_cli.headless:
            return
        try:
            self._input = carb.input.acquire_input_interface()
            self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
        except Exception as exc:
            print(f"[WARN] Support constraint keyboard control unavailable: {exc}")
            self._input = None
            self._keyboard = None
            self._sub_keyboard = None

    def _setup_ui_window(self):
        if ui is None or args_cli.headless:
            return

        def add_float_control(label: str, key: str, value: float, callback, step: float | None = None):
            with ui.HStack(height=24):
                ui.Label(label, width=130)
                field = ui.FloatDrag(width=ui.Fraction(1))
                field.model.set_value(value)
                field.model.add_value_changed_fn(lambda model: callback(float(model.as_float)))
                self._ui_models[key] = field.model
                if step is not None:
                    ui.Button("+", width=32, clicked_fn=lambda s=step, k=key: self._nudge_model(k, s))
                    ui.Button("-", width=32, clicked_fn=lambda s=step, k=key: self._nudge_model(k, -s))

        title = "Support Constraint" if self.mode != "prismatic_z" else "Support Prismatic Z"
        self._window = ui.Window(
            title,
            width=360,
            height=230,
            visible=True,
            dock_preference=ui.DockPreference.RIGHT_TOP,
        )
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                self._ui_labels["title"] = ui.Label(f"Constraint body: {self.body_name} ({self.mode})")

                with ui.HStack(height=24):
                    ui.Label("Enable support", width=130)
                    enabled_model = ui.SimpleBoolModel()
                    enabled_model.set_value(self.enabled)
                    enabled_model.add_value_changed_fn(lambda model: self.set_enabled(bool(model.as_bool)))
                    ui.CheckBox(model=enabled_model, width=24)
                    self._ui_models["enabled"] = enabled_model

                if self.mode == "prismatic_z":
                    add_float_control("Target Z", "target_z", self.target_z, self.set_target_z, step=self.height_step)
                    self._ui_labels["hint"] = ui.Label("Hotkeys: T/G move, B/N step, Y toggle support")
                else:
                    self._ui_labels["hint"] = ui.Label("Hotkey: Y toggle support")

    def _nudge_model(self, key: str, delta: float):
        model = self._ui_models.get(key)
        if model is None:
            return
        model.set_value(float(model.as_float) + delta)

    def _sync_ui_models(self):
        if not self._ui_models:
            return
        if "enabled" in self._ui_models:
            self._ui_models["enabled"].set_value(self.enabled)
        if "target_z" in self._ui_models:
            self._ui_models["target_z"].set_value(float(self.target_z))

    def _on_keyboard_event(self, event):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "Y":
                self.set_enabled(not self.enabled)
            elif self.mode == "prismatic_z":
                if event.input.name == "T":
                    self.vertical_input = 1.0
                elif event.input.name == "G":
                    self.vertical_input = -1.0
                elif event.input.name == "B":
                    self.set_target_z(self.target_z + self.height_step)
                elif event.input.name == "N":
                    self.set_target_z(self.target_z - self.height_step)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in ("T", "G"):
                self.vertical_input = 0.0
        return True

    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)
        if self._joint_enabled_attr:
            self._joint_enabled_attr.Set(self.enabled)
        state = "enabled" if self.enabled else "disabled"
        print(f"[INFO] Support constraint {state}.")
        self._sync_ui_models()

    def set_target_z(self, value: float):
        if self.mode != "prismatic_z":
            return
        self.target_z = float(value)
        if self._target_attr is not None:
            self._target_attr.Set(float(self.target_z))
        self._sync_ui_models()

    def print_help(self):
        print("[INFO] Support constraint controls:")
        print(f"  mode        : {self.mode}")
        print(f"  attach body : {self.body_name}")
        print("  press Y     : toggle support on/off")
        if self.mode == "prismatic_z":
            print("  hold T/G    : move target along Z continuously")
            print("  press B/N   : move target along Z by one step")
            print("  GUI         : use the support constraint window to edit Target Z")

    def step(self, sim_dt: float):
        if not self.enabled:
            return
        if self.mode != "prismatic_z":
            return
        if self.vertical_input != 0.0:
            self.set_target_z(self.target_z + self.vertical_input * self.height_rate * sim_dt)


def create_support_constraint(robot: Articulation):
    if args_cli.support_constraint == "none":
        return None
    if physx_utils is None or omni is None or UsdPhysics is None:
        print("[WARN] Support constraint requires Isaac Sim USD/PhysX modules; skipping.")
        return None

    stage = omni.usd.get_context().get_stage()
    body_id, body_name = _resolve_body_with_fallback(
        robot,
        args_cli.support_constraint_body,
        ("pelvis", "torso_link", "base", "base_link"),
    )
    link_paths = robot.root_physx_view.link_paths[0]
    if body_id >= len(link_paths):
        raise RuntimeError(f"Support constraint body id {body_id} is out of range for link paths.")
    body_prim_path = link_paths[body_id]
    body_prim = stage.GetPrimAtPath(body_prim_path)
    if not body_prim.IsValid():
        raise RuntimeError(f"Support constraint prim '{body_prim_path}' is invalid.")

    joint_type = "Fixed" if args_cli.support_constraint == "fixed" else "Prismatic"
    joint_prim = physx_utils.createJoint(stage=stage, joint_type=joint_type, from_prim=None, to_prim=body_prim)
    if args_cli.support_constraint == "prismatic_z":
        pj = UsdPhysics.PrismaticJoint(joint_prim)
        pj.CreateAxisAttr().Set("Z")
        pj.CreateLowerLimitAttr().Set(float(args_cli.support_prismatic_lower))
        pj.CreateUpperLimitAttr().Set(float(args_cli.support_prismatic_upper))

    print(
        "[INFO] Created support constraint: "
        f"type={args_cli.support_constraint}, body={body_name}, body_prim={body_prim_path}, joint={joint_prim.GetPath()}"
    )
    target_z = 0.0
    if args_cli.support_constraint == "prismatic_z":
        target_z = float(robot.data.body_pos_w[0, body_id, 2])
    return SupportConstraintController(robot, joint_prim, args_cli.support_constraint, body_id, body_name, target_z=target_z)


def _build_virtual_hand_body_candidates(robot: Articulation, preferred_name: str) -> list[str]:
    candidates: list[str] = []
    preferred_tokens = (
        preferred_name,
        "right_hand",
        "left_hand",
        "right_forearm",
        "left_forearm",
        "right_elbow",
        "left_elbow",
        "right_shoulder",
        "left_shoulder",
        "torso",
        "pelvis",
        "head",
    )
    lower_to_name = {name.lower(): name for name in robot.body_names}
    for token in preferred_tokens:
        token_lower = token.lower()
        for name_lower, name in lower_to_name.items():
            if token_lower and token_lower in name_lower and name not in candidates:
                candidates.append(name)
    if not candidates:
        candidates.append(robot.body_names[0])
    return candidates


class ElasticHoist:
    """Mujoco-like elastic support tool with runtime height adjustment."""

    def __init__(self, robot: Articulation, sim_cfg: sim_utils.SimulationCfg | None = None):
        self.robot = robot
        self.device = robot.device
        self.enabled = args_cli.enable_hoist
        self.stiffness = float(args_cli.hoist_stiffness)
        self.damping = float(args_cli.hoist_damping)
        self.planar_stiffness = float(args_cli.hoist_planar_stiffness)
        self.planar_damping = float(args_cli.hoist_planar_damping)
        self.rest_length = float(args_cli.hoist_rest_length)
        self.height_rate = float(args_cli.hoist_height_rate)
        self.height_step = float(args_cli.hoist_height_step)
        self.max_force = float(args_cli.hoist_max_force)
        self.preload_force = float(args_cli.hoist_preload_force)
        self.auto_preload = bool(args_cli.hoist_auto_preload)
        self.auto_preload_scale = float(args_cli.hoist_auto_preload_scale)
        self.debug_interval = int(args_cli.hoist_debug_interval)
        self._debug_step = 0
        self.vertical_input = 0.0
        self.total_mass = None
        self.gravity_mag = None
        self.weight_force = None

        self.body_id, self.body_name = self._resolve_body(args_cli.hoist_body)
        body_pos = self.robot.data.body_pos_w[0, self.body_id].clone()
        self.anchor_point = body_pos.clone()
        self.anchor_point[2] += float(args_cli.hoist_height_offset)
        self.current_force = torch.zeros(3, device=self.device, dtype=torch.float32)

        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        self._mouse = None
        self._sub_mouse = None
        self._viewport = None
        self.drag_mode = False
        self.drag_xy_active = False
        self.drag_z_active = False
        self._mouse_prev = None
        self._drag_scale_xy = 0.003
        self._drag_scale_z = 0.004
        self._window = None
        self._ui_models: dict[str, object] = {}
        self._ui_labels: dict[str, object] = {}
        self.anchor_visualizer = None
        self.attach_visualizer = None
        self._setup_keyboard()
        self._setup_visualizers()
        self._configure_preload(sim_cfg)
        self._setup_ui_window()
        self.print_help()

    def _resolve_body(self, name: str) -> tuple[int, str]:
        return _resolve_body_with_fallback(self.robot, name, ("torso_link", "pelvis", "base", "base_link"))

    def _setup_keyboard(self):
        if carb is None or omni is None or args_cli.headless:
            return
        try:
            self._input = carb.input.acquire_input_interface()
            self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
            self._mouse = omni.appwindow.get_default_app_window().get_mouse()
            self._sub_mouse = self._input.subscribe_to_mouse_events(self._mouse, self._on_mouse_event)
            self._viewport = get_viewport_from_window_name("Viewport") if get_viewport_from_window_name else None
        except Exception as exc:
            print(f"[WARN] Hoist keyboard control unavailable: {exc}")
            self._input = None
            self._keyboard = None
            self._sub_keyboard = None
            self._mouse = None
            self._sub_mouse = None
            self._viewport = None

    def _setup_visualizers(self):
        anchor_cfg = SPHERE_MARKER_CFG.copy()
        anchor_cfg.prim_path = "/Visuals/Hoist/anchor"
        anchor_cfg.markers["sphere"].radius = 0.08
        anchor_cfg.markers["sphere"].visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.25, 0.25))
        self.anchor_visualizer = VisualizationMarkers(anchor_cfg)

        attach_cfg = SPHERE_MARKER_CFG.copy()
        attach_cfg.prim_path = "/Visuals/Hoist/attach"
        attach_cfg.markers["sphere"].radius = 0.06
        attach_cfg.markers["sphere"].visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.8, 1.0))
        self.attach_visualizer = VisualizationMarkers(attach_cfg)

    def _configure_preload(self, sim_cfg: sim_utils.SimulationCfg | None):
        try:
            masses = self.robot.root_physx_view.get_masses()
            self.total_mass = float(masses[0].sum())
        except Exception:
            self.total_mass = None

        try:
            if sim_cfg is not None:
                gravity_w = torch.tensor(sim_cfg.gravity, device=self.device, dtype=torch.float32)
            else:
                gravity_w = torch.tensor((0.0, 0.0, -9.81), device=self.device, dtype=torch.float32)
            self.gravity_mag = float(torch.linalg.norm(gravity_w))
        except Exception:
            self.gravity_mag = None

        if self.total_mass is not None and self.gravity_mag is not None:
            self.weight_force = self.total_mass * self.gravity_mag
            if self.auto_preload:
                target_preload = max(0.0, self.weight_force * max(self.auto_preload_scale, 0.0))
                self.preload_force = max(self.preload_force, target_preload)
                if self.max_force > 0.0:
                    self.max_force = max(self.max_force, min(self.preload_force * 1.6, 6500.0))
            print(
                "[INFO] Hoist preload config: "
                f"mass={self.total_mass:.2f}kg, g={self.gravity_mag:.3f}m/s^2, "
                f"weight={self.weight_force:.1f}N, preload={self.preload_force:.1f}N, max={self.max_force:.1f}N"
            )
        else:
            print(f"[INFO] Hoist preload config: preload={self.preload_force:.1f}N, max={self.max_force:.1f}N")

    def _setup_ui_window(self):
        if ui is None or args_cli.headless:
            return

        def add_float_control(label: str, key: str, value: float, callback, step_label: str | None = None, step=None):
            with ui.HStack(height=24):
                ui.Label(label, width=130)
                field = ui.FloatDrag(width=ui.Fraction(1))
                field.model.set_value(value)
                field.model.add_value_changed_fn(lambda model: callback(float(model.as_float)))
                self._ui_models[key] = field.model
                if step is not None:
                    ui.Button(
                        step_label or "+",
                        width=32,
                        clicked_fn=lambda s=step, k=key: self._nudge_model(k, s),
                    )

        self._window = ui.Window(
            "Hoist Control",
            width=360,
            height=430,
            visible=True,
            dock_preference=ui.DockPreference.RIGHT_TOP,
        )
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                self._ui_labels["title"] = ui.Label(f"H2 hoist attached to: {self.body_name}")
                self._ui_labels["mass"] = ui.Label("")

                with ui.HStack(height=24):
                    ui.Label("Enable hoist", width=130)
                    enabled_model = ui.SimpleBoolModel()
                    enabled_model.set_value(self.enabled)
                    enabled_model.add_value_changed_fn(lambda model: self.set_enabled(bool(model.as_bool)))
                    ui.CheckBox(model=enabled_model, width=24)
                    self._ui_models["enabled"] = enabled_model

                with ui.HStack(height=24):
                    ui.Label("Drag mode", width=130)
                    drag_model = ui.SimpleBoolModel()
                    drag_model.set_value(self.drag_mode)
                    drag_model.add_value_changed_fn(lambda model: self.set_drag_mode(bool(model.as_bool)))
                    ui.CheckBox(model=drag_model, width=24)
                    self._ui_models["drag_mode"] = drag_model

                with ui.HStack(height=24):
                    ui.Button("Recenter XY", clicked_fn=self.recenter_anchor_xy)
                    ui.Button("Body +0.8m", clicked_fn=lambda: self.place_anchor_above_body(0.8))
                    ui.Button("Body +1.2m", clicked_fn=lambda: self.place_anchor_above_body(1.2))

                with ui.HStack(height=24):
                    ui.Button("X-", width=40, clicked_fn=lambda: self.nudge_anchor(0, -0.05))
                    ui.Button("X+", width=40, clicked_fn=lambda: self.nudge_anchor(0, 0.05))
                    ui.Button("Y-", width=40, clicked_fn=lambda: self.nudge_anchor(1, -0.05))
                    ui.Button("Y+", width=40, clicked_fn=lambda: self.nudge_anchor(1, 0.05))
                    ui.Button("Z-", width=40, clicked_fn=lambda: self.nudge_anchor(2, -0.05))
                    ui.Button("Z+", width=40, clicked_fn=lambda: self.nudge_anchor(2, 0.05))

                add_float_control("Anchor X", "anchor_x", float(self.anchor_point[0]), lambda v: self.set_anchor_axis(0, v))
                add_float_control("Anchor Y", "anchor_y", float(self.anchor_point[1]), lambda v: self.set_anchor_axis(1, v))
                add_float_control("Anchor Z", "anchor_z", float(self.anchor_point[2]), lambda v: self.set_anchor_axis(2, v))
                add_float_control("Vertical K", "stiffness", self.stiffness, self.set_stiffness)
                add_float_control("Vertical D", "damping", self.damping, self.set_damping)
                add_float_control("Up preload", "preload_force", self.preload_force, self.set_preload_force)
                add_float_control("Planar K", "planar_stiffness", self.planar_stiffness, self.set_planar_stiffness)
                add_float_control("Planar D", "planar_damping", self.planar_damping, self.set_planar_damping)

                with ui.HStack(height=24):
                    ui.Button("Soft", clicked_fn=self.apply_soft_preset)
                    ui.Button("Balanced", clicked_fn=self.apply_balanced_preset)
                    ui.Button("Stiff", clicked_fn=self.apply_stiff_preset)

                self._ui_labels["anchor"] = ui.Label("")
                self._ui_labels["body"] = ui.Label("")
                self._ui_labels["force"] = ui.Label("")
                self._ui_labels["hint"] = ui.Label(
                    "Use sliders/buttons with the mouse. Drag Mode: left-drag moves X/Y, right-drag moves Z."
                )

    def _on_keyboard_event(self, event):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "T":
                self.vertical_input = 1.0
            elif event.input.name == "G":
                self.vertical_input = -1.0
            elif event.input.name == "Y":
                self.set_enabled(not self.enabled)
            elif event.input.name == "R":
                self.recenter_anchor_xy()
            elif event.input.name == "B":
                self.nudge_anchor(2, self.height_step)
            elif event.input.name == "N":
                self.nudge_anchor(2, -self.height_step)
            elif event.input.name == "H":
                self.set_drag_mode(not self.drag_mode)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in ("T", "G"):
                self.vertical_input = 0.0
        return True

    def _get_mouse_coords(self) -> tuple[float, float] | None:
        if self._input is None:
            return None
        x, y = self._input.get_mouse_coords_normalized(None)
        if ui is not None:
            x *= ui.Workspace.get_main_window_width()
            y *= ui.Workspace.get_main_window_height()
        return float(x), float(y)

    def _on_mouse_event(self, event, *_):
        if not self.drag_mode or not self.enabled:
            return True

        coords = self._get_mouse_coords()
        if coords is None:
            return True

        if event.type == carb.input.MouseEventType.LEFT_BUTTON_DOWN:
            self.drag_xy_active = True
            self._mouse_prev = coords
        elif event.type == carb.input.MouseEventType.LEFT_BUTTON_UP:
            self.drag_xy_active = False
            self._mouse_prev = None
        elif event.type == carb.input.MouseEventType.RIGHT_BUTTON_DOWN:
            self.drag_z_active = True
            self._mouse_prev = coords
        elif event.type == carb.input.MouseEventType.RIGHT_BUTTON_UP:
            self.drag_z_active = False
            self._mouse_prev = None
        elif event.type == carb.input.MouseEventType.MOVE and self._mouse_prev is not None:
            dx = coords[0] - self._mouse_prev[0]
            dy = coords[1] - self._mouse_prev[1]
            if self.drag_xy_active:
                self._drag_anchor_xy(dx, dy)
            elif self.drag_z_active:
                self.anchor_point[2] += -dy * self._drag_scale_z
                self._sync_ui_models()
            self._mouse_prev = coords
        return True

    def _drag_anchor_xy(self, dx: float, dy: float):
        if self._viewport is not None and ViewportCameraState is not None and Gf is not None:
            try:
                camera_path = self._viewport.get_active_camera()
                camera_state = ViewportCameraState(camera_path, self._viewport)
                cam_pos = camera_state.position_world
                cam_target = camera_state.target_world
                forward = torch.tensor(
                    [cam_target[0] - cam_pos[0], cam_target[1] - cam_pos[1], cam_target[2] - cam_pos[2]],
                    device=self.device,
                    dtype=torch.float32,
                )
                forward = forward / max(float(torch.linalg.norm(forward)), 1e-6)
                world_up = torch.tensor([0.0, 0.0, 1.0], device=self.device, dtype=torch.float32)
                right = torch.cross(forward, world_up, dim=0)
                if float(torch.linalg.norm(right)) < 1e-6:
                    right = torch.tensor([1.0, 0.0, 0.0], device=self.device, dtype=torch.float32)
                else:
                    right = right / torch.linalg.norm(right)
                up_plane = torch.cross(world_up, right, dim=0)
                up_plane = up_plane / max(float(torch.linalg.norm(up_plane)), 1e-6)
                self.anchor_point += (-dx * self._drag_scale_xy) * right + (dy * self._drag_scale_xy) * up_plane
                self._sync_ui_models()
                return
            except Exception:
                pass

        self.anchor_point[0] += -dx * self._drag_scale_xy
        self.anchor_point[1] += dy * self._drag_scale_xy
        self._sync_ui_models()

    def _nudge_model(self, key: str, delta: float):
        model = self._ui_models.get(key)
        if model is None:
            return
        model.set_value(float(model.as_float) + delta)

    def _sync_ui_models(self):
        if not self._ui_models:
            return
        if "enabled" in self._ui_models:
            self._ui_models["enabled"].set_value(self.enabled)
        if "drag_mode" in self._ui_models:
            self._ui_models["drag_mode"].set_value(self.drag_mode)
        for key, value in (
            ("anchor_x", float(self.anchor_point[0])),
            ("anchor_y", float(self.anchor_point[1])),
            ("anchor_z", float(self.anchor_point[2])),
            ("stiffness", self.stiffness),
            ("damping", self.damping),
            ("preload_force", self.preload_force),
            ("planar_stiffness", self.planar_stiffness),
            ("planar_damping", self.planar_damping),
        ):
            if key in self._ui_models:
                self._ui_models[key].set_value(value)

    def _refresh_ui_labels(self):
        if not self._ui_labels:
            return
        body_pos = self.robot.data.body_pos_w[0, self.body_id]
        if "mass" in self._ui_labels:
            if self.weight_force is None:
                self._ui_labels["mass"].text = (
                    f"Preload: {self.preload_force:.1f} N, Max force: {self.max_force:.1f} N"
                )
            else:
                self._ui_labels["mass"].text = (
                    f"Mass: {self.total_mass:.2f} kg, Weight: {self.weight_force:.1f} N, "
                    f"Preload: {self.preload_force:.1f} N, Max: {self.max_force:.1f} N"
                )
        self._ui_labels["anchor"].text = (
            f"Anchor xyz: {float(self.anchor_point[0]):+.2f}, {float(self.anchor_point[1]):+.2f}, {float(self.anchor_point[2]):+.2f}"
        )
        self._ui_labels["body"].text = (
            f"Body xyz: {float(body_pos[0]):+.2f}, {float(body_pos[1]):+.2f}, {float(body_pos[2]):+.2f}"
        )
        self._ui_labels["force"].text = (
            f"Hoist force xyz: {float(self.current_force[0]):+.1f}, {float(self.current_force[1]):+.1f}, {float(self.current_force[2]):+.1f}"
        )

    def _update_markers(self):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].unsqueeze(0)
        anchor_pos = self.anchor_point.unsqueeze(0)
        if self.anchor_visualizer is not None:
            self.anchor_visualizer.visualize(anchor_pos)
        if self.attach_visualizer is not None:
            self.attach_visualizer.visualize(body_pos)

    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)
        state = "enabled" if self.enabled else "disabled"
        print(f"[INFO] Hoist {state}.")
        self._sync_ui_models()

    def set_drag_mode(self, enabled: bool):
        self.drag_mode = bool(enabled)
        if not self.drag_mode:
            self.drag_xy_active = False
            self.drag_z_active = False
            self._mouse_prev = None
        state = "enabled" if self.drag_mode else "disabled"
        print(f"[INFO] Hoist drag mode {state}.")
        self._sync_ui_models()

    def set_anchor_axis(self, axis: int, value: float):
        self.anchor_point[axis] = float(value)

    def nudge_anchor(self, axis: int, delta: float):
        self.anchor_point[axis] += float(delta)
        if axis == 2:
            print(f"[INFO] Hoist anchor z -> {float(self.anchor_point[2]):.3f} m")
        self._sync_ui_models()

    def recenter_anchor_xy(self):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].clone()
        self.anchor_point[0] = body_pos[0]
        self.anchor_point[1] = body_pos[1]
        print("[INFO] Hoist anchor recentered above the attachment body.")
        self._sync_ui_models()

    def place_anchor_above_body(self, height_offset: float):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].clone()
        self.anchor_point[0] = body_pos[0]
        self.anchor_point[1] = body_pos[1]
        self.anchor_point[2] = body_pos[2] + float(height_offset)
        print(f"[INFO] Hoist anchor placed above body with offset {height_offset:.2f} m.")
        self._sync_ui_models()

    def set_stiffness(self, value: float):
        self.stiffness = max(0.0, float(value))

    def set_damping(self, value: float):
        self.damping = max(0.0, float(value))

    def set_preload_force(self, value: float):
        self.preload_force = max(0.0, float(value))

    def set_planar_stiffness(self, value: float):
        self.planar_stiffness = max(0.0, float(value))

    def set_planar_damping(self, value: float):
        self.planar_damping = max(0.0, float(value))

    def apply_soft_preset(self):
        self.stiffness = 500.0
        self.damping = 100.0
        self.preload_force = 0.0
        self.planar_stiffness = 120.0
        self.planar_damping = 160.0
        self._sync_ui_models()

    def apply_balanced_preset(self):
        self.stiffness = 800.0
        self.damping = 120.0
        self.preload_force = 350.0
        self.planar_stiffness = 200.0
        self.planar_damping = 220.0
        self._sync_ui_models()

    def apply_stiff_preset(self):
        self.stiffness = 1100.0
        self.damping = 180.0
        self.preload_force = 800.0
        self.planar_stiffness = 280.0
        self.planar_damping = 320.0
        self._sync_ui_models()

    def print_help(self):
        print("[INFO] Elastic hoist controls:")
        print(f"  attach body: {self.body_name}")
        print("  hold T/G    : raise/lower hoist anchor continuously")
        print("  press B/N   : raise/lower hoist anchor by one step")
        print("  press Y     : toggle hoist on/off")
        print("  press H     : toggle viewport drag mode")
        print("  press R     : recenter anchor above current body position")
        print(
            "  hoist model : vertical spring-damper + taut-cable upward preload + planar centering/damping"
        )
        print("  GUI         : use the 'Hoist Control' window sliders/buttons with the mouse")
        print("  Drag mode   : left-drag moves anchor in view plane, right-drag changes height")

    def step(self, sim_dt: float):
        self._debug_step += 1
        self.current_force.zero_()
        if not self.enabled:
            self._update_markers()
            self._refresh_ui_labels()
            return

        if self.vertical_input != 0.0:
            self.anchor_point[2] += self.vertical_input * self.height_rate * sim_dt

        body_pos = self.robot.data.body_pos_w[0, self.body_id]
        body_vel = self.robot.data.body_lin_vel_w[0, self.body_id]

        planar_error = self.anchor_point[:2] - body_pos[:2]
        planar_force = self.planar_stiffness * planar_error - self.planar_damping * body_vel[:2]

        vertical_displacement = self.anchor_point[2] - body_pos[2] - self.rest_length
        preload_force = self.preload_force if vertical_displacement > 0.0 else 0.0
        vertical_force = self.stiffness * vertical_displacement - self.damping * body_vel[2] + preload_force

        if self.max_force > 0.0:
            planar_norm = torch.linalg.norm(planar_force)
            if float(planar_norm) > self.max_force:
                planar_force = planar_force / planar_norm * self.max_force
            vertical_force = torch.clamp(vertical_force, -self.max_force, self.max_force)

        force = torch.zeros(3, device=self.device, dtype=torch.float32)
        force[:2] = planar_force.to(torch.float32)
        force[2] = vertical_force.to(torch.float32)
        self.current_force = force.clone()

        forces = torch.zeros((self.robot.num_instances, 1, 3), device=self.device, dtype=torch.float32)
        torques = torch.zeros_like(forces)
        forces[0, 0] = force.to(torch.float32)
        self.robot.permanent_wrench_composer.set_forces_and_torques(
            forces=forces,
            torques=torques,
            body_ids=[self.body_id],
            is_global=True,
        )
        if self.debug_interval > 0 and (self._debug_step % self.debug_interval) == 0:
            print(
                "[HOIST] "
                f"body={self.body_name}, "
                f"anchor_z={float(self.anchor_point[2]):+.3f}, "
                f"body_z={float(body_pos[2]):+.3f}, "
                f"disp_z={float(vertical_displacement):+.3f}, "
                f"preload={float(preload_force):+.1f}, "
                f"fz={float(force[2]):+.1f}, "
                f"max={float(self.max_force):+.1f}"
            )
        self._update_markers()
        self._refresh_ui_labels()


class HardHoist:
    """Hard hoist: directly writes the robot root pose so the selected body stays at the anchor."""

    def __init__(self, robot: Articulation):
        self.robot = robot
        self.device = robot.device
        self.enabled = args_cli.enable_hoist
        self.height_rate = float(args_cli.hoist_height_rate)
        self.height_step = float(args_cli.hoist_height_step)
        self.vertical_input = 0.0

        self.body_id, self.body_name = self._resolve_body(args_cli.hoist_body)
        body_pos = self.robot.data.body_pos_w[0, self.body_id].clone()
        self.anchor_point = body_pos.clone()
        self.anchor_point[2] += float(args_cli.hoist_height_offset)

        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        self._mouse = None
        self._sub_mouse = None
        self._viewport = None
        self.drag_mode = False
        self.drag_xy_active = False
        self.drag_z_active = False
        self._mouse_prev = None
        self._drag_scale_xy = 0.003
        self._drag_scale_z = 0.004
        self._window = None
        self._ui_models: dict[str, object] = {}
        self._ui_labels: dict[str, object] = {}
        self.anchor_visualizer = None
        self.attach_visualizer = None

        self._setup_keyboard()
        self._setup_visualizers()
        self._setup_ui_window()
        self.print_help()

    def _resolve_body(self, name: str) -> tuple[int, str]:
        return _resolve_body_with_fallback(self.robot, name, ("torso_link", "pelvis", "base", "base_link"))

    def _setup_keyboard(self):
        if carb is None or omni is None or args_cli.headless:
            return
        try:
            self._input = carb.input.acquire_input_interface()
            self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
            self._mouse = omni.appwindow.get_default_app_window().get_mouse()
            self._sub_mouse = self._input.subscribe_to_mouse_events(self._mouse, self._on_mouse_event)
            self._viewport = get_viewport_from_window_name("Viewport") if get_viewport_from_window_name else None
        except Exception as exc:
            print(f"[WARN] Hard hoist keyboard control unavailable: {exc}")
            self._input = None
            self._keyboard = None
            self._sub_keyboard = None
            self._mouse = None
            self._sub_mouse = None
            self._viewport = None

    def _setup_visualizers(self):
        anchor_cfg = SPHERE_MARKER_CFG.copy()
        anchor_cfg.prim_path = "/Visuals/HardHoist/anchor"
        anchor_cfg.markers["sphere"].radius = 0.08
        anchor_cfg.markers["sphere"].visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.2, 0.2))
        self.anchor_visualizer = VisualizationMarkers(anchor_cfg)

        attach_cfg = SPHERE_MARKER_CFG.copy()
        attach_cfg.prim_path = "/Visuals/HardHoist/attach"
        attach_cfg.markers["sphere"].radius = 0.06
        attach_cfg.markers["sphere"].visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.8, 1.0))
        self.attach_visualizer = VisualizationMarkers(attach_cfg)

    def _setup_ui_window(self):
        if ui is None or args_cli.headless:
            return

        def add_float_control(label: str, key: str, value: float, callback, step_label: str | None = None, step=None):
            with ui.HStack(height=24):
                ui.Label(label, width=130)
                field = ui.FloatDrag(width=ui.Fraction(1))
                field.model.set_value(value)
                field.model.add_value_changed_fn(lambda model: callback(float(model.as_float)))
                self._ui_models[key] = field.model
                if step is not None:
                    ui.Button(
                        step_label or "+",
                        width=32,
                        clicked_fn=lambda s=step, k=key: self._nudge_model(k, s),
                    )

        self._window = ui.Window(
            "Hard Hoist Control",
            width=360,
            height=330,
            visible=True,
            dock_preference=ui.DockPreference.RIGHT_TOP,
        )
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                self._ui_labels["title"] = ui.Label(f"Hard hoist attached to: {self.body_name}")

                with ui.HStack(height=24):
                    ui.Label("Enable hoist", width=130)
                    enabled_model = ui.SimpleBoolModel()
                    enabled_model.set_value(self.enabled)
                    enabled_model.add_value_changed_fn(lambda model: self.set_enabled(bool(model.as_bool)))
                    ui.CheckBox(model=enabled_model, width=24)
                    self._ui_models["enabled"] = enabled_model

                with ui.HStack(height=24):
                    ui.Label("Drag mode", width=130)
                    drag_model = ui.SimpleBoolModel()
                    drag_model.set_value(self.drag_mode)
                    drag_model.add_value_changed_fn(lambda model: self.set_drag_mode(bool(model.as_bool)))
                    ui.CheckBox(model=drag_model, width=24)
                    self._ui_models["drag_mode"] = drag_model

                with ui.HStack(height=24):
                    ui.Button("Recenter XY", clicked_fn=self.recenter_anchor_xy)
                    ui.Button("Body +0.8m", clicked_fn=lambda: self.place_anchor_above_body(0.8))
                    ui.Button("Body +1.2m", clicked_fn=lambda: self.place_anchor_above_body(1.2))

                with ui.HStack(height=24):
                    ui.Button("X-", width=40, clicked_fn=lambda: self.nudge_anchor(0, -0.05))
                    ui.Button("X+", width=40, clicked_fn=lambda: self.nudge_anchor(0, 0.05))
                    ui.Button("Y-", width=40, clicked_fn=lambda: self.nudge_anchor(1, -0.05))
                    ui.Button("Y+", width=40, clicked_fn=lambda: self.nudge_anchor(1, 0.05))
                    ui.Button("Z-", width=40, clicked_fn=lambda: self.nudge_anchor(2, -0.05))
                    ui.Button("Z+", width=40, clicked_fn=lambda: self.nudge_anchor(2, 0.05))

                add_float_control("Anchor X", "anchor_x", float(self.anchor_point[0]), lambda v: self.set_anchor_axis(0, v))
                add_float_control("Anchor Y", "anchor_y", float(self.anchor_point[1]), lambda v: self.set_anchor_axis(1, v))
                add_float_control("Anchor Z", "anchor_z", float(self.anchor_point[2]), lambda v: self.set_anchor_axis(2, v))

                self._ui_labels["anchor"] = ui.Label("")
                self._ui_labels["body"] = ui.Label("")
                self._ui_labels["hint"] = ui.Label(
                    "This mode teleports robot root. Drag Mode: left-drag moves X/Y, right-drag moves Z."
                )

    def _on_keyboard_event(self, event):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "T":
                self.vertical_input = 1.0
            elif event.input.name == "G":
                self.vertical_input = -1.0
            elif event.input.name == "Y":
                self.set_enabled(not self.enabled)
            elif event.input.name == "R":
                self.recenter_anchor_xy()
            elif event.input.name == "B":
                self.nudge_anchor(2, self.height_step)
            elif event.input.name == "N":
                self.nudge_anchor(2, -self.height_step)
            elif event.input.name == "H":
                self.set_drag_mode(not self.drag_mode)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in ("T", "G"):
                self.vertical_input = 0.0
        return True

    def _get_mouse_coords(self) -> tuple[float, float] | None:
        if self._input is None:
            return None
        x, y = self._input.get_mouse_coords_normalized(None)
        if ui is not None:
            x *= ui.Workspace.get_main_window_width()
            y *= ui.Workspace.get_main_window_height()
        return float(x), float(y)

    def _on_mouse_event(self, event, *_):
        if not self.drag_mode or not self.enabled:
            return True

        coords = self._get_mouse_coords()
        if coords is None:
            return True

        if event.type == carb.input.MouseEventType.LEFT_BUTTON_DOWN:
            self.drag_xy_active = True
            self._mouse_prev = coords
        elif event.type == carb.input.MouseEventType.LEFT_BUTTON_UP:
            self.drag_xy_active = False
            self._mouse_prev = None
        elif event.type == carb.input.MouseEventType.RIGHT_BUTTON_DOWN:
            self.drag_z_active = True
            self._mouse_prev = coords
        elif event.type == carb.input.MouseEventType.RIGHT_BUTTON_UP:
            self.drag_z_active = False
            self._mouse_prev = None
        elif event.type == carb.input.MouseEventType.MOVE and self._mouse_prev is not None:
            dx = coords[0] - self._mouse_prev[0]
            dy = coords[1] - self._mouse_prev[1]
            if self.drag_xy_active:
                self._drag_anchor_xy(dx, dy)
            elif self.drag_z_active:
                self.anchor_point[2] += -dy * self._drag_scale_z
                self._sync_ui_models()
            self._mouse_prev = coords
        return True

    def _drag_anchor_xy(self, dx: float, dy: float):
        if self._viewport is not None and ViewportCameraState is not None and Gf is not None:
            try:
                camera_path = self._viewport.get_active_camera()
                camera_state = ViewportCameraState(camera_path, self._viewport)
                cam_pos = camera_state.position_world
                cam_target = camera_state.target_world
                forward = torch.tensor(
                    [cam_target[0] - cam_pos[0], cam_target[1] - cam_pos[1], cam_target[2] - cam_pos[2]],
                    device=self.device,
                    dtype=torch.float32,
                )
                forward = forward / max(float(torch.linalg.norm(forward)), 1e-6)
                world_up = torch.tensor([0.0, 0.0, 1.0], device=self.device, dtype=torch.float32)
                right = torch.cross(forward, world_up, dim=0)
                if float(torch.linalg.norm(right)) < 1e-6:
                    right = torch.tensor([1.0, 0.0, 0.0], device=self.device, dtype=torch.float32)
                else:
                    right = right / torch.linalg.norm(right)
                up_plane = torch.cross(world_up, right, dim=0)
                up_plane = up_plane / max(float(torch.linalg.norm(up_plane)), 1e-6)
                self.anchor_point += (-dx * self._drag_scale_xy) * right + (dy * self._drag_scale_xy) * up_plane
                self._sync_ui_models()
                return
            except Exception:
                pass

        self.anchor_point[0] += -dx * self._drag_scale_xy
        self.anchor_point[1] += dy * self._drag_scale_xy
        self._sync_ui_models()

    def _nudge_model(self, key: str, delta: float):
        model = self._ui_models.get(key)
        if model is None:
            return
        model.set_value(float(model.as_float) + delta)

    def _sync_ui_models(self):
        if not self._ui_models:
            return
        if "enabled" in self._ui_models:
            self._ui_models["enabled"].set_value(self.enabled)
        if "drag_mode" in self._ui_models:
            self._ui_models["drag_mode"].set_value(self.drag_mode)
        for key, value in (
            ("anchor_x", float(self.anchor_point[0])),
            ("anchor_y", float(self.anchor_point[1])),
            ("anchor_z", float(self.anchor_point[2])),
        ):
            if key in self._ui_models:
                self._ui_models[key].set_value(value)

    def _refresh_ui_labels(self):
        if not self._ui_labels:
            return
        body_pos = self.robot.data.body_pos_w[0, self.body_id]
        self._ui_labels["anchor"].text = (
            f"Anchor xyz: {float(self.anchor_point[0]):+.2f}, {float(self.anchor_point[1]):+.2f}, {float(self.anchor_point[2]):+.2f}"
        )
        self._ui_labels["body"].text = (
            f"Body xyz: {float(body_pos[0]):+.2f}, {float(body_pos[1]):+.2f}, {float(body_pos[2]):+.2f}"
        )

    def _update_markers(self):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].unsqueeze(0)
        anchor_pos = self.anchor_point.unsqueeze(0)
        if self.anchor_visualizer is not None:
            self.anchor_visualizer.visualize(anchor_pos)
        if self.attach_visualizer is not None:
            self.attach_visualizer.visualize(body_pos)

    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)
        state = "enabled" if self.enabled else "disabled"
        print(f"[INFO] Hard hoist {state}.")
        self._sync_ui_models()

    def set_drag_mode(self, enabled: bool):
        self.drag_mode = bool(enabled)
        if not self.drag_mode:
            self.drag_xy_active = False
            self.drag_z_active = False
            self._mouse_prev = None
        state = "enabled" if self.drag_mode else "disabled"
        print(f"[INFO] Hard hoist drag mode {state}.")
        self._sync_ui_models()

    def set_anchor_axis(self, axis: int, value: float):
        self.anchor_point[axis] = float(value)

    def nudge_anchor(self, axis: int, delta: float):
        self.anchor_point[axis] += float(delta)
        if axis == 2:
            print(f"[INFO] Hard hoist anchor z -> {float(self.anchor_point[2]):.3f} m")
        self._sync_ui_models()

    def recenter_anchor_xy(self):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].clone()
        self.anchor_point[0] = body_pos[0]
        self.anchor_point[1] = body_pos[1]
        print("[INFO] Hard hoist anchor recentered above the attachment body.")
        self._sync_ui_models()

    def place_anchor_above_body(self, height_offset: float):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].clone()
        self.anchor_point[0] = body_pos[0]
        self.anchor_point[1] = body_pos[1]
        self.anchor_point[2] = body_pos[2] + float(height_offset)
        print(f"[INFO] Hard hoist anchor placed above body with offset {height_offset:.2f} m.")
        self._sync_ui_models()

    def print_help(self):
        print("[INFO] Hard hoist controls:")
        print(f"  attach body: {self.body_name}")
        print("  hold T/G    : raise/lower anchor continuously")
        print("  press B/N   : raise/lower anchor by one step")
        print("  press Y     : toggle hoist on/off")
        print("  press H     : toggle viewport drag mode")
        print("  press R     : recenter anchor above current body position")
        print("  model       : teleports robot root pose to keep the attachment body at the anchor")
        print("  GUI         : use the 'Hard Hoist Control' window")

    def step(self, sim_dt: float):
        if not self.enabled:
            self._update_markers()
            self._refresh_ui_labels()
            return

        if self.vertical_input != 0.0:
            self.anchor_point[2] += self.vertical_input * self.height_rate * sim_dt

        root_pos = self.robot.data.root_pos_w[0]
        root_quat = self.robot.data.root_quat_w[0]
        body_pos = self.robot.data.body_pos_w[0, self.body_id]
        offset = body_pos - root_pos
        desired_root_pos = self.anchor_point - offset

        root_pose = torch.zeros((self.robot.num_instances, 7), device=self.device, dtype=torch.float32)
        root_pose[0, :3] = desired_root_pos.to(torch.float32)
        root_pose[0, 3:] = root_quat.to(torch.float32)
        root_vel = torch.zeros((self.robot.num_instances, 6), device=self.device, dtype=torch.float32)
        self.robot.write_root_pose_to_sim(root_pose)
        self.robot.write_root_velocity_to_sim(root_vel)
        self._update_markers()
        self._refresh_ui_labels()


class VirtualHand:
    """Virtual hand tool for lifting or stabilizing a specific robot body."""

    def __init__(self, robot: Articulation):
        self.robot = robot
        self.device = robot.device
        self.enabled = args_cli.enable_virtual_hand
        self.stiffness = float(args_cli.hand_stiffness)
        self.damping = float(args_cli.hand_damping)
        self.max_force = float(args_cli.hand_max_force)
        self.position_step = float(args_cli.hand_position_step)
        self.current_force = torch.zeros(3, device=self.device, dtype=torch.float32)

        self.body_candidates = _build_virtual_hand_body_candidates(robot, args_cli.hand_body)
        self.active_body_index = 0
        self.body_id, self.body_name = self._resolve_active_body()
        self.target_point = self.robot.data.body_pos_w[0, self.body_id].clone()

        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        self._window = None
        self._ui_models: dict[str, object] = {}
        self._ui_labels: dict[str, object] = {}
        self.target_visualizer = None
        self.attach_visualizer = None
        self._setup_keyboard()
        self._setup_visualizers()
        self._setup_ui_window()
        self.print_help()

    def _resolve_active_body(self) -> tuple[int, str]:
        candidate = self.body_candidates[self.active_body_index]
        return _resolve_body_with_fallback(self.robot, candidate, ("torso_link", "pelvis", "base_link"))

    def _setup_keyboard(self):
        if carb is None or omni is None or args_cli.headless:
            return
        try:
            self._input = carb.input.acquire_input_interface()
            self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
        except Exception as exc:
            print(f"[WARN] Virtual hand keyboard control unavailable: {exc}")
            self._input = None
            self._keyboard = None
            self._sub_keyboard = None

    def _setup_visualizers(self):
        target_cfg = SPHERE_MARKER_CFG.copy()
        target_cfg.prim_path = "/Visuals/VirtualHand/target"
        target_cfg.markers["sphere"].radius = 0.07
        target_cfg.markers["sphere"].visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.95, 0.2))
        self.target_visualizer = VisualizationMarkers(target_cfg)

        attach_cfg = SPHERE_MARKER_CFG.copy()
        attach_cfg.prim_path = "/Visuals/VirtualHand/attach"
        attach_cfg.markers["sphere"].radius = 0.05
        attach_cfg.markers["sphere"].visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 1.0, 0.35))
        self.attach_visualizer = VisualizationMarkers(attach_cfg)

    def _setup_ui_window(self):
        if ui is None or args_cli.headless:
            return

        def add_float_control(label: str, key: str, value: float, callback):
            with ui.HStack(height=24):
                ui.Label(label, width=130)
                field = ui.FloatDrag(width=ui.Fraction(1))
                field.model.set_value(value)
                field.model.add_value_changed_fn(lambda model: callback(float(model.as_float)))
                self._ui_models[key] = field.model

        self._window = ui.Window(
            "Virtual Hand Control",
            width=380,
            height=460,
            visible=True,
            dock_preference=ui.DockPreference.RIGHT_TOP,
        )
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                self._ui_labels["body_title"] = ui.Label(f"Target body: {self.body_name}")

                with ui.HStack(height=24):
                    ui.Label("Enable hand", width=130)
                    enabled_model = ui.SimpleBoolModel()
                    enabled_model.set_value(self.enabled)
                    enabled_model.add_value_changed_fn(lambda model: self.set_enabled(bool(model.as_bool)))
                    ui.CheckBox(model=enabled_model, width=24)
                    self._ui_models["enabled"] = enabled_model

                with ui.HStack(height=24):
                    ui.Button("Prev Body", clicked_fn=self.select_previous_body)
                    ui.Button("Next Body", clicked_fn=self.select_next_body)
                    ui.Button("Snap Target", clicked_fn=self.snap_target_to_body)

                with ui.HStack(height=24):
                    ui.Button("X-", width=40, clicked_fn=lambda: self.nudge_target(0, -self.position_step))
                    ui.Button("X+", width=40, clicked_fn=lambda: self.nudge_target(0, self.position_step))
                    ui.Button("Y-", width=40, clicked_fn=lambda: self.nudge_target(1, -self.position_step))
                    ui.Button("Y+", width=40, clicked_fn=lambda: self.nudge_target(1, self.position_step))
                    ui.Button("Z-", width=40, clicked_fn=lambda: self.nudge_target(2, -self.position_step))
                    ui.Button("Z+", width=40, clicked_fn=lambda: self.nudge_target(2, self.position_step))

                add_float_control("Target X", "target_x", float(self.target_point[0]), lambda v: self.set_target_axis(0, v))
                add_float_control("Target Y", "target_y", float(self.target_point[1]), lambda v: self.set_target_axis(1, v))
                add_float_control("Target Z", "target_z", float(self.target_point[2]), lambda v: self.set_target_axis(2, v))
                add_float_control("Hand K", "stiffness", self.stiffness, self.set_stiffness)
                add_float_control("Hand D", "damping", self.damping, self.set_damping)

                with ui.HStack(height=24):
                    ui.Button("Soft", clicked_fn=self.apply_soft_preset)
                    ui.Button("Balanced", clicked_fn=self.apply_balanced_preset)
                    ui.Button("Strong", clicked_fn=self.apply_strong_preset)

                self._ui_labels["target"] = ui.Label("")
                self._ui_labels["body"] = ui.Label("")
                self._ui_labels["force"] = ui.Label("")
                self._ui_labels["hint"] = ui.Label(
                    "Hotkeys: M toggle, P snap, J/L move X, I/K move Y, U/O move Z, </> change body."
                )

    def _on_keyboard_event(self, event):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        if event.input.name == "M":
            self.set_enabled(not self.enabled)
        elif event.input.name == "P":
            self.snap_target_to_body()
        elif event.input.name == "J":
            self.nudge_target(0, -self.position_step)
        elif event.input.name == "L":
            self.nudge_target(0, self.position_step)
        elif event.input.name == "I":
            self.nudge_target(1, self.position_step)
        elif event.input.name == "K":
            self.nudge_target(1, -self.position_step)
        elif event.input.name == "U":
            self.nudge_target(2, self.position_step)
        elif event.input.name == "O":
            self.nudge_target(2, -self.position_step)
        elif event.input.name == "COMMA":
            self.select_previous_body()
        elif event.input.name == "PERIOD":
            self.select_next_body()
        return True

    def _sync_ui_models(self):
        if not self._ui_models:
            return
        if "enabled" in self._ui_models:
            self._ui_models["enabled"].set_value(self.enabled)
        for key, value in (
            ("target_x", float(self.target_point[0])),
            ("target_y", float(self.target_point[1])),
            ("target_z", float(self.target_point[2])),
            ("stiffness", self.stiffness),
            ("damping", self.damping),
        ):
            if key in self._ui_models:
                self._ui_models[key].set_value(value)

    def _refresh_ui_labels(self):
        if not self._ui_labels:
            return
        body_pos = self.robot.data.body_pos_w[0, self.body_id]
        self._ui_labels["body_title"].text = f"Target body: {self.body_name}"
        self._ui_labels["target"].text = (
            f"Target xyz: {float(self.target_point[0]):+.2f}, {float(self.target_point[1]):+.2f}, {float(self.target_point[2]):+.2f}"
        )
        self._ui_labels["body"].text = (
            f"Body xyz: {float(body_pos[0]):+.2f}, {float(body_pos[1]):+.2f}, {float(body_pos[2]):+.2f}"
        )
        self._ui_labels["force"].text = (
            f"Hand force xyz: {float(self.current_force[0]):+.1f}, {float(self.current_force[1]):+.1f}, {float(self.current_force[2]):+.1f}"
        )

    def _update_markers(self):
        body_pos = self.robot.data.body_pos_w[0, self.body_id].unsqueeze(0)
        target_pos = self.target_point.unsqueeze(0)
        if self.target_visualizer is not None:
            self.target_visualizer.visualize(target_pos)
        if self.attach_visualizer is not None:
            self.attach_visualizer.visualize(body_pos)

    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)
        state = "enabled" if self.enabled else "disabled"
        print(f"[INFO] Virtual hand {state}.")
        self._sync_ui_models()

    def select_previous_body(self):
        self.active_body_index = (self.active_body_index - 1) % len(self.body_candidates)
        self.body_id, self.body_name = self._resolve_active_body()
        self.snap_target_to_body(log_message=False)
        print(f"[INFO] Virtual hand body -> {self.body_name}")

    def select_next_body(self):
        self.active_body_index = (self.active_body_index + 1) % len(self.body_candidates)
        self.body_id, self.body_name = self._resolve_active_body()
        self.snap_target_to_body(log_message=False)
        print(f"[INFO] Virtual hand body -> {self.body_name}")

    def snap_target_to_body(self, log_message: bool = True):
        self.target_point = self.robot.data.body_pos_w[0, self.body_id].clone()
        self._sync_ui_models()
        if log_message:
            print(f"[INFO] Virtual hand target snapped to {self.body_name}.")

    def set_target_axis(self, axis: int, value: float):
        self.target_point[axis] = float(value)

    def nudge_target(self, axis: int, delta: float):
        self.target_point[axis] += float(delta)
        self._sync_ui_models()

    def set_stiffness(self, value: float):
        self.stiffness = max(0.0, float(value))

    def set_damping(self, value: float):
        self.damping = max(0.0, float(value))

    def apply_soft_preset(self):
        self.stiffness = 450.0
        self.damping = 80.0
        self._sync_ui_models()

    def apply_balanced_preset(self):
        self.stiffness = 700.0
        self.damping = 120.0
        self._sync_ui_models()

    def apply_strong_preset(self):
        self.stiffness = 950.0
        self.damping = 180.0
        self._sync_ui_models()

    def print_help(self):
        print("[INFO] Virtual hand controls:")
        print(f"  target body : {self.body_name}")
        print("  press M     : toggle virtual hand on/off")
        print("  press P     : snap target to current body position")
        print("  press J/L   : move target in world X")
        print("  press I/K   : move target in world Y")
        print("  press U/O   : move target in world Z")
        print("  press </>   : switch target body")
        print("  GUI         : use the 'Virtual Hand Control' window for body selection and target editing")

    def step(self, sim_dt: float):
        del sim_dt
        self.current_force.zero_()
        body_pos = self.robot.data.body_pos_w[0, self.body_id]
        body_vel = self.robot.data.body_lin_vel_w[0, self.body_id]

        if not self.enabled:
            self._update_markers()
            self._refresh_ui_labels()
            return

        force = self.stiffness * (self.target_point - body_pos) - self.damping * body_vel
        force_norm = torch.linalg.norm(force)
        if self.max_force > 0.0 and float(force_norm) > self.max_force:
            force = force / force_norm * self.max_force
        self.current_force = force.clone()

        forces = torch.zeros((self.robot.num_instances, 1, 3), device=self.device, dtype=torch.float32)
        torques = torch.zeros_like(forces)
        forces[0, 0] = force.to(torch.float32)
        self.robot.permanent_wrench_composer.set_forces_and_torques(
            forces=forces,
            torques=torques,
            body_ids=[self.body_id],
            is_global=True,
        )
        self._update_markers()
        self._refresh_ui_labels()


class StartupHoldToggle:
    def __init__(self, bridge: UnitreeSdk2IsaacBridge):
        self.bridge = bridge
        self._input = None
        self._keyboard = None
        self._sub_keyboard = None
        self._window = None
        self._ui_models: dict[str, object] = {}
        self._ui_labels: dict[str, object] = {}
        self._setup_keyboard()
        self._setup_ui_window()
        self.print_help()

    def _setup_keyboard(self):
        if carb is None or omni is None or args_cli.headless:
            return
        try:
            self._input = carb.input.acquire_input_interface()
            self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
            self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
        except Exception as exc:
            print(f"[WARN] Startup hold keyboard control unavailable: {exc}")
            self._input = None
            self._keyboard = None
            self._sub_keyboard = None

    def _setup_ui_window(self):
        if ui is None or args_cli.headless:
            return
        self._window = ui.Window(
            "Startup Hold",
            width=320,
            height=150,
            visible=True,
            dock_preference=ui.DockPreference.RIGHT_TOP,
        )
        with self._window.frame:
            with ui.VStack(spacing=6, height=0):
                with ui.HStack(height=24):
                    ui.Label("Force hold", width=130)
                    enabled_model = ui.SimpleBoolModel()
                    enabled_model.set_value(bool(self.bridge.hold_override_enabled))
                    enabled_model.add_value_changed_fn(lambda model: self._set_enabled(bool(model.as_bool)))
                    ui.CheckBox(model=enabled_model, width=24)
                    self._ui_models["enabled"] = enabled_model
                self._ui_labels["state"] = ui.Label("")
                self._ui_labels["hint"] = ui.Label("Hotkey: H toggle startup hold")

    def _set_enabled(self, enabled: bool):
        enabled = bool(enabled)
        if enabled == bool(self.bridge.hold_override_enabled):
            return
        self.bridge.toggle_hold_override()
        self._sync_ui_models()

    def _sync_ui_models(self):
        if "enabled" in self._ui_models:
            self._ui_models["enabled"].set_value(bool(self.bridge.hold_override_enabled))

    def _refresh_ui_labels(self):
        if not self._ui_labels:
            return
        override = bool(self.bridge.hold_override_enabled)
        received = bool(self.bridge.command_received)
        if override:
            self._ui_labels["state"].text = "State: HOLD (override)" if received else "State: HOLD (pre-cmd)"
        else:
            default_hold = bool(self.bridge.hold_default_pose_enabled)
            if received:
                self._ui_labels["state"].text = "State: LOWCMD"
            else:
                self._ui_labels["state"].text = "State: HOLD (default)" if default_hold else "State: PASSIVE"

    def _on_keyboard_event(self, event):
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        if event.input.name == "H":
            self.bridge.toggle_hold_override()
            self._sync_ui_models()
        return True

    def print_help(self):
        print("[INFO] Startup hold controls:")
        print("  press H     : toggle startup hold override on/off")

    def step(self, _sim_dt: float):
        self._refresh_ui_labels()


def design_scene(robot_spec: RobotSpec) -> Articulation:
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)

    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    if not robot_spec.use_usd:
        return Articulation(robot_spec.asset_cfg.replace(prim_path="/World/Robot"))

    articulation_root_prim_path = args_cli.h2_articulation_root.strip() or None
    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=args_cli.h2_usd,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=args_cli.h2_linear_damping,
                angular_damping=args_cli.h2_angular_damping,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=2,
            ),
            activate_contact_sensors=True,
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
        actuators={
            "all": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit_sim=500.0,
                velocity_limit_sim=100.0,
                stiffness=0.0
                if args_cli.use_lowcmd_kp_kd or (not bool(args_cli.hold_default_pose))
                else float(args_cli.startup_hold_kp),
                damping=0.0
                if args_cli.use_lowcmd_kp_kd or (not bool(args_cli.hold_default_pose))
                else float(args_cli.startup_hold_kd),
            )
        },
        articulation_root_prim_path=articulation_root_prim_path,
    )
    return Articulation(robot_cfg)


def main():
    robot_key = "g1_29dof" if args_cli.robot == "g1" else args_cli.robot
    robot_spec = ROBOT_SPECS[robot_key]

    ChannelFactoryInitialize(args_cli.domain_id, args_cli.interface)

    sim_cfg = sim_utils.SimulationCfg(
        dt=args_cli.physics_dt,
        render_interval=max(1, args_cli.render_interval),
        device=args_cli.device,
    )
    sim_cfg.physx.enable_external_forces_every_iteration = bool(args_cli.physx_enable_external_forces_every_iteration)
    sim_cfg.physx.min_velocity_iteration_count = int(args_cli.physx_min_velocity_iterations)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.5, 2.5, 2.0], target=[0.0, 0.0, 0.8])

    robot = design_scene(robot_spec)
    sim.reset()
    robot.update(sim.get_physics_dt())
    support_tool = create_support_constraint(robot)
    if support_tool is not None:
        sim.reset()
        robot.update(sim.get_physics_dt())

    bridge = UnitreeSdk2IsaacBridge(robot, robot_spec)
    bridge.reset_robot()
    bridge.print_mapping()
    if args_cli.enable_hoist:
        if args_cli.hoist_model == "hard":
            print("[WARN] Hoist model 'hard' teleports the robot and is intended for short debug use only.")
            hoist = HardHoist(robot)
        else:
            hoist = ElasticHoist(robot, sim_cfg)
    else:
        hoist = NullTool()
    virtual_hand = VirtualHand(robot) if args_cli.enable_virtual_hand else NullTool()
    startup_hold = StartupHoldToggle(bridge) if (not args_cli.headless) else NullTool()
    print("[INFO] Unitree SDK2 Isaac bridge is running.")
    print(
        f"[INFO] Robot: {robot_spec.name}, dt: {args_cli.physics_dt}, "
        f"render_interval: {max(1, args_cli.render_interval)}, "
        f"interface: {args_cli.interface or '<default>'}"
    )

    sim_dt = sim.get_physics_dt()
    step_count = 0

    while simulation_app.is_running():
        bridge.apply_low_cmd(sim_dt)
        if support_tool is not None:
            support_tool.step(sim_dt)
        robot.permanent_wrench_composer.reset()
        hoist.step(sim_dt)
        virtual_hand.step(sim_dt)
        startup_hold.step(sim_dt)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim_dt)
        bridge.publish_all()

        if args_cli.status_interval_steps and step_count % int(args_cli.status_interval_steps) == 0:
            print(
                "[INFO] "
                f"Sim time: {step_count * sim_dt:.3f}s, "
                f"lowstate published: {bridge.low_state_publish_count}, "
                f"lowcmd received: {bridge.command_received}, "
                f"lowcmd count: {bridge.low_cmd_count}"
            )
        if args_cli.debug_lowcmd_interval_steps and step_count % int(args_cli.debug_lowcmd_interval_steps) == 0:
            print(bridge.lowcmd_diagnostics(sim_dt=sim_dt, step_count=step_count))
        step_count += 1


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
