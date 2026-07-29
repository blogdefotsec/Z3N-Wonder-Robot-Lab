#!/usr/bin/env python3
"""H2 upper-body teach-and-playback helper.

支持两种控制后端:
1. --mode motion: 使用 rt/arm_sdk，并通过 kNotUsedJoint 权重进入/退出 ArmSDK。
2. --mode debug: 使用 rt/lowcmd，先释放活动运动模式，再直接发底层控制。

支持两种工作流程:
1. 默认录制 + 回放。
2. --play <file.json> 只播放，不录制。
"""

import argparse
import json
import subprocess
import sys
import threading
import time
from copy import deepcopy
from numbers import Real
from pathlib import Path

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


HERE = Path(__file__).resolve().parent
OUT_DIR = HERE / "output"

CONTROL_MODE_MOTION = "motion"
CONTROL_MODE_DEBUG = "debug"

ARM_SDK_WEIGHT_JOINT = 31

KP_PLAYBACK = 80.0
KD_PLAYBACK = 1.5
KP_DEBUG_HOLD = 300.0
KD_DEBUG_HOLD = 3.0


class H2JointIndex:
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    HeadYaw = 29
    HeadPitch = 30

    kNotUsedJoint = ARM_SDK_WEIGHT_JOINT


UPPER_BODY_JOINTS = [
    H2JointIndex.WaistYaw,
    H2JointIndex.WaistRoll,
    H2JointIndex.WaistPitch,
    H2JointIndex.LeftShoulderPitch,
    H2JointIndex.LeftShoulderRoll,
    H2JointIndex.LeftShoulderYaw,
    H2JointIndex.LeftElbow,
    H2JointIndex.LeftWristRoll,
    H2JointIndex.LeftWristPitch,
    H2JointIndex.LeftWristYaw,
    H2JointIndex.RightShoulderPitch,
    H2JointIndex.RightShoulderRoll,
    H2JointIndex.RightShoulderYaw,
    H2JointIndex.RightElbow,
    H2JointIndex.RightWristRoll,
    H2JointIndex.RightWristPitch,
    H2JointIndex.RightWristYaw,
    H2JointIndex.HeadYaw,
    H2JointIndex.HeadPitch,
]
UPPER_BODY_JOINT_SET = set(UPPER_BODY_JOINTS)

JOINT_NAMES = {
    H2JointIndex.WaistYaw: "WaistYaw",
    H2JointIndex.WaistRoll: "WaistRoll",
    H2JointIndex.WaistPitch: "WaistPitch",
    H2JointIndex.LeftShoulderPitch: "LeftShoulderPitch",
    H2JointIndex.LeftShoulderRoll: "LeftShoulderRoll",
    H2JointIndex.LeftShoulderYaw: "LeftShoulderYaw",
    H2JointIndex.LeftElbow: "LeftElbow",
    H2JointIndex.LeftWristRoll: "LeftWristRoll",
    H2JointIndex.LeftWristPitch: "LeftWristPitch",
    H2JointIndex.LeftWristYaw: "LeftWristYaw",
    H2JointIndex.RightShoulderPitch: "RightShoulderPitch",
    H2JointIndex.RightShoulderRoll: "RightShoulderRoll",
    H2JointIndex.RightShoulderYaw: "RightShoulderYaw",
    H2JointIndex.RightElbow: "RightElbow",
    H2JointIndex.RightWristRoll: "RightWristRoll",
    H2JointIndex.RightWristPitch: "RightWristPitch",
    H2JointIndex.RightWristYaw: "RightWristYaw",
    H2JointIndex.HeadYaw: "HeadYaw",
    H2JointIndex.HeadPitch: "HeadPitch",
}


def detect_iface():
    out = subprocess.run(
        ["ip", "-4", "-br", "addr"], capture_output=True, text=True, check=False
    ).stdout
    return next((line.split()[0] for line in out.splitlines() if "192.168.123." in line), None)


def release_motion_mode():
    """Release active high-level motion service before sending lowcmd."""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    status, result = msc.CheckMode()
    if not status:
        print("WARN: MotionSwitcher CheckMode failed; continue with lowcmd.", flush=True)
        return
    if not result.get("name"):
        print("motion switcher: no active mode to release", flush=True)
        return
    print(f"motion switcher: releasing active mode '{result.get('name')}'", flush=True)
    for _ in range(10):
        msc.ReleaseMode()
        time.sleep(1.0)
        status, result = msc.CheckMode()
        if status and not result.get("name"):
            print("motion switcher: lowcmd control is now available", flush=True)
            return
    raise RuntimeError(f"failed to release active motion mode: {result}")


def load_trajectory_file(file_path):
    try:
        raw = file_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise ValueError(f"播放文件不存在: {file_path}") from exc

    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"播放文件不是合法 JSON: {exc}") from exc

    if not isinstance(payload, dict):
        raise ValueError("播放文件根节点必须是 JSON 对象。")

    joint_ids = payload.get("joint_ids")
    if joint_ids != UPPER_BODY_JOINTS:
        raise ValueError(
            "播放文件 joint_ids 与当前脚本上半身关节定义不一致。"
        )

    sample_count = payload.get("sample_count")
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("播放文件 samples 必须是非空数组。")
    if sample_count is not None and sample_count != len(samples):
        raise ValueError("播放文件 sample_count 与实际样本数量不一致。")

    normalized_samples = []
    prev_t = -1e-9
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"samples[{index}] 必须是对象。")
        t = sample.get("t")
        q = sample.get("q")
        if not isinstance(t, Real):
            raise ValueError(f"samples[{index}].t 必须是数值。")
        if t < 0:
            raise ValueError(f"samples[{index}].t 不能小于 0。")
        if t < prev_t:
            raise ValueError("samples 的时间戳必须单调不减。")
        if not isinstance(q, list):
            raise ValueError(f"samples[{index}].q 必须是数组。")
        if len(q) != len(UPPER_BODY_JOINTS):
            raise ValueError(
                f"samples[{index}].q 长度错误，应为 {len(UPPER_BODY_JOINTS)}。"
            )
        q_values = []
        for joint_idx, value in enumerate(q):
            if not isinstance(value, Real):
                raise ValueError(
                    f"samples[{index}].q[{joint_idx}] 必须是数值。"
                )
            q_values.append(float(value))
        normalized_samples.append({"t": float(t), "q": q_values})
        prev_t = float(t)

    return payload, normalized_samples


class EnterMonitor:
    def __init__(self, prompt):
        self.prompt = prompt
        self._event = threading.Event()
        self._thread = threading.Thread(target=self._wait_input, daemon=True)

    def _wait_input(self):
        try:
            input(self.prompt)
        except EOFError:
            return
        self._event.set()

    def start(self):
        self._thread.start()

    def is_set(self):
        return self._event.is_set()


class RecorderPlayer:
    def __init__(self, control_mode, iface, publish_dt, record_dt, playback_dt, transition_time):
        self.control_mode = control_mode
        self.iface = iface
        self.publish_dt = publish_dt
        self.record_dt = record_dt
        self.playback_dt = playback_dt
        self.transition_time = transition_time

        self.publish_topic = (
            "rt/arm_sdk" if self.control_mode == CONTROL_MODE_MOTION else "rt/lowcmd"
        )

        self.low_state = None
        self.first_low_state = threading.Event()
        self.low_state_lock = threading.Lock()
        self.cmd_lock = threading.Lock()
        self.stop_pub = threading.Event()

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()

        self.publisher = None
        self.low_state_sub = None
        self.pub_thread = None

        self.current_weight = 0.0
        self.debug_hold_q = None
        self.last_record_file = None

    def init(self):
        ChannelFactoryInitialize(0, self.iface)
        if self.control_mode == CONTROL_MODE_DEBUG:
            release_motion_mode()

        self.publisher = ChannelPublisher(self.publish_topic, LowCmd_)
        self.publisher.Init()

        self.low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.low_state_sub.Init(self.low_state_handler, 10)

        self._init_cmd()

    def _init_cmd(self):
        self.low_cmd.mode_pr = 0
        for idx in range(len(self.low_cmd.motor_cmd)):
            mc = self.low_cmd.motor_cmd[idx]
            mc.mode = 1
            mc.q = 0.0
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = 0.0
            mc.kd = 0.0

        if len(self.low_cmd.motor_cmd) > ARM_SDK_WEIGHT_JOINT:
            self.low_cmd.motor_cmd[ARM_SDK_WEIGHT_JOINT].q = 0.0

    def low_state_handler(self, msg: LowState_):
        with self.low_state_lock:
            self.low_state = msg
        with self.cmd_lock:
            self.low_cmd.mode_machine = msg.mode_machine
        self.first_low_state.set()

    def wait_for_low_state(self, timeout=10.0):
        if not self.first_low_state.wait(timeout):
            raise RuntimeError("等待 lowstate 超时，请确认 DDS 与机器人连接正常。")

    def start_publisher(self):
        self.pub_thread = threading.Thread(target=self._publisher_loop, daemon=True)
        self.pub_thread.start()

    def _publisher_loop(self):
        while not self.stop_pub.is_set():
            with self.cmd_lock:
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.publisher.Write(self.low_cmd)
            time.sleep(self.publish_dt)

    def shutdown(self):
        self.stop_pub.set()
        if self.pub_thread is not None:
            self.pub_thread.join(timeout=1.0)

    def _snapshot_motor_q(self):
        with self.low_state_lock:
            if self.low_state is None:
                raise RuntimeError("尚未收到 lowstate。")
            return [state.q for state in self.low_state.motor_state]

    def snapshot_upper_body(self):
        motor_q = self._snapshot_motor_q()
        return [motor_q[j] for j in UPPER_BODY_JOINTS]

    def capture_debug_hold_pose(self):
        self.debug_hold_q = self._snapshot_motor_q()

    def _set_motion_zero_torque_pose(self):
        current_q = self.snapshot_upper_body()
        with self.cmd_lock:
            for joint, q in zip(UPPER_BODY_JOINTS, current_q):
                mc = self.low_cmd.motor_cmd[joint]
                mc.mode = 1
                mc.q = q
                mc.dq = 0.0
                mc.kp = 0.0
                mc.kd = 0.0
                mc.tau = 0.0

    def _set_debug_zero_torque_pose(self):
        if self.debug_hold_q is None:
            self.capture_debug_hold_pose()
        joint_count = min(len(self.debug_hold_q), len(self.low_cmd.motor_cmd))
        with self.cmd_lock:
            for joint in range(joint_count):
                mc = self.low_cmd.motor_cmd[joint]
                mc.mode = 1
                mc.dq = 0.0
                mc.tau = 0.0
                if joint in UPPER_BODY_JOINT_SET:
                    current_q = self.debug_hold_q[joint]
                    mc.q = current_q
                    mc.kp = 0.0
                    mc.kd = 0.0
                else:
                    mc.q = self.debug_hold_q[joint]
                    mc.kp = KP_DEBUG_HOLD
                    mc.kd = KD_DEBUG_HOLD

    def set_zero_torque_pose(self):
        if self.control_mode == CONTROL_MODE_MOTION:
            self._set_motion_zero_torque_pose()
        else:
            self._set_debug_zero_torque_pose()

    def _set_motion_playback_pose(self, q_list):
        with self.cmd_lock:
            for joint, q in zip(UPPER_BODY_JOINTS, q_list):
                mc = self.low_cmd.motor_cmd[joint]
                mc.mode = 1
                mc.q = q
                mc.dq = 0.0
                mc.kp = KP_PLAYBACK
                mc.kd = KD_PLAYBACK
                mc.tau = 0.0

    def _set_debug_playback_pose(self, q_list):
        if self.debug_hold_q is None:
            self.capture_debug_hold_pose()
        joint_count = min(len(self.debug_hold_q), len(self.low_cmd.motor_cmd))
        q_by_joint = dict(zip(UPPER_BODY_JOINTS, q_list))
        with self.cmd_lock:
            for joint in range(joint_count):
                mc = self.low_cmd.motor_cmd[joint]
                mc.mode = 1
                mc.dq = 0.0
                mc.tau = 0.0
                if joint in UPPER_BODY_JOINT_SET:
                    mc.q = q_by_joint[joint]
                    mc.kp = KP_PLAYBACK
                    mc.kd = KD_PLAYBACK
                else:
                    mc.q = self.debug_hold_q[joint]
                    mc.kp = KP_DEBUG_HOLD
                    mc.kd = KD_DEBUG_HOLD

    def set_playback_pose(self, q_list):
        if self.control_mode == CONTROL_MODE_MOTION:
            self._set_motion_playback_pose(q_list)
        else:
            self._set_debug_playback_pose(q_list)

    def set_arm_sdk_weight(self, weight):
        if self.control_mode != CONTROL_MODE_MOTION:
            return
        with self.cmd_lock:
            self.low_cmd.motor_cmd[ARM_SDK_WEIGHT_JOINT].q = weight
        self.current_weight = weight

    def ramp_arm_sdk_weight(self, start_weight, end_weight):
        if self.control_mode != CONTROL_MODE_MOTION:
            return
        steps = max(2, int(self.transition_time / self.publish_dt))
        for step_idx in range(steps + 1):
            ratio = step_idx / steps
            weight = start_weight + (end_weight - start_weight) * ratio
            self.set_arm_sdk_weight(weight)
            time.sleep(self.publish_dt)

    def enter_zero_torque_mode(self):
        if self.control_mode == CONTROL_MODE_DEBUG:
            self.capture_debug_hold_pose()
        self.set_zero_torque_pose()
        if self.control_mode == CONTROL_MODE_MOTION:
            self.ramp_arm_sdk_weight(self.current_weight, 1.0)

    def enter_playback_mode(self, first_q):
        if self.control_mode == CONTROL_MODE_DEBUG:
            self.capture_debug_hold_pose()
        self.set_playback_pose(first_q)
        if self.control_mode == CONTROL_MODE_MOTION and self.current_weight < 1.0:
            self.ramp_arm_sdk_weight(self.current_weight, 1.0)

    def exit_control(self):
        if self.control_mode == CONTROL_MODE_MOTION:
            self.ramp_arm_sdk_weight(self.current_weight, 0.0)
        else:
            self.set_zero_torque_pose()

    def record(self):
        self.set_zero_torque_pose()
        monitor = EnterMonitor("录制中。再次按下 Enter 停止录制并进入预备播放状态...\n")
        monitor.start()

        started_at = time.time()
        next_sample = started_at
        samples = []

        while not monitor.is_set():
            now = time.time()
            if now < next_sample:
                time.sleep(min(0.001, next_sample - now))
                continue

            q_list = self.snapshot_upper_body()
            samples.append({"t": now - started_at, "q": q_list})
            next_sample += self.record_dt

        if not samples:
            samples.append({"t": 0.0, "q": self.snapshot_upper_body()})
        return samples

    def save_record(self, trigger_word, samples):
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        out_path = OUT_DIR / f"arm_record_{self.control_mode}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        payload = {
            "format_version": 1,
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "trigger_word": trigger_word,
            "record_mode": self.control_mode,
            "joint_ids": UPPER_BODY_JOINTS,
            "joint_names": [JOINT_NAMES[j] for j in UPPER_BODY_JOINTS],
            "record_dt": self.record_dt,
            "playback_dt": self.playback_dt,
            "sample_count": len(samples),
            "samples": samples,
        }
        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.last_record_file = out_path
        return out_path

    def playback(self, samples):
        if not samples:
            raise RuntimeError("没有可播放的轨迹样本。")

        monitor = EnterMonitor("播放中。按下 Enter 可立即停止播放并退出当前控制...\n")
        monitor.start()

        started_at = time.time()
        for sample in samples:
            if monitor.is_set():
                return False

            target_time = started_at + sample["t"]
            while time.time() < target_time:
                if monitor.is_set():
                    return False
                time.sleep(0.001)

            self.set_playback_pose(sample["q"])

        final_pose = deepcopy(samples[-1]["q"])
        hold_until = time.time() + max(self.playback_dt, 0.2)
        while time.time() < hold_until:
            if monitor.is_set():
                return False
            self.set_playback_pose(final_pose)
            time.sleep(self.playback_dt)

        return True


def build_argparser():
    parser = argparse.ArgumentParser(
        description="H2 上半身录制与回放脚本",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=(CONTROL_MODE_MOTION, CONTROL_MODE_DEBUG),
        help="motion 使用 armSDK，debug 使用 lowcmd；该参数必选",
    )
    parser.add_argument(
        "--play",
        type=Path,
        help="直接播放指定 JSON 轨迹文件；启用后不进行录制",
    )
    parser.add_argument(
        "--iface",
        default=None,
        help="DDS 网卡名，不填则自动探测 192.168.123.x 网卡",
    )
    parser.add_argument("--publish-dt", type=float, default=0.01, help="发布周期，单位秒")
    parser.add_argument("--record-dt", type=float, default=0.02, help="录制采样周期，单位秒")
    parser.add_argument("--playback-dt", type=float, default=0.02, help="播放保持周期，单位秒")
    parser.add_argument("--transition-time", type=float, default=1.0, help="motion 模式进入/退出 ArmSDK 的权重过渡时间，单位秒")
    return parser


def main():
    parser = build_argparser()
    args = parser.parse_args()
    if args.mode is None:
        parser.print_help()
        return 1

    iface = args.iface or detect_iface()
    if not iface:
        print("未自动探测到 192.168.123.x 网卡，请使用 --iface 手动指定。", flush=True)
        return 1

    play_payload = None
    play_samples = None
    if args.play is not None:
        try:
            play_payload, play_samples = load_trajectory_file(args.play)
        except ValueError as exc:
            print(f"播放文件格式检查失败: {exc}", flush=True)
            return 1
        print(
            f"播放文件格式检查通过: {args.play}，共 {len(play_samples)} 帧。",
            flush=True,
        )

    print("WARNING: 请确保机器人周围无障碍，且操作者可随时接管。", flush=True)
    player = RecorderPlayer(
        control_mode=args.mode,
        iface=iface,
        publish_dt=args.publish_dt,
        record_dt=args.record_dt,
        playback_dt=args.playback_dt,
        transition_time=args.transition_time,
    )

    try:
        player.init()
        player.wait_for_low_state()
        player.start_publisher()

        if play_samples is not None:
            input("轨迹检查完成。按下 Enter 开始播放动作...\n")
            player.enter_playback_mode(play_samples[0]["q"])
            finished = player.playback(play_samples)
            source_text = args.play
        else:
            trigger_word = input("请输入一个词并回车，用于确认本次录制: ").strip()
            if not trigger_word:
                print("未输入确认词，程序退出。", flush=True)
                return 1

            input("按下 Enter 开始录制。开始后进入零力矩示教状态...\n")
            player.enter_zero_torque_mode()
            samples = player.record()
            record_path = player.save_record(trigger_word, samples)

            print(f"录制完成，共 {len(samples)} 帧。轨迹已保存到: {record_path}", flush=True)
            input("已进入预备播放状态。按下 Enter 开始播放录制动作...\n")
            player.enter_playback_mode(samples[0]["q"])
            finished = player.playback(samples)
            source_text = record_path

        if finished:
            print(f"播放结束，准备退出当前控制。轨迹来源: {source_text}", flush=True)
        else:
            print(f"检测到播放中断请求，准备退出当前控制。轨迹来源: {source_text}", flush=True)

        player.exit_control()
        if args.mode == CONTROL_MODE_MOTION:
            print("已退出 ArmSDK。", flush=True)
        else:
            print("已停止 lowcmd 控制。", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，准备退出当前控制。", flush=True)
        try:
            player.exit_control()
        except Exception:
            pass
        return 1
    except Exception as exc:
        print(f"程序异常: {exc}", flush=True)
        try:
            player.exit_control()
        except Exception:
            pass
        return 1
    finally:
        if "player" in locals():
            player.shutdown()


if __name__ == "__main__":
    sys.exit(main())
