"""Bimanual YAM: two SIMULATED arms (I2RT's MuJoCo models) behind the interface yam_control drives. NO HARDWARE.

Runs on the lab host in I2RT's environment (/home/dev/yam/i2rt/.venv/bin/python). Every robot is created with
sim=True and no CAN link is opened; there is deliberately no path to the real arms in this file.

Note on the simulator: I2RT's SimRobot puts the joints straight at each command (no motor dynamics; its physics
thread is not started here), so a simulation shows what the control decides, not how smoothly real motors
would follow it.

Kinematics: the IK is a damped least-squares step loop (mink, as in I2RT's Kinematics.ik) toward the tool
target, started from the arm's last command and kept inside the joint ranges. Clearance: one MuJoCo model holds
both arms at their bases and the table; a pair of parts is a problem when
  - the two arms come closer than lim.clearance,
  - an arm body other than its base comes closer to the table than lim.clearance (fingertips: touching is
    allowed, pressing in is not),
  - two parts of the same arm overlap by more than 1 mm (neighbouring links, and the gripper's own parts, are
    exempt: their simplified shapes always touch).
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import mink
import mujoco
import numpy as np
from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType

from yam_setup import ArmSetup, Limits

SITE = "grasp_site"
N_ARM = 6
GRIPPER_PARTS = {"gripper", "tip_left", "tip_right"}
TIPS = {"tip_left", "tip_right"}
SELF_OVERLAP = -0.001


def _yaw_T(xyz: Sequence[float], yaw: float) -> np.ndarray:
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = xyz
    return T


def pose_to_T(pose: Sequence) -> np.ndarray:
    (x, y, z), (qx, qy, qz, qw) = pose
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                 [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                 [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]  # fmt: skip
    T[:3, 3] = (x, y, z)
    return T


def T_to_pose(T: np.ndarray) -> tuple:
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(T[:3, :3]).flatten())  # (w, x, y, z)
    return (tuple(float(v) for v in T[:3, 3]), (float(q[1]), float(q[2]), float(q[3]), float(q[0])))


class SimYamPair:
    sim = True

    def __init__(self, arms: Sequence[ArmSetup], lim: Limits, arm_type: str = "YAM", gripper_type: str = "LINEAR_4310",
                 ik_iters: int = 20):  # fmt: skip
        self.lim, self.ik_iters = lim, ik_iters
        self.arms = {a.name: a for a in arms}
        self.names = tuple(self.arms)
        self.robots, self.cfg, self.T_wb, self.T_bw = {}, {}, {}, {}
        for a in arms:
            r = get_yam_robot(channel=a.channel, arm_type=ArmType[arm_type], gripper_type=GripperType[gripper_type], sim=True)
            if not r.get_robot_info().get("sim"):
                raise RuntimeError("expected a simulated robot")  # never reached: sim=True above
            self.robots[a.name] = r
            self.T_wb[a.name] = _yaw_T(a.base_xyz, a.base_yaw)
            self.T_bw[a.name] = np.linalg.inv(self.T_wb[a.name])
        self.xml = self.robots[self.names[0]].xml_path
        self.model = mujoco.MjModel.from_xml_path(self.xml)
        for a in arms:
            self.cfg[a.name] = mink.Configuration(self.model)
        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        self.ranges = [tuple(float(v) for v in self.model.jnt_range[i]) for i in range(N_ARM)]
        self.slide = tuple(float(v) for v in self.model.jnt_range[N_ARM])  # gripper finger slide, m
        self._build_clearance_model()

    # --- the interface yam_control uses ----------------------------------------------------------------------
    def joint_ranges(self, name: str) -> list[tuple[float, float]]:
        return list(self.ranges)

    def joints(self, name: str) -> list[float]:
        return [float(v) for v in self.robots[name].get_joint_pos()]

    def command(self, name: str, q7: Sequence[float]) -> None:
        self.robots[name].command_joint_pos(np.asarray(q7, dtype=float))

    def _q_model(self, q6: Sequence[float], grip: float = 0.0) -> np.ndarray:
        q = np.zeros(self.model.nq)
        q[:N_ARM] = q6
        q[N_ARM] = q[N_ARM + 1] = self.slide[0] + grip * (self.slide[1] - self.slide[0])
        return q

    def fk(self, name: str, q6: Sequence[float]) -> tuple:
        c = self.cfg[name]
        c.update(self._q_model(q6))
        T_b = c.get_transform_frame_to_world(SITE, "site").as_matrix()
        return T_to_pose(self.T_wb[name] @ T_b)

    def solve(self, name: str, pose: Sequence, q6_init: Sequence[float]) -> tuple[list[float], float, float]:
        c = self.cfg[name]
        c.update(self._q_model(q6_init))
        task = mink.FrameTask(frame_name=SITE, frame_type="site", position_cost=1.0, orientation_cost=1.0, lm_damping=1.0)
        task.set_target(mink.SE3.from_matrix(self.T_bw[name] @ pose_to_T(pose)))
        err = task.compute_error(c)
        for _ in range(self.ik_iters):
            if np.linalg.norm(err[:3]) <= 1e-4 and np.linalg.norm(err[3:]) <= 1e-4:
                break
            vel = mink.solve_ik(c, [task], 0.01, "quadprog", damping=1e-4, limits=self.ik_limits)
            c.integrate_inplace(vel, 0.01)
            err = task.compute_error(c)
        return [float(v) for v in c.q[:N_ARM]], float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))

    def clearance_problems(self, q_by_name: Mapping[str, Sequence[float]]) -> dict[str, float]:
        m, d = self.cm, self.cd
        for n, q in q_by_name.items():
            adr = self.cm_qadr[n]
            g = q[N_ARM] if len(q) > N_ARM else 0.0
            d.qpos[adr] = self._q_model(q[:N_ARM], g)
        mujoco.mj_forward(m, d)
        out: dict[str, float] = {}
        for c in d.contact[: d.ncon]:
            a, b = self.cm_part[c.geom1], self.cm_part[c.geom2]
            if a > b:
                a, b = b, a
            what = self._judge(a, b, float(c.dist))
            if what and (what not in out or c.dist < out[what]):
                out[what] = float(c.dist)
        return out

    # --- simulation only -------------------------------------------------------------------------------------
    def place(self, name: str, q7: Sequence[float]) -> None:
        """Put a simulated arm somewhere (test set-up). Not part of the arm interface."""
        self.command(name, q7)

    def close(self) -> None:
        for r in self.robots.values():
            r.close()

    # --- clearance model ---------------------------------------------------------------------------------------
    def _build_clearance_model(self) -> None:
        spec = mujoco.MjSpec()
        spec.worldbody.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1])
        for n, a in self.arms.items():
            child = mujoco.MjSpec.from_file(self.xml)
            q = np.empty(4)
            mujoco.mju_mat2Quat(q, np.ascontiguousarray(self.T_wb[n][:3, :3]).flatten())
            frame = spec.worldbody.add_frame(pos=list(a.base_xyz), quat=list(q))
            frame.attach_body(child.body("base"), n + "/", "")
        self.cm = spec.compile()
        self.cm.geom_margin[:] = self.lim.clearance / 2  # contacts are reported from (sum of margins) apart
        self.cd = mujoco.MjData(self.cm)
        name = lambda kind, i: mujoco.mj_id2name(self.cm, kind, i)  # noqa: E731
        self.cm_part = []  # per geom: (arm or "", body) e.g. ("left", "link3"), ("", "table")
        for g in range(self.cm.ngeom):
            b = name(mujoco.mjtObj.mjOBJ_BODY, self.cm.geom_bodyid[g])
            self.cm_part.append(tuple(b.split("/", 1)) if "/" in b else ("", "table"))
        self.cm_parent = {}
        for b in range(1, self.cm.nbody):
            self.cm_parent[name(mujoco.mjtObj.mjOBJ_BODY, b)] = name(mujoco.mjtObj.mjOBJ_BODY, self.cm.body_parentid[b])
        self.cm_qadr = {}
        for n in self.arms:
            self.cm_qadr[n] = [self.cm.jnt_qposadr[mujoco.mj_name2id(self.cm, mujoco.mjtObj.mjOBJ_JOINT, f"{n}/joint{i + 1}")]
                               for i in range(self.model.nq)]  # fmt: skip

    def _judge(self, a: tuple, b: tuple, dist: float) -> str:
        (arm_a, part_a), (arm_b, part_b) = a, b
        if arm_a == "":  # the table (sorted first)
            if part_b == "base":
                return ""
            if part_b in TIPS:
                return f"{arm_b} fingertip pressing into the table" if dist < 0 else ""
            return f"{arm_b} {part_b} and the table" if dist < self.lim.clearance else ""
        if arm_a != arm_b:
            return f"{arm_a} {part_a} and {arm_b} {part_b}" if dist < self.lim.clearance else ""
        fa, fb = f"{arm_a}/{part_a}", f"{arm_b}/{part_b}"
        if self.cm_parent.get(fa) == fb or self.cm_parent.get(fb) == fa:
            return ""
        if part_a in GRIPPER_PARTS and part_b in GRIPPER_PARTS:
            return ""
        return f"{arm_a} arm: {part_a} and {part_b} overlap" if dist < SELF_OVERLAP else ""
