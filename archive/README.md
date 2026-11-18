# Archive

One-off test sessions and analyses from building the xArm teleop (2026-09-30 to 2026-10-12). Nothing in daily use
runs them, `scripts/teleop sync` does not install them, and no test imports them. They are kept because the
reports in `reports/` were made with them and name them (with their old paths, `scripts/...` and `scripts/ops/...`).

| Path | Named in |
|---|---|
| `scripts/analyze_step04b_test2.py`, `test3.py`, `test4.py` | `reports/step04b_test2_*`, `step04b_test3_*`, `step04b_test4_*` |
| `scripts/step04b_test2_cues.py` | `reports/step04b_test2_*` to `step04b_test5_*` |
| `scripts/analyze_checkpoint5_live.py`, `analyze_recenter_direction.py` | `reports/checkpoint5_*`, `docs/checkpoint5_pose_transport_v2.md` |
| `scripts/analyze_teleop_dryrun.py` | `reports/checkpoint6_teleop_adapter_dryrun_*` |
| `scripts/camera_feed_check.py` | `reports/checkpoint7_*` |
| `ops/gripper_live_session.sh` | `reports/checkpoint8_*` |
| `ops/episode_to_mp4.sh` | `reports/checkpoint11_*` |
| `scripts/analyze_live_teleop.py`, `ops/forehead_dryrun_session.sh`, `ops/gripper_forehead_session.sh`, `ops/lab_first_move_run.sh`, `ops/lab_gripper_step_run.sh`, `ops/lab_run_detached.sh`, `ops/link_video_ab.sh` | none; the sessions of checkpoints 6-10 |

The Python ones still import the shared modules from `scripts/` (run them from the repository root, e.g.
`python3 archive/scripts/analyze_checkpoint5_live.py ...`; `step04b_test2_cues.py` needs `PYTHONPATH=scripts`).
The shell ones name programs by their `scripts/...` paths, which have not moved; run them from the repository root.
