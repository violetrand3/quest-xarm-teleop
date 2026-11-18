# Checkpoint 5: diagnostic pose link, protocol v2 (`quest-pose-link`)

**DIAGNOSTIC ONLY: ROBOT NOT CONNECTED.** No component imports a robot or gripper SDK, opens a robot or gripper connection, or has an option to enable motion. "Input available" is a statement about the Quest input stream. It is not robot readiness.

```
Quest (org.questxarm.reader, tag QuestXarmPoseV1)
   │ USB, adb logcat (one owned child per acquisition generation)
   ▼
quest_pose_stream_sender.py ──TCP 127.0.0.1:18765──▶ ssh -L (dedicated tunnel) ──▶ 127.0.0.1:28765
   every record → ContinuityTracker → 30 Hz limiter                quest_pose_stream_receiver.py (lab)
```

Protocol v1 (`docs/step03_transport.md`, legacy app, world-frame matrices) is unchanged and still works. The two versions are not interchangeable: a v1 client is refused by the v2 receiver and the reverse.

## Components
| File | Where | Purpose |
|---|---|---|
| `scripts/quest_pose_v1.py` | both | Decoder for `QuestXarmPoseV1` telemetry (Step 4A). |
| `scripts/quest_pose_transport.py` | both | v2 constants, strict decode, raw right-input rule, `ContinuityTracker`, packet build and validation. |
| `scripts/quest_pose_stream_sender.py` | local | Owned logcat acquisition, per-record processing, rate limiter, sender session. |
| `scripts/quest_pose_stream_receiver.py` | lab | Loopback-only receiver, identity and generation checks, watchdog display. |
| `scripts/quest_transport.py`, `quest_stream_sender.py`, `quest_stream_receiver.py` | both / local / lab | v1 framing, session and server code, reused through small hooks (`proto_version`, `_decode`, `_hello_packet`, `decode_fn`, `log_fields`). |
| `archive/scripts/analyze_checkpoint5_live.py` | local | Offline analysis of one live run. |
| `tests/test_quest_pose_transport.py`, `tests/fixtures/fake_adb.py` | both | Synthetic offline tests. Sender and acquisition tests run locally only. |

## Wire format
Framing is the same as v1: newline-delimited JSON, at most 16 KiB per frame, `NaN`/`Infinity` rejected, nothing evaluated. Every packet has `v` = **2** and `type`. After the handshake every sender packet has `session`, `seq` (starts at 1, +1 each), `send_mono_ns` (sender host monotonic clock).

| type | direction | fields |
|---|---|---|
| `hello` | S→R | `session`, `sender_run`, `protocol` = `quest-pose-link`, `source_format` = `QuestXarmPoseV1`, `source_schema` = 1, `frame` = `openxr_local`, `role` |
| `hello_ack` | R→S | `session`, `receiver`, `diagnostic_only: true`, `robot_connected: false` |
| `pose` | S→R | see below |
| `source_event` | S→R | `acq_gen`, `event` (the source event record, unmodified), `source_recv_mono_ns` |
| `status` | S→R | `acq_gen`, `source_up`, `source_note`, `stale`, `sid`, `last_source_seq`, `source_age_ms`, `continuity` |
| `bye` | S→R | none |
| `ack` | R→S | `seq`, `status` (`ok`/`rejected`), `reason`, `continuous` (pose only) |
| `error` | R→S | `reason` |

The receiver refuses a `hello` whose protocol, source format, source schema or frame differs from its own.

### `pose`
```
frame        "openxr_local"   XR_REFERENCE_SPACE_TYPE_LOCAL, unmodified
sender_run   random id per sender process
acq_gen      acquisition generation (one owned logcat child each)
source       {build, sid, session_gen, seq, t_query_ns, t_query_kind: "predicted_display_time",
              recv_mono_ns, device_log_time}
continuity   {gen, loss_count, source_gap_records, breaks: [{reason, …}], breaks_not_listed,
              decimated_since_last}
raw_right    {available, reasons: [...]}
record       the original source JSON object, unmodified (null stays null)
matrices     {head, grip_l, grip_r}: 4x4 T_LOCAL_from_X built from the record's p/q, or null
status       {source_up, stale}
```
The receiver decodes `record` again with `quest_pose_v1.decode_record`, checks that `matrices` equal the record's own poses within 1e-9, and recomputes `raw_right` from the record. Any disagreement is a rejection. No coordinate mapping, smoothing or recenter compensation exists on either side.

`t_query_ns` is the predicted display time used for `xrLocateSpace`. It is not a sensor capture time. `recv_mono_ns` is when the sender host read the log line.

## Source identity
| Field | Lifetime | Meaning |
|---|---|---|
| `sender_run` | sender process | One run of the sender. |
| `acq_gen` | one logcat child | Increments on every acquisition start **and** abort. |
| `sid` | one app process | Generated in `AppInit`. It is **not** an OpenXR session id. |
| `session_gen` | per `sid` | Count of `session_begin` events seen for this `sid`. |
| `seq` | one app process | Shared by pose and event records, +1 per record. |
| `local_epoch` | one app process | Count of LOCAL `ReferenceSpaceChangePending` events the app received. Other space types don't increment it. It is never reset per session. |

The receiver keeps the current identity `(sender_run, acq_gen, build, sid, session_gen, local_epoch, local_change_time_ns)`. When `(acq_gen, sid)` changes, the old pair is **retired**: a later pose carrying it is rejected and never becomes current.

## Raw right-input availability
Input is available only when all of these hold in the same record: session focused, pose action active, position and orientation both VALID and both TRACKED, grip (squeeze) action active. Otherwise every concurrent reason is listed and the receiver hides the values. This is kept separate from continuity: input can be available on both sides of a break.

## Continuity
`ContinuityTracker` sees **every** decoded source record, in order, before the 30 Hz limiter. `continuity_gen` increments on every break and is never reset during a sender run.

| Break reason | Trigger |
|---|---|
| `acquisition_restart` | new acquisition generation |
| `app_run_changed` | `sid` changed |
| `session_begin`, `session_end`, `session_state:<S>` | OpenXR session events; any state other than FOCUSED |
| `source_gap` | source `seq` skipped (missing records of unknown type) |
| `parse_error`, `source_overflow`, `events_lost`, `event_queue_overflow` | undecodable line; app-side overflow record; lost events |
| `local_space_change`, `unknown_space_change` | LOCAL change event; change event of a type the app doesn't identify |
| `ref_counter_changed_event_absent`, `ref_change_time_changed_event_absent` | pose record shows a new `local_epoch` or change time although no event was seen |
| `input_lost:<reason>` | raw right input went from available to unavailable |
| `source_down:<note>`, `source_stale` | app not running or acquisition aborted; watchdog |

- `loss_count` counts input losses. `source_gap_records` counts missing source records. `decimated_since_last` counts poses intentionally skipped by the limiter. These three are never merged.
- Pending breaks ride on the next `pose` or `status` packet. If any are pending, a `status` packet is sent at once, so a break on a decimated record still reaches the receiver.
- The receiver marks a pose `continuous` only if it and the previous pose are both available, share the same `continuity_gen` and the same identity. A generation that goes backwards, or an identity change without a generation change, is a rejection.
- Nothing latches motion. There is no motion-enable anywhere.

## Watchdogs (diagnostic thresholds, not safety timeouts)
| Where | Default | Measures |
|---|---|---|
| sender `--stale-ms` | 500 ms | Sender-clock time since the last source record was read. Runs in the send loop, independent of the reader thread. On expiry: `source_stale`, input cleared. |
| receiver `--transport-stale-ms` | 500 ms | Receiver-clock time since any packet arrived. |
| receiver `--input-stale-ms` | 500 ms | Receiver-clock time since a pose with available input arrived. `status` packets never refresh it. |

A `status` packet with `source_up: false` or `stale: true` clears the receiver's input immediately.

## Acquisition ownership
The sender starts one `adb logcat` child per generation (tag filter, `-T <device time>`), attached before the app is launched. A 1 s `pidof` poll reports "app not running".
- `SIGUSR1`: abort the generation. The child is sent SIGKILL (this also works on a stopped child), reaped, its pipe closed and its reader thread joined. Lines tagged with an old generation are discarded and counted.
- `SIGUSR2`: start a new generation.
- A suspended child is never resumed. The shared adb server and unrelated Android logs are not touched.

## Commands
```
# lab (loopback only; port 8765 belongs to another program and is not used)
python3 scripts/quest_pose_stream_receiver.py --port 28765 --log logs/rx.jsonl --summary logs/rx.summary.json

# local: dedicated tunnel, then sender
ssh -N -o BatchMode=yes -o ControlMaster=no -o ControlPath=none -o ExitOnForwardFailure=yes \
    -L 127.0.0.1:18765:127.0.0.1:28765 linux2
python3 scripts/quest_pose_stream_sender.py --port 18765 --expect-build s4a-ff0da2692500 \
    --log tx.jsonl --source-log src.jsonl --summary tx.summary.json --acq-file acq.json

# offline
python3 -m unittest discover -s tests
python3 archive/scripts/analyze_checkpoint5_live.py data/checkpoint5 live_20261001-151605
```

## Test results (2026-10-01)
| Where | Python | Result |
|---|---|---|
| local | system python3 | **95 tests OK** (25 in `test_quest_pose_transport.py`) |
| lab `linux2` | 3.10.12 | **39 tests OK** (shared and receiver tests; sender tests are local only) |

The 25 v2 tests cover: version separation, telemetry and flag preservation, tamper rejection, single TRACKED bit, gap vs decimation, brief loss between transmitted poses, reference counter change with the event absent, session and reference events, restart generations, the watchdog, foreign builds and duplicates, incompatible hello, end-to-end values, identity change without a generation change, heartbeats not refreshing input, sender-reported stale/down, **old identity rejected after restart**, hidden values when unavailable, handshake, v1 client refused, every record processed before decimation, stale heartbeats, old-generation lines discarded, and a **suspended child detected, killed and replaced** (using the synthetic `fake_adb.py`).

Live results are in `reports/checkpoint5_input_connection_closeout_20261001.md`.

## Limits
- Arrival rate says nothing about absolute sample age. No Quest-to-host clock relation is established.
- RTT is an application round trip on the sender's clock. Half of it is not a one-way latency.
- Timestamps from different computers are never subtracted.
- The 500 ms thresholds were chosen for diagnosis. A robot path needs its own deadman and timeout design.
