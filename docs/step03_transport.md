# Step 3: diagnostic Quest → lab link

**DIAGNOSTIC ONLY: ROBOT NOT CONNECTED.** No component opens a robot or gripper connection, imports a robot SDK or ROS, or has an option to enable robot control.

```
Quest ──USB/adb logcat──▶ quest_stream_sender.py ──TCP 127.0.0.1:18765──▶ ssh -L (dedicated tunnel)
                                                                              │
                              lab: quest_stream_receiver.py ◀── 127.0.0.1:28765 ┘
```

## Components
| File | Where | Purpose |
|---|---|---|
| `scripts/quest_input_monitor.py` | both | Step 2 acquisition/parser. The sender reuses it through optional `on_message`/`on_error`/`on_transport` hooks. The receiver imports only `check_pose` and `CONTROLLER_KEYS`. |
| `scripts/quest_transport.py` | both | Framing, strict decode, field validation, input-state rule, stats. |
| `scripts/quest_stream_sender.py` | local | Live acquisition, a latest-sample slot, and a rate-limited TCP sender. |
| `scripts/quest_stream_receiver.py` | lab | A loopback-only diagnostic receiver with a watchdog display. |
| `tests/test_quest_transport.py` | both | Synthetic offline tests. The Sender tests run locally only. |

## Wire format (protocol v1)
Packets are newline-delimited JSON: one UTF-8 JSON object per line.
- **Size:** at most **16 KiB** per frame, including the newline. A longer frame drops the connection.
- **Numbers:** `NaN`, `Infinity` and overflowing literals (such as `1e999`) are rejected on encode and on decode.
- **Safety:** nothing is evaluated or unpickled.
- **Reads:** partial TCP reads and several frames in one read are handled by `LineFramer`.

Every packet has `v` (1) and `type`. After the handshake, every sender packet also has:
- `session`: a new random id per TCP connection.
- `seq`: starts at 1 and increases by exactly 1.
- `source_session`: a new random id per sender process, identifying the acquisition run.
- `send_mono_ns`: the sender host's `time.monotonic_ns()`.

| type | direction | fields |
|---|---|---|
| `hello` | S→R | `session`, `source_session`, `role`, `frame`, `tracking_validity` |
| `hello_ack` | R→S | `session`, `receiver`, `diagnostic_only: true`, `robot_connected: false` |
| `sample` | S→R | see below |
| `invalidate` | S→R | `reason`, `after_source_seq`, `source_recv_mono_ns`, `detail`, `source` |
| `heartbeat` | S→R | `source` {`up`, `note`, `last_source_seq`, `age_ms`} |
| `bye` | S→R | none |
| `ack` | R→S | `session`, `seq`, `status` (`ok` or `rejected`), `reason` |
| `error` | R→S | `reason` (sent for "busy", a packet before hello, or the wrong session) |

### `sample`
One source message, sent unchanged:
- `source_seq`: the Step 2 monitor's valid-message counter.
- `source_recv_mono_ns`: the sender host's time when it read the log line. This is **not** a Quest sensor-capture time.
- `device_log_time`: the device clock time when the line was written to the log.
- `frame`: always `"headset_relative_observed"`.
- `tracking_validity`: always `"unknown"`.
- `poses`: `{"r": 4x4 | null, "l": 4x4 | null}` in the unchanged source frame, row-major. Both keys are required, and a missing pose is `null`.
- `pose_present`, `pose_validation`: presence and homogeneous-check results.
- `buttons`: exactly the fields that were present. Booleans stay booleans and analog values stay lists. An absent field stays absent.
- `input_state`: `PRESENT` or `MISSING`.
- `source`: {`up`, `note`, `last_source_seq`}.

Missing values are never replaced with zero, identity, the other controller's pose, or an earlier message.

**Input rule:** `PRESENT` means the right-controller pose (`REQUIRED_CONTROLLERS = ("r",)`) is structurally present and valid.
It does **not** mean physical tracking is verified, and it does **not** permit robot control.

## Sender behaviour
- **Acquisition:** one adb logcat reader (Step 2 `AdbLogcatSource`) runs in its own thread and writes into a **latest-sample slot**. The slot holds one sample, plus a bounded list of invalidation events (64; the oldest are dropped first and counted).
- **Send rate:** send opportunities occur at `--max-rate` (default **30 Hz**). This is a transport-test setting, not a robot servo rate. At each opportunity the sender sends:
  1. Every pending invalidation event, oldest first.
  2. Then the newest sample, but only if its `source_seq` is higher than the last one sent. A cached sample is never re-sent.
  3. If neither of those happened, a heartbeat every 200 ms.
- **Skips:** source messages that are replaced before a send opportunity count as `source_frames_skipped`. These are intentional and are not errors.
- **Invalidation:** each of the following clears the slot (or replaces it with the missing sample) and queues an invalidation event. Consecutive identical reasons are coalesced.
  - An empty message: `&`, for example while a system menu has focus.
  - A message without the right pose.
  - A parse error.
  - An adb or app loss.
- **Heartbeats** carry source status only. They are not counted as input.
- **Samples from before the session** are never sent: the session starts at the slot's current `source_seq`.
- **Timeouts:**
  - Connect: 3 s.
  - Handshake: 3 s.
  - Each `sendall`: 1 s.
  - Oldest unacknowledged packet: 2 s.
  - At most 256 unacknowledged packets.
  - Kernel send buffer: 32 KiB.
- **Failure:** any of these timeouts, or EOF or a socket error, ends the session. Pending data is discarded and the process exits with code 4. There is **no automatic reconnect**. You reconnect by running the sender again, which starts a new session. Nothing is replayed.

## Receiver behaviour
- **Binding:** it binds **127.0.0.1 only** (hard-coded; there is no host option). The port defaults to 8765. On the lab computer 8765 was already in use by another loopback listener, which was left untouched, so this deployment uses `--port 28765`.
- **Clients:** one client at a time. A second client receives `error: busy` and is closed.
- **Session rules:**
  - The first packet must be `hello`.
  - `seq` must increase. A duplicate or out-of-order packet is rejected and acknowledged as `rejected`, and the current input is kept.
  - `source_seq` must increase within the session. A duplicate or out-of-order sample is rejected and invalidates the input.
  - Each new connection resets all state. No input carries over from a previous session.
- **Validation:** a malformed or invalid sample (non-finite value, bad shape, fields that disagree) is rejected and **clears** the current input.
  An `invalidate` packet, or a sample without the right pose, clears the input **immediately**.
  EOF or a socket error clears it and shows DISCONNECTED.
- **Watchdog:** the display loop runs in the main thread every 0.5 s using the receiver's clock. The socket handler thread may be blocked in `recv`, and staleness is still detected.

### Thresholds (diagnostic, NOT approved robot safety timeouts)
| Option | Default | Measures (all on the receiver's clock) |
|---|---|---|
| `--transport-stale-ms` | 500 | Time since **any** packet (sample, invalidate or heartbeat) arrived on the current connection. Above the limit, transport shows STALE. |
| `--input-stale-ms` | 500 | Time since the receiver last accepted a **new valid PRESENT sample**. Heartbeats never refresh it. Above the limit, input shows STALE. |

Status values:
- **Transport:** `LIVE`, `STALE` or `DISCONNECTED`.
- **Input:**
  - `PRESENT`
  - `MISSING`: explicitly missing or invalidated, or there is no session.
  - `STALE`
  - `SOURCE_DISCONNECTED`: the sender reports that adb or the app is down.

## Timing: what is measured and what is not
| Metric | Clock | Meaning |
|---|---|---|
| Application RTT | sender | From the sender writing a packet to the sender reading its ack. It includes the tunnel in both directions and receiver processing. |
| Acquisition-to-send age | sender | From the host reading the log line to the sender writing the sample. It does not include time inside the Quest, adb or the USB link. |
| Receiver inter-arrival gaps | receiver | Gaps between consecutive valid samples (and between all packets) as seen on the lab host. |

Limitations:
- Monotonic timestamps from different hosts are never subtracted.
- RTT/2 is **not** a one-way latency.
- Regular arrival at the receiver does **not** prove that samples weren't delayed inside adb, SSH or TCP.

## Commands
The lab SSH alias is `linux2` (from your existing SSH config, which is not modified). The tunnel is dedicated: it doesn't reuse a ControlMaster connection, it forwards a single loopback port, and it runs no remote command.

```bash
# 1. Lab receiver (on the lab computer, e.g. in its own terminal via your normal SSH session)
cd ~/quest-xarm-teleop-dryrun/scripts && python3 quest_stream_receiver.py --port 28765 \
    --log ../logs/rx_<name>.jsonl --summary ../logs/rx_<name>.summary.json

# 2. Dedicated tunnel (local computer)
ssh -N -o ControlMaster=no -o ControlPath=none -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
    -L 127.0.0.1:18765:127.0.0.1:28765 linux2

# 3. Local sender (local computer)
cd ~/quest-xarm-teleop/scripts && python3 quest_stream_sender.py --serial <SERIAL> --port 18765 \
    --max-rate 30 --log ../data/step03/tx_<name>.jsonl --summary ../data/step03/tx_<name>.summary.json
```

The tunnel's listening port alone does not prove the receiver is reachable. The sender only starts streaming after a `hello`/`hello_ack` exchange with `robot_connected: false`.
