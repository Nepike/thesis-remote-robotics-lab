# Remote robotics laboratory

A server that lets people drive real robots over the network. Clients connect by WebSocket,
take out locks on physical devices, queue commands with priorities, stream telemetry back, and
launch group manoeuvres that run entirely server-side. Overhead cameras track every robot by
ArUco marker, so the server always knows where things actually are.

> Undergraduate thesis project, MIPT, 2026. The written thesis (in Russian) is in
> [`docs/thesis-ru.pdf`](docs/thesis-ru.pdf).

## The problem

A teaching lab has a handful of robots and many more students than robots. Physical presence
does not scale, and neither does "whoever is holding the joystick wins". What was needed was
a programmatic way in: a client library that hands you a robot object, guarantees nobody else
is driving it at the same time, and behaves predictably when two people ask for the same
hardware at once.

## Architecture

```
   Python client                 Manager (one process)                    Hardware
 ───────────────────      ───────────────────────────────────      ────────────────────
                          ┌──────────────────────────────┐
  RemoteLab ──WebSocket──▶│  FastAPI + ws_handler        │
    .device(name)         │  auth, sessions              │
    .lock()               ├──────────────────────────────┤
    .submit(cmd) ────────▶│  AccessController  (locks)   │
    .subscribe_telemetry  │  CommandScheduler  (queues)  │        ┌── RosInterface ──▶ ROS ──▶ robots
    .run_procedure ──────▶│  ProcedureManager            │───────▶│
                          │  DeviceSupervisor  (health)  │        └── SerialInterface ──▶ ESP32 / Arduino
                          ├──────────────────────────────┤
            telemetry ◀───│  Drivers: Yarp13, SimpleSerial│
                          └──────────────┬───────────────┘
                                         │
                              Navigation │ ArUco + Kalman + A*
                                         ▼
                                  overhead cameras
```

Everything lives in one asyncio process. That is a deliberate constraint, not an accident:
device ownership, command queues and procedure state are all in-memory and would need
distributed locking the moment a second worker appeared. `network/server.py` says as much —
do not run uvicorn with `--workers > 1`.

### Concurrency model

| Concern | How it is handled |
|---|---|
| Two clients, one robot | `AccessController` — devices are `exclusive` (must be locked) or `shared` (free-for-all). Locks are released automatically when a client disconnects |
| Ordering | `CommandScheduler` — one queue per device, priority-ordered, with a worker per device |
| Long commands | Commands are awaitable handles. A client may fire-and-forget, `await` completion, wait with a timeout, or `interrupt()` what is running now |
| Dead hardware | `DeviceSupervisor` watches telemetry liveness and marks devices down |
| Group manoeuvres | Procedures run *on the server*, acquire the devices they need themselves, and are cancellable |

Three procedures ship with it: `stop_all` (emergency stop for the whole lab), `sync_test`
(a synchronised parade that also verifies every robot is reporting telemetry) and
`all_go_home` (each robot returns to its home cell, avoiding obstacles).

### Navigation

`all_go_home` is where the pieces meet. Cameras above the arena see ArUco markers in three
ID ranges — robots, fixed anchors, and obstacles. The anchors give a homography recomputed
every frame, so nobody has to click calibration points by hand. Poses from all cameras are
fused in an unscented Kalman filter; obstacle markers are dropped into an occupancy grid;
A\* plans a route and a waypoint controller drives it.

Setup, marker ID allocation and printing are documented in
[`manager/navigation/ARUCO_SETUP.md`](manager/navigation/ARUCO_SETUP.md) (in Russian).

## Layout

| Path | What it is |
|---|---|
| `manager/` | The server. Orchestrator, scheduler, access control, drivers, hardware interfaces |
| `manager/network/` | FastAPI app, WebSocket protocol, authentication, request models |
| `manager/navigation/` | ArUco detection, multi-camera fusion, localisation, A\* and marker generation |
| `client/` | The Python client library — connection handling, device handles, awaitable commands |
| `client/example.py` | A runnable tour of every client feature, doubling as documentation |
| `firmware/yarp13-mega/` | Robot firmware: motor PID, encoders, compass, rangefinders, servos, reflex stop |
| `firmware/esp32-gateway/` | Wi-Fi gateway between the robot and the server |
| `firmware/custom-protocol/` | A binary serial protocol with typed arguments and CRC, plus master/slave examples |
| `docs/` | The thesis |

### The wire protocol

`firmware/custom-protocol/` is a self-contained library for talking to a microcontroller over
a serial line. Packets are `[start][command][type flags][length][payload][CRC]`, arguments
carry their own types (`bool`, `uint8`, `int16`, `uint16`, `float`, `string`), and both ends
share a single header. It sits beside the ROS path rather than under it, and it is the part
of the project closest to plain embedded C++.

## Running it

The server needs ROS Noetic on Ubuntu 20.04 and a camera. The ROS workspace itself belongs to
the laboratory and is not redistributed here; the drivers expect it built at `~/ros`, and
source it as `~/ros/devel/setup.bash`.

```bash
cd manager
pip install -r requirements.txt
python network/server.py --host 0.0.0.0 --port 8000
```

On first start the server writes `devices.json` and `users.json` templates next to itself and
stops — both are per-deployment data, kept out of git so that updating the code never wipes
the registered users and hardware.

Then, from a client machine:

```bash
python client/example.py
```

which prints what is connected, the command catalogue for each driver type, and then walks
through telemetry, exclusive access, group commands, a server-side procedure and an emergency
stop.

Driving a robot takes about as much code as you would hope:

```python
async with RemoteLab("lab.host:8000", "user", "password") as lab:
    robot = lab.device("yarp13-01")
    async with robot.lock():
        cmd = await robot.submit("move", speed_lin=0.2, duration=3.0)
        await cmd
```

## Adding a device

Devices are data, not code, up to the point where the protocol differs. A new robot of an
existing type is a row in `devices.json`. A genuinely new type means a driver subclassing
`RosBasedDriver` or `SerialBasedDriver`, registered in `DriverFactory` under a string key —
after which the client's command catalogue picks it up without touching client logic.

## Notes

- The ROS workspace is the laboratory's, not mine, and is not included. Everything in this
  repository is my own work.
- Comments and inline documentation are a mix of English and Russian.
- There is no automated test suite; the system was validated against the physical lab, and
  `sync_test` exists partly as a smoke test you can run on real hardware.
