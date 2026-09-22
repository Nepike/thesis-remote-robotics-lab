# Data class repository - describes the telemetry structure for each device type

from dataclasses import dataclass


@dataclass
class Yarp13Telemetry:
    # Encoders & derived speeds (ticks per sensor period)
    enc_left: int
    enc_right: int
    speed_left: int
    speed_right: int

    # Orientation
    compass: int    # heading, degrees
    pitch: float
    roll: float

    # Power
    acc_voltage: float

    # Range finders (cm): front, sides, rear
    rf_center: int
    rf_left: int
    rf_right: int
    rf_side_left_fwd: int
    rf_side_right_fwd: int
    rf_side_left_bck: int
    rf_side_right_bck: int
    rf_bck_center: int

    # Motor outputs (PWM -255..255)
    pwm_left: float
    pwm_right: float

    # Bumpers bitmask: bit0=left, bit1=center, bit2=right
    bumpers: int

    # Status flags bitmask (STAT_NO_CMD, STAT_POWER, STAT_RF_LEFT, STAT_RF_RIGHT)
    status: int

    # Number of commands received by the device since boot
    cmd_count: int


@dataclass
class MicrobotTelemetry:
    """
    Telemetry for the three-wheeled micro-robot (firmware/esp32-microbot).

    The ESP32 sends one newline-terminated line per snapshot:
        t=<ms>,rf_l=<cm>,rf_c=<cm>,rf_r=<cm>,vbat=<V>,v=<m/s>,w=<rad/s>,
        pwm_l=<int>,pwm_r=<int>,armed=<0|1>,kp=<f>,ki=<f>,kd=<f>
    """
    uptime_ms: int          # milliseconds since the ESP32 booted

    # Sharp GP2Y0A21 rangefinders, centimetres. -1 means "outside 10..80 cm",
    # which the sensor genuinely cannot resolve: below ~10 cm its curve folds
    # back and a very close obstacle reads like a distant one. Treat -1 as
    # "unknown", never as "clear".
    rf_left: int
    rf_center: int
    rf_right: int

    vbat: float             # battery voltage through the on-board divider, V

    # Setpoint the firmware is currently acting on (not a measurement — there
    # are no encoders yet, see the PID section of the firmware).
    speed_lin: float        # m/s
    speed_ang: float        # rad/s

    pwm_left: int           # actually applied duty, -255..255
    pwm_right: int
    armed: bool             # power stage enabled (TB6612 STBY / local kill switch)

    # PID gains currently loaded in the firmware, echoed back so a client can
    # confirm a set_pid actually landed.
    kp: float
    ki: float
    kd: float


@dataclass
class SimpleSerialTelemetry:
    """
    Telemetry for a minimal serial device.

    Expected line format from the device (newline-terminated):
        uptime=<int>,value=<float>,status=<str>
    Example:
        uptime=123,value=45.6,status=OK
    """
    uptime: int     # seconds since boot
    value: float    # primary sensor reading (device-specific meaning)
    status: str     # free-form status string from the device
