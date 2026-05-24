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
