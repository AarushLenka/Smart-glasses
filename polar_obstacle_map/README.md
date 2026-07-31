# Polar Obstacle Map — MPU6050 + VL53L0X + ESP32

VL53L0X is a **single-point** ToF sensor, not a scanner. The map comes from *sweeping*:
MPU6050 gives bearing (yaw = azimuth, pitch = elevation), VL53L0X gives range, the host
drops each `(yaw, pitch, range)` triple into a spherical cell. Sweep left/right **and**
up/down to fill it.

```
firmware/polar_scanner/polar_scanner.ino   ESP32: reads both sensors, streams CSV @115200
firmware/i2c_scan/i2c_scan.ino             diagnostic: list what's actually on the bus
polar_map.py                               host: spherical cells + live 3D plot
sample_scan.csv                            synthetic scan (no hardware needed)
sample_map.png                             what the output looks like
```

## Wiring

Both sensors share one I2C bus — no address conflict (MPU6050 `0x68`, VL53L0X `0x29`).

| ESP32 | MPU6050 | VL53L0X |
|-------|---------|---------|
| 3V3   | VCC     | VIN     |
| GND   | GND     | GND     |
| GPIO21| SDA     | SDA     |
| GPIO22| SCL     | SCL     |

VL53L0X `XSHUT` / `GPIO1` unconnected.

**Mounting:** the MPU6050 sits perpendicular to the ToF bore — rotated +90° about its own
Y axis — so the board's yaw axis is the MPU's −X, not its Z. `mountRotate()` in the
firmware rotates gyro and accel into the ToF's frame once, and every formula downstream
stays there. Mounted differently? Change `mountRotate()`, not the filter maths.

## Firmware

Arduino Library Manager: **Adafruit MPU6050**, **Adafruit VL53L0X**, **Adafruit Unified
Sensor**, **Adafruit BusIO**. Flash `firmware/polar_scanner/polar_scanner.ino`.

On boot it calibrates the gyro — **hold the board still for ~2 s**. Serial commands:
`z` = zero heading (point straight ahead first), `c` = recalibrate.

Output: `t_ms,yaw_deg,pitch_deg,roll_deg,range_mm,ok` (lines starting `#` are comments).

## Host

```
pip install -r requirements.txt
python polar_map.py --port /dev/ttyUSB0            # live plot
python polar_map.py --port /dev/ttyUSB0 --log s.csv # live + record
python polar_map.py --replay sample_scan.csv        # replay a recording
python polar_map.py --replay s.csv --headless       # one static plot at the end
python polar_map.py --selftest                      # logic check, no hardware
```

Plot: 3D point cloud, +X ahead, +Y left, +Z up (right-handed, matches +yaw). Axes are a
±2 m cube on 500 mm ticks — exactly the sensor's reachable volume. Colour temperature
carries proximity: **red = close, blue = far**, normalized over the real 30–2000 mm
window. Alpha fades with cell age. Green line is current heading, green triangle at the
origin is the sensor. Title reports the nearest obstacle in the forward ±30° cone.

Flags: `--sector` (cell width, default 5°), `--ttl` (cell lifetime, default 4 s —
lower it if you move through the room, raise it for a static sweep).

## Calibration

- `YAW_SCALE` in the firmware: spin exactly 360°, read the yaw; if it says 352, set `360.0/352.0`.
- `ZUPT_DPS`: yaw only integrates above this rate, so drift freezes while still. Raise if
  the heading creeps at rest, lower if slow sweeps get dropped.
- `MAX_MM` in `polar_map.py` is 2000 (VL53L0X long-range spec); readings past it are noise.

## Known limits

- No magnetometer, so yaw is dead-reckoned. It drifts over minutes even with ZUPT — press `z` to re-zero.
- One ToF beam (~25° cone) means the map is only as dense as your sweep. Sweep slowly, in both axes.
- Elevation clamps at ±90° instead of wrapping — straight up is not straight down.

## Troubleshooting

No serial output? Wrong port is the usual cause — a CH343 bridge shows up as
`/dev/ttyACM0`, not `/dev/ttyUSB0`. One process per port: close any IDE serial monitor
first or you get `OSError: [Errno 16] Device or resource busy`.

`# ERROR: MPU6050 not found at 0x68`? Flash `firmware/i2c_scan/i2c_scan.ino` and read the
scan. If it lists `0x29` only, the bus, pins, pull-ups and power rail are all fine and the
MPU6050 simply isn't answering — and it's not an AD0 problem either, since that would show
up at `0x69`. Ranked causes: GY-521 LDO dropping out on 3V3 (move only the MPU's VCC to
5V/VIN), SDA/SCL off by one breadboard row, GND not shared, dead clone module.
