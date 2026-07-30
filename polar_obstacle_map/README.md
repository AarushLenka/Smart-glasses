# Polar Obstacle Map — MPU6050 + VL53L0X + ESP32

VL53L0X is a **single-point** ToF sensor, not a scanner. The map comes from *sweeping*:
MPU6050 gives bearing (integrated gyro yaw), VL53L0X gives range, the host bins
`(yaw, range)` pairs into angular sectors.

```
firmware/polar_scanner/polar_scanner.ino   ESP32: reads both sensors, streams CSV @115200
polar_map.py                               host: bins into sectors + live polar plot
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

VL53L0X `XSHUT` / `GPIO1` unconnected. Point the ToF the same direction the yaw
axis measures — i.e. the sensor rotates with the board about the Z axis.

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

Plot: 0° = straight ahead, clockwise = right turn (matches +yaw). Colour = range,
fade = sector age. Green line is current heading. Title reports the nearest
obstacle in the forward ±30° cone.

Flags: `--sector` (bin width, default 5°), `--ttl` (sector lifetime, default 4 s —
lower it if you move through the room, raise it for a static sweep).

## Calibration

- `YAW_SCALE` in the firmware: spin exactly 360°, read the yaw; if it says 352, set `360.0/352.0`.
- `ZUPT_DPS`: yaw only integrates above this rate, so drift freezes while still. Raise if
  the heading creeps at rest, lower if slow sweeps get dropped.
- `MAX_MM` in `polar_map.py` is 2000 (VL53L0X long-range spec); readings past it are noise.

## Known limits

- No magnetometer, so yaw is dead-reckoned. It drifts over minutes even with ZUPT — press `z` to re-zero.
- One ToF beam (~25° cone) means the map is only as dense as your sweep. Sweep slowly.
