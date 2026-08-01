#!/usr/bin/env python3
"""Build and visualize a 3D obstacle map from the ESP32 scanner CSV stream.

The VL53L0X is a single-point ToF: one range per sample. Sweeping the head
turns that stream into a map -- yaw gives azimuth, pitch gives elevation,
and each (yaw, pitch, range) triple drops into a spherical cell that keeps
the nearest recent hit. Sweep left/right AND up/down to fill it.

    python polar_map.py --port /dev/ttyUSB0            # live from ESP32
    python polar_map.py --port /dev/ttyUSB0 --log s.csv # live + record
    python polar_map.py --replay s.csv                 # replay a recording
    python polar_map.py --selftest                     # no hardware needed
"""

import argparse
import csv
import math
import sys
import time

# VL53L0X long-range spec; beyond this the reading is noise.
MIN_MM, MAX_MM = 30, 2000


class PolarMap:
    """Spherical cells: azimuth over [-180, 180), elevation over [-90, 90].

    Each cell holds the nearest recent range. Cells are indexed (az, el) and
    stored in a dict -- a sweep only ever touches a thin band of the sphere,
    so a dense 72x36 array would be mostly None.
    """

    def __init__(self, sector_deg=5.0, ttl_s=4.0, max_mm=MAX_MM, elev_deg=None):
        self.n = int(round(360.0 / sector_deg))
        self.sector_deg = 360.0 / self.n
        self.elev_deg = elev_deg or sector_deg
        self.n_el = int(round(180.0 / self.elev_deg))
        self.ttl_s = ttl_s
        self.max_mm = max_mm
        self.cells = {}          # (i_az, i_el) -> (range_mm, stamp)

    def bin_of(self, yaw_deg):
        return int((yaw_deg + 180.0) // self.sector_deg) % self.n

    def elev_bin_of(self, pitch_deg):
        i = int((pitch_deg + 90.0) // self.elev_deg)
        return min(max(i, 0), self.n_el - 1)   # clamp: elevation doesn't wrap

    def angle_of(self, i):
        """Azimuth cell centre, degrees, same convention as yaw (0 = ahead)."""
        return -180.0 + (i + 0.5) * self.sector_deg

    def elev_of(self, j):
        """Elevation cell centre, degrees (0 = level)."""
        return -90.0 + (j + 0.5) * self.elev_deg

    def update(self, yaw_deg, range_mm, now, pitch_deg=0.0):
        """Insert a reading. Nearest-wins within TTL, else the new value owns it."""
        if not (MIN_MM <= range_mm <= self.max_mm):
            return False
        key = (self.bin_of(yaw_deg), self.elev_bin_of(pitch_deg))
        prev = self.cells.get(key)
        if prev is not None and (now - prev[1]) <= self.ttl_s and prev[0] < range_mm:
            self.cells[key] = (prev[0], now)  # keep closer obstacle, refresh age
            return False
        self.cells[key] = (range_mm, now)
        return True

    def snapshot(self, now):
        """[(az_deg, el_deg, range_mm, age_s)] for unexpired cells."""
        out = []
        for (i, j), (r, stamp) in self.cells.items():
            age = now - stamp
            if age <= self.ttl_s:
                out.append((self.angle_of(i), self.elev_of(j), r, age))
        return out

    def nearest(self, now, fov_deg=60.0, heading=0.0):
        """Closest obstacle in the cone around `heading`, or None.

        Returns (range_mm, az_deg) with az relative to `heading`, so it lines up
        with what the plot draws.
        """
        ahead = [(r, wrap180(a - heading)) for a, e, r, _ in self.snapshot(now)
                 if abs(wrap180(a - heading)) <= fov_deg / 2 and abs(e) <= fov_deg / 2]
        return min(ahead) if ahead else None


def wrap180(a):
    """Fold an angle into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def to_xyz(az_deg, el_deg, r_mm):
    """Spherical to Cartesian. +X ahead, +Y left, +Z up, matching the yaw sign."""
    az, el = math.radians(az_deg), math.radians(el_deg)
    horiz = r_mm * math.cos(el)
    return horiz * math.cos(az), -horiz * math.sin(az), r_mm * math.sin(el)


def parse_line(line):
    """CSV row -> (t_ms, yaw, pitch, roll, range_mm, ok), or None if not a sample."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) != 6:
        return None
    try:
        t_ms = int(parts[0])
        yaw, pitch, roll = (float(p) for p in parts[1:4])
        return t_ms, yaw, pitch, roll, int(parts[4]), int(parts[5])
    except ValueError:
        return None


def serial_source(port, baud, log_path=None):
    import serial  # pyserial

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # ESP32 resets on port open
    ser.reset_input_buffer()
    log = open(log_path, "w") if log_path else None
    try:
        while True:
            raw = ser.readline().decode("utf-8", "replace")
            if log and raw:
                log.write(raw)
            sample = parse_line(raw)
            if sample:
                yield sample
    finally:
        ser.close()
        if log:
            log.close()


def replay_source(path, speed=4.0):
    """Replay a recorded CSV, roughly honouring the original timing."""
    t0 = None
    wall0 = time.time()
    with open(path) as f:
        for raw in f:
            sample = parse_line(raw)
            if not sample:
                continue
            if t0 is None:
                t0 = sample[0]
            target = (sample[0] - t0) / 1000.0 / speed - (time.time() - wall0)
            if target > 0:
                time.sleep(min(target, 0.5))
            yield sample


def run(source, sector_deg, ttl_s, headless=False):
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    pmap = PolarMap(sector_deg=sector_deg, ttl_s=ttl_s)

    if headless:
        heading = 0.0
        for yaw, pitch, rng, ok in ((s[1], s[2], s[4], s[5]) for s in source):
            heading = yaw
            if ok:
                pmap.update(yaw, rng, time.time(), pitch)
        # No "now" in a finished run, so FRONT is where the scanner ended up.
        plot_static(pmap, plt, Normalize, heading)
        return

    plt.ion()
    fig = plt.figure(figsize=(10, 8.5))
    ax = fig.add_subplot(projection="3d")
    style_axes(ax, pmap.max_mm)
    # Colour temperature carries proximity: hot = close, cold = far. Normalized
    # over the sensor's real 30..2000 mm window so the ramp isn't wasted on
    # ranges the VL53L0X can never report.
    scat = ax.scatter([], [], [], s=45, c=[], cmap="turbo_r",
                      norm=Normalize(MIN_MM, pmap.max_mm), depthshade=False)
    beam, = ax.plot([0, 0], [0, 0], [0, 0], color="#0a7f2e", lw=2.0)
    title = ax.set_title("scanning...", fontsize=11, pad=14)
    fig.colorbar(scat, ax=ax, pad=0.02, shrink=0.55, aspect=28,
                 label="range (mm)  -  red = close, blue = far")
    fig.subplots_adjust(left=0.02, right=0.90, bottom=0.04, top=0.93)

    last_draw = 0.0
    for _t, yaw, pitch, _roll, rng, ok in source:
        now = time.time()
        if ok:
            pmap.update(yaw, rng, now, pitch)
        if now - last_draw < 0.1:      # ~10 fps; the stream is 100 Hz
            continue
        last_draw = now

        # Cells are stored in the world frame (yaw as integrated since boot),
        # but the plot is drawn head-relative: subtract the live yaw so FRONT is
        # always wherever the scanner is pointing right now and the cloud swings
        # around it. Elevation is absolute (gravity-referenced), so UP/DOWN and
        # the pitch of every point stay untouched.
        pts = pmap.snapshot(now)
        if pts:
            xyz = [to_xyz(a - yaw, e, r) for a, e, r, _ in pts]
            rr = [r for _, _, r, _ in pts]
            # _offsets3d is the only way to update a 3D scatter in place;
            # set_offsets is 2D-only. Private, but stable across matplotlib 3.x.
            scat._offsets3d = tuple(zip(*xyz))
            scat.set_array(rr)
            scat.set_alpha([max(0.15, 1.0 - a / ttl_s) for _, _, _, a in pts])
        bx, by, bz = to_xyz(0.0, pitch, pmap.max_mm)   # head-relative: beam is FRONT
        beam.set_data([0, bx], [0, by])
        beam.set_3d_properties([0, bz])

        near = pmap.nearest(now, heading=yaw)
        title.set_text(f"yaw {yaw:6.1f}  pitch {pitch:5.1f}  cells {len(pts):4d}   "
                       + (f"nearest ahead {near[0]:4d} mm @ {near[1]:.0f}deg"
                          if near else "path clear"))
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        if not plt.fignum_exists(fig.number):
            break


# Which way is which, in head-relative coordinates: +X is wherever the scanner
# currently points, +Y left of it, +Z up. FRONT is a live direction, not a fixed
# compass bearing -- UP/DOWN stay absolute because elevation comes from gravity.
DIRECTIONS = [
    ((1, 0, 0), "FRONT", "#c1121f"),
    ((-1, 0, 0), "BACK", "#7d8597"),
    ((0, 1, 0), "LEFT", "#0353a4"),
    ((0, -1, 0), "RIGHT", "#0353a4"),
    ((0, 0, 1), "UP", "#2a9d8f"),
    ((0, 0, -1), "DOWN", "#7d8597"),
]


def style_axes(ax, max_mm):
    # Axis numbers are the cramped part: six labelled arrows say which way is
    # which far better than three tick-label runs, so keep ticks sparse.
    ticks = list(range(-max_mm, max_mm + 1, 1000))
    for setlim, setticks in ((ax.set_xlim, ax.set_xticks),
                             (ax.set_ylim, ax.set_yticks),
                             (ax.set_zlim, ax.set_zticks)):
        setlim(-max_mm * 1.3, max_mm * 1.3)   # headroom so arrow labels fit
        setticks(ticks)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((1, 1, 1, 0))   # opaque panes hid the axis lines
        axis.set_tick_params(labelsize=7, colors="#666666")
    ax.set_box_aspect((1, 1, 1))      # equal scale: geometry stays undistorted
    ax.view_init(elev=22, azim=-60)
    ax.grid(True, alpha=0.2)

    # Arrows run past the cube face so a label never lands on the point cloud.
    tip, label = max_mm * 1.0, max_mm * 1.22
    for (dx, dy, dz), name, colour in DIRECTIONS:
        ax.quiver(0, 0, 0, dx * tip, dy * tip, dz * tip, color=colour,
                  arrow_length_ratio=0.08, lw=1.4, alpha=0.55)
        ax.text(dx * label, dy * label, dz * label, name, color=colour,
                fontsize=9, fontweight="bold", ha="center", va="center")
    ax.scatter([0], [0], [0], c="#0a7f2e", s=70, marker="^")   # sensor origin


def plot_static(pmap, plt, Normalize, heading=0.0):
    # Whole-run summary: ignore TTL, every cell ever filled is interesting.
    # Same head-relative rotation as the live view -- FRONT is `heading`.
    pts = [(pmap.angle_of(i) - heading, pmap.elev_of(j), r)
           for (i, j), (r, _) in pmap.cells.items()]
    xyz = [to_xyz(a, e, r) for a, e, r in pts]
    fig = plt.figure(figsize=(10, 8.5))
    ax = fig.add_subplot(projection="3d")
    style_axes(ax, pmap.max_mm)
    sc = ax.scatter(*(zip(*xyz) if xyz else ([], [], [])),
                    c=[r for _, _, r in pts], cmap="turbo_r", s=45,
                    norm=Normalize(MIN_MM, pmap.max_mm), depthshade=False)
    fig.colorbar(sc, ax=ax, pad=0.02, shrink=0.55, aspect=28,
                 label="range (mm)  -  red = close, blue = far")
    ax.set_title(f"3D obstacle map - {len(pts)} cells", fontsize=11, pad=14)
    fig.subplots_adjust(left=0.02, right=0.90, bottom=0.04, top=0.93)
    plt.show()


def selftest():
    m = PolarMap(sector_deg=10.0, ttl_s=2.0)
    assert m.n == 36 and m.n_el == 18
    assert m.bin_of(0.0) == 18 and m.bin_of(-180.0) == 0 and m.bin_of(179.9) == 35
    assert m.bin_of(185.0) == m.bin_of(-175.0)                 # azimuth wraps
    assert abs(m.angle_of(m.bin_of(43.0)) - 45.0) < 5.0        # centre near input

    # Elevation clamps instead of wrapping: straight up is not straight down.
    assert m.elev_bin_of(0.0) == 9 and m.elev_bin_of(-90.0) == 0
    assert m.elev_bin_of(90.0) == 17 and m.elev_bin_of(200.0) == 17
    assert abs(m.elev_of(m.elev_bin_of(-32.0)) + 35.0) < 5.0

    t = 1000.0
    assert m.update(0.0, 500, t)
    assert not m.update(2.0, 900, t)          # same cell, farther -> nearest wins
    assert m.update(2.0, 200, t)              # same cell, closer -> replaces
    assert not m.update(0.0, 10, t)           # below MIN_MM rejected
    assert not m.update(0.0, 9999, t)         # beyond max rejected
    assert m.update(0.0, 900, t + 3.0)        # stale cell -> new value owns it

    # Same azimuth, different elevation = a different cell, not a collision.
    assert m.update(0.0, 700, t + 3.0, pitch_deg=40.0)
    assert len(m.snapshot(t + 3.0)) == 2
    assert m.snapshot(t + 9.0) == []          # everything expired

    assert m.nearest(t + 3.0) == (900, 5.0)   # the 40 deg cell is outside the cone
    m.update(150.0, 100, t + 3.0)             # behind: outside the forward FOV
    assert m.nearest(t + 3.0)[0] == 900

    # Head-relative: turn to face 150 deg and the cell behind you is now ahead,
    # reported at ~0 deg relative. This is what the plot rotates by.
    r, rel = m.nearest(t + 3.0, heading=150.0)
    assert r == 100 and abs(rel) <= m.sector_deg   # within one cell of dead ahead
    assert wrap180(-190.0) == 170.0 and wrap180(190.0) == -170.0

    # Rotating the whole cloud by the heading must not touch elevation: a point
    # 90 deg to the world-right, viewed while facing it, lands on FRONT (+X).
    x, y, z = to_xyz(90.0 - 90.0, 30.0, 1000)
    ux, uy, uz = to_xyz(90.0, 30.0, 1000)
    assert abs(z - uz) < 1e-9                 # elevation survives the rotation
    assert x > 0 and abs(y) < 1e-9            # and it is dead ahead now

    # Straight ahead, level -> pure +X. Yaw right (+90) -> -Y. Pitch up -> +Z.
    x, y, z = to_xyz(0.0, 0.0, 1000)
    assert abs(x - 1000) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6
    x, y, z = to_xyz(90.0, 0.0, 1000)
    assert abs(x) < 1e-6 and abs(y + 1000) < 1e-6
    x, y, z = to_xyz(0.0, 90.0, 1000)
    assert abs(z - 1000) < 1e-6 and abs(x) < 1e-6

    # The arrows must point where to_xyz actually puts things, or the labels
    # lie -- which is exactly the bug the arrows were added to prevent.
    named = {n: v for v, n, _ in DIRECTIONS}
    for (az, el), name in (((0, 0), "FRONT"), ((180, 0), "BACK"),
                           ((-90, 0), "LEFT"), ((90, 0), "RIGHT"),
                           ((0, 90), "UP"), ((0, -90), "DOWN")):
        got = to_xyz(az, el, 1000)
        want = named[name]
        assert all(abs(g / 1000 - w) < 1e-6 for g, w in zip(got, want)), name

    assert parse_line("#header") is None
    assert parse_line("1,2,3") is None
    assert parse_line("12,-45.5,1.0,2.0,830,1") == (12, -45.5, 1.0, 2.0, 830, 1)
    print("selftest ok")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--replay", help="replay a recorded CSV instead of reading serial")
    p.add_argument("--log", help="write the raw serial stream to this CSV")
    p.add_argument("--sector", type=float, default=5.0, help="sector width, degrees")
    p.add_argument("--ttl", type=float, default=4.0, help="sector lifetime, seconds")
    p.add_argument("--headless", action="store_true",
                   help="consume the whole source, then plot once")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()

    if a.selftest:
        selftest()
        return
    if a.replay:
        src = replay_source(a.replay)
    elif a.port:
        src = serial_source(a.port, a.baud, a.log)
    else:
        p.error("need --port, --replay, or --selftest")

    try:
        run(src, a.sector, a.ttl, a.headless)
    except KeyboardInterrupt:
        print("\nstopped", file=sys.stderr)


if __name__ == "__main__":
    main()
