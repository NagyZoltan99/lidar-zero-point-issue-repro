# ovrtx OmniLidar — silent 0-point reproduction

Minimal, self-contained reproduction for: **`OmniLidar` (Generic Core) silently
returns 0 points for a wide / near-pole `emitterState:s001:elevationDeg` fan,
with no error or warning** (even at `log_level="verbose"`).

It depends only on `numpy`, `ovrtx`, and `ovstage` — no application code.

## Files

- `scene.usda` — a tiny scene: a ground plane and one cube, each with a visible
  `UsdPreviewSurface` shader and a non-visual (sensor) `concrete` material.
- `repro.py` — authors a rotary `OmniLidar` inline (single emitter state whose
  `elevationDeg` is the vertical fan), references it into the scene, steps the
  sensor's `RenderProduct`, and prints the valid return count (`Counts[0]`).
- `pyproject.toml` — declares the three dependencies and the NVIDIA package index.

## Run

With [uv](https://docs.astral.sh/uv/):

```bash
uv run repro.py
```

Or with any environment that already has `ovrtx` + `ovstage` + `numpy`:

```bash
python repro.py
```

The default run prints a config matrix. Single config and verbose renderer log:

```bash
# one config, write the renderer log so you can confirm nothing is logged for 0 points
python repro.py --elev-min -90 --elev-max 90 --rotate 90 0 -90 --log ovrtx.log --verbose
```

CLI options: `--elev-min`, `--elev-max`, `--emitters`, `--rotate RX RY RZ`,
`--azimuth-end`, `--log`, `--verbose`.

## Expected output

```
Running config matrix (azimuth 0..90):
       rotateXYZ |      elev band | emitters |  valid pts
---------------------------------------------------------
 (0.0, 0.0, 0.0) |    -45.0..45.0 |      167 |     ~148800
 (0.0, 0.0, 0.0) |    -90.0..90.0 |      334 |          0
(90.0, 0.0, -90.0) |    -60.0..60.0 |      223 |      ~99700
(90.0, 0.0, -90.0) |    -80.0..80.0 |      297 |          0
(90.0, 0.0, -90.0) |    -90.0..90.0 |      334 |          0
```

Moderate elevation bands return a dense cloud; wide / near-pole bands return
**0** points. The sensor initializes, schedules, and runs identically in both
cases — nothing in the (verbose) renderer log explains the empty result.
(Point counts vary by a few per run; the 0 vs non-0 split is stable.)

## The bug

The authored `elevationDeg` array is correct and evenly spaced in every case
(the script builds it with `numpy.linspace`). The only thing that changes the
outcome from "dense cloud" to "0 points, no diagnostic" is widening the fan
toward the vertical poles, or combining a moderate fan with a steep prim mount.
There is no documented elevation-range control on `OmniSensorGenericLidarCoreAPI`
and no runtime message when every ray is dropped.

## Environment where this was observed

- RTX 4060 Laptop GPU, driver 596.08, Windows
- Python 3.12.14
- `ovrtx==0.5.0.377615`, `ovstage==0.2.0.377349`
- Also observed on `ovrtx==0.4.1.364340` / `ovstage==0.1.1.355824`
- Standalone attached-`ovstage` renderer, `motion_bvh=ENABLE`; the runtime
  selects the `LidarWPM` plugin (`rtx.sensors.lidar.wpm.plugin`).

> On Windows `repro.py` adds `ovrtx/bin/plugins/gpu.foundation` to the DLL search
> path automatically before importing `ovrtx`.
