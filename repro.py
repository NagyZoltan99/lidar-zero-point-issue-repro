"""Minimal reproduction: OmniLidar (Generic Core) silently returns 0 points for
a wide / near-pole emitterState elevationDeg fan, with no diagnostic.

Authors a rotary OmniLidar whose vertical fan is a single emitter state
(OmniSensorGenericLidarCoreEmitterStateAPI:s001) spanning [elev_min, elev_max],
references it into a tiny scene, steps the sensor's RenderProduct, and prints
the valid return count (PointCloud Counts[0]).

By default it runs a small matrix of (mount, elevation-band) configs and prints
a table: moderate bands return points, wide / near-pole bands return 0 -- with
no error or warning in the renderer log either way.

Usage
-----
Run the default matrix:
    python repro.py

Run a single config:
    python repro.py --elev-min -90 --elev-max 90 --rotate 90 0 -90

Write the renderer log (verbose) to inspect that nothing is logged for 0 points:
    python repro.py --elev-min -90 --elev-max 90 --rotate 90 0 -90 --log ovrtx.log --verbose

Requires: ovrtx + ovstage installed in the active environment, and an
RTX GPU. On Windows the ovrtx GPU-foundation DLL dir is added automatically.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path

import numpy as np

SCENE = str(Path(__file__).resolve().parent / "scene.usda")
ROOT = "/World"
PRODUCT_PATH = f"{ROOT}/Render/Products/ScannerProduct"
POINTCLOUD_VAR = f"{ROOT}/Render/Vars/PointCloud"


def _bootstrap_windows_cuda_dll() -> None:
    """ovrtx needs its gpu.foundation plugin dir on the Windows DLL path."""
    if os.name != "nt":
        return
    spec = importlib.util.find_spec("ovrtx")
    if spec is None or spec.origin is None:
        return
    dll_dir = os.path.join(os.path.dirname(spec.origin), "bin", "plugins", "gpu.foundation")
    if os.path.isdir(dll_dir):
        os.add_dll_directory(dll_dir)


def sensor_usda(elev_min: float, elev_max: float, num_emitters: int,
                rotate_xyz: tuple[float, float, float],
                azimuth_end: float, fire_step_ns: int = 100) -> str:
    """A rotary OmniLidar whose vertical fan is a single emitter column of
    `num_emitters` samples evenly spread over [elev_min, elev_max]."""
    elevs = np.linspace(elev_min, elev_max, num_emitters)
    azimuth = ", ".join(["0.0"] * num_emitters)
    elevation = ", ".join(f"{e:.5f}" for e in elevs)
    fire = ", ".join(str(fire_step_ns * i) for i in range(num_emitters))
    channel = ", ".join(str(i + 1) for i in range(num_emitters))
    rx, ry, rz = rotate_xyz
    return f'''#usda 1.0
(
    defaultPrim = "World"
)

def Xform "World"
{{
    def Xform "TLS_Rig"
    {{
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def OmniLidar "TLS" (
            prepend apiSchemas = ["OmniSensorGenericLidarCoreAPI", "OmniSensorGenericLidarCoreEmitterStateAPI:s001"]
        )
        {{
            token omni:sensor:Core:elementsCoordsType = "CARTESIAN"
            token omni:sensor:Core:outputFrameOfReference = "WORLD"
            token omni:sensor:Core:scanType = "ROTARY"
            bool omni:sensor:Core:accumulateOutputs = true
            bool omni:sensor:Core:instantLidar = true
            bool omni:sensor:Core:includeInvalidPoints = false
            uint omni:sensor:Core:maxReturns = 2
            float omni:sensor:Core:nearRangeM = 0.3
            float omni:sensor:Core:farRangeM = 200.0
            float omni:sensor:Core:validStartAzimuthDeg = 0.0
            float omni:sensor:Core:validEndAzimuthDeg = {azimuth_end}
            uint omni:sensor:Core:numberOfEmitters = {num_emitters}
            uint omni:sensor:Core:numberOfChannels = {num_emitters}
            float[] omni:sensor:Core:emitterState:s001:azimuthDeg = [{azimuth}]
            float[] omni:sensor:Core:emitterState:s001:elevationDeg = [{elevation}]
            uint[] omni:sensor:Core:emitterState:s001:fireTimeNs = [{fire}]
            uint[] omni:sensor:Core:emitterState:s001:channelId = [{channel}]
            double2 omni:sensor:frameRate = (10, 1)
            double3 xformOp:translate = (0, 0, 1)
            float3 xformOp:rotateXYZ = ({rx}, {ry}, {rz})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
        }}
    }}

    def "Render"
    {{
        def "Products"
        {{
            def RenderProduct "ScannerProduct"
            {{
                rel camera = <{ROOT}/TLS_Rig/TLS>
                rel orderedVars = [<../../Vars/PointCloud>]
            }}
        }}
        def "Vars"
        {{
            def RenderVar "PointCloud"
            {{
                uniform string sourceName = "PointCloud"
                token[] channels = ["Coordinates", "Intensity", "Counts", "TimeOffsetNs"]
            }}
        }}
    }}
}}
'''


def run_once(elev_min: float, elev_max: float, num_emitters: int,
             rotate_xyz: tuple[float, float, float], azimuth_end: float,
             log_path: str | None, verbose: bool) -> int:
    """Author + step the sensor once, return the valid point count."""
    import ovrtx
    import ovstage
    from ovstage import population

    cfg_kwargs = dict(motion_bvh=ovrtx.MotionBvh.ENABLE)
    if log_path:
        cfg_kwargs["log_file_path"] = log_path
    cfg_kwargs["log_level"] = "verbose" if verbose else "info"

    renderer = ovrtx.Renderer(ovrtx.RendererConfig(**cfg_kwargs))
    stage = ovstage.Stage("ovrtx_lidar_repro")
    renderer.attach_ovstage(stage)
    try:
        ordinal = 1
        population.open_usd(stage, SCENE, ordinal=ordinal)
        stage.advance_write_floor(ordinal, ovstage.Scope.ALL).wait()

        ordinal = 2
        usda = sensor_usda(elev_min, elev_max, num_emitters, rotate_xyz, azimuth_end)
        population.add_usd_reference_from_string(stage, usda, ROOT)
        population.apply_usd_changes(stage, ordinal=ordinal)
        stage.advance_write_floor(ordinal, ovstage.Scope.ALL).wait()

        # Warm up (first step compiles shaders) then read one scan.
        for _ in range(3):
            renderer.step(render_products={PRODUCT_PATH}, delta_time=0.1, ordinal=ordinal)
        products = renderer.step(render_products={PRODUCT_PATH}, delta_time=0.1, ordinal=ordinal)

        frame = products[PRODUCT_PATH].frames[0]
        rvs = frame.render_vars
        rv = rvs.get(POINTCLOUD_VAR) or rvs.get("PointCloud")
        if rv is None:
            raise RuntimeError(f"PointCloud render var not found; keys={list(rvs.keys())}")
        with rv.map(device=ovrtx.Device.CPU) as pc:
            count = int(np.asarray(np.from_dlpack(pc["Counts"]))[0])
        del products
        # Note: ovrtx may log a harmless "Renderer destroyed with 1 active
        # mapping" RuntimeWarning at teardown (DLPack view bookkeeping). It does
        # not affect the returned count.
        return count
    finally:
        try:
            renderer.detach_ovstage()
        except Exception:
            pass
        stage.destroy()
        renderer.destroy()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elev-min", type=float, help="emitter elevation band min (deg)")
    ap.add_argument("--elev-max", type=float, help="emitter elevation band max (deg)")
    ap.add_argument("--emitters", type=int, default=167,
                    help="number of emitters (vertical samples); default 167")
    ap.add_argument("--rotate", type=float, nargs=3, metavar=("RX", "RY", "RZ"),
                    default=[90.0, 0.0, -90.0],
                    help="OmniLidar prim xformOp:rotateXYZ (deg); default 90 0 -90")
    ap.add_argument("--azimuth-end", type=float, default=90.0,
                    help="validEndAzimuthDeg; default 90")
    ap.add_argument("--log", type=str, default=None, help="write renderer log to this file")
    ap.add_argument("--verbose", action="store_true", help="use verbose renderer log level")
    args = ap.parse_args()

    _bootstrap_windows_cuda_dll()

    # Single config if elevation band given explicitly.
    if args.elev_min is not None and args.elev_max is not None:
        rot = tuple(args.rotate)
        count = run_once(args.elev_min, args.elev_max, args.emitters, rot,
                         args.azimuth_end, args.log, args.verbose)
        print(f"rotateXYZ={rot} elev=[{args.elev_min},{args.elev_max}] "
              f"emitters={args.emitters} -> valid points = {count}")
        return 0

    # Default: a matrix showing the works -> 0 transition.
    print("Running config matrix (azimuth 0..%g):\n" % args.azimuth_end)
    header = f"{'rotateXYZ':>16} | {'elev band':>14} | {'emitters':>8} | {'valid pts':>10}"
    print(header)
    print("-" * len(header))
    matrix = [
        ((0.0, 0.0, 0.0),    -45.0,  45.0, 167),
        ((0.0, 0.0, 0.0),    -90.0,  90.0, 334),
        ((90.0, 0.0, -90.0), -60.0,  60.0, 223),
        ((90.0, 0.0, -90.0), -80.0,  80.0, 297),
        ((90.0, 0.0, -90.0), -90.0,  90.0, 334),
    ]
    for rot, lo, hi, n in matrix:
        count = run_once(lo, hi, n, rot, args.azimuth_end, args.log, args.verbose)
        print(f"{str(rot):>16} | {f'{lo}..{hi}':>14} | {n:>8} | {count:>10}")
    print("\nModerate bands return points; wide / near-pole bands return 0 "
          "with no error or warning in the renderer log.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
