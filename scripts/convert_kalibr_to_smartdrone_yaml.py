#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert Kalibr calibration YAML to SmartDrone runtime YAML files.

Inputs:
- camchain yaml from `kalibr_calibrate_imu_camera`
- imu yaml used by Kalibr

Outputs:
- stereo yaml for SmartDrone native backends and the optional legacy ORB-SLAM3 adapter
- stereo_inertial yaml for SmartDrone native backends and the optional legacy ORB-SLAM3 adapter
- mono_inertial_right yaml for right-camera mono-imu mode

Example:
  python3 scripts/convert_kalibr_to_smartdrone_yaml.py \
    --camchain ~/workspace/kalibr_data/calib_runs/calib_data_1-camchain-imucam.yaml \
    --imu ~/workspace/kalibr_data/imu.yaml \
    --out-stereo-plain config/stereo.yaml \
    --out-stereo config/stereo_inertial.yaml \
    --out-mono-right config/mono_inertial_right.yaml
"""

import argparse
import math
import os
import posixpath
import subprocess
import sys

try:
    import yaml
except ImportError as exc:
    print("[error] PyYAML is required: pip install pyyaml", file=sys.stderr)
    raise


def split_remote_path(path: str):
    if ":" not in path:
        return None, path
    user_host, remote_path = path.split(":", 1)
    if not user_host or "/" in user_host or "\\" in user_host:
        return None, path
    return user_host, remote_path


def read_text(path: str) -> str:
    user_host, remote_path = split_remote_path(path)
    if user_host is None:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()

    cmd = ["ssh", user_host, "cat", remote_path]
    try:
        return subprocess.check_output(cmd, text=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Failed to read remote file: {path}") from exc


def read_yaml(path: str):
    return yaml.safe_load(read_text(path))


def join_any_path(base: str, leaf: str) -> str:
    user_host, remote_path = split_remote_path(base)
    if user_host is None:
        return os.path.join(base, leaf)
    return f"{user_host}:{posixpath.join(remote_path, leaf)}"


def list_remote_files(root: str):
    user_host, remote_path = split_remote_path(root)
    if user_host is None:
        return [os.path.join(root, name) for name in os.listdir(root)]

    cmd = ["ssh", user_host, "find", remote_path, "-maxdepth", "2", "-type", "f"]
    try:
        output = subprocess.check_output(cmd, text=True)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Failed to list remote files: {root}") from exc
    files = [line.strip() for line in output.splitlines() if line.strip()]
    return [f"{user_host}:{path}" for path in files]


def find_latest_matching_file(root: str, suffixes):
    files = list_remote_files(root)
    matched = []
    for path in files:
        local_path = split_remote_path(path)[1]
        if any(local_path.endswith(suffix) for suffix in suffixes):
            matched.append(path)
    if not matched:
        raise RuntimeError(f"No files matching {suffixes} found under {root}")
    matched.sort(key=lambda p: split_remote_path(p)[1])
    return matched[-1]


def mat4_from_rows(rows):
    if len(rows) != 4 or any(len(r) != 4 for r in rows):
        raise ValueError("Expected 4x4 matrix")
    return [[float(v) for v in row] for row in rows]


def mat3_transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def mat3_mul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def mat3_vec_mul(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def se3_inverse(T):
    R = [row[:3] for row in T[:3]]
    t = [T[i][3] for i in range(3)]
    Rt = mat3_transpose(R)
    tinv = [-x for x in mat3_vec_mul(Rt, t)]
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            out[i][j] = Rt[i][j]
        out[i][3] = tinv[i]
    out[3][3] = 1.0
    return out


def se3_mul(a, b):
    Ra = [row[:3] for row in a[:3]]
    ta = [a[i][3] for i in range(3)]
    Rb = [row[:3] for row in b[:3]]
    tb = [b[i][3] for i in range(3)]
    R = mat3_mul(Ra, Rb)
    t_rb = mat3_vec_mul(Ra, tb)
    t = [t_rb[i] + ta[i] for i in range(3)]
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            out[i][j] = R[i][j]
        out[i][3] = t[i]
    out[3][3] = 1.0
    return out


def flatten_mat4_row_major(T):
    values = []
    for row in T:
        values.extend(row)
    return values


def format_float(v: float) -> str:
    if math.isfinite(v):
        text = f"{v:.14f}".rstrip("0")
        if text.endswith("."):
            text += "0"
        return text
    raise ValueError(f"Non-finite float: {v}")


def format_opencv_mat4(name: str, T):
    values = ", ".join(format_float(v) for v in flatten_mat4_row_major(T))
    return (
        f"{name}:\n"
        f"  rows: 4\n"
        f"  cols: 4\n"
        f"  dt: f\n"
        f"  data: [{values}]\n"
    )


def extract_camera(cam_node, key_prefix: str):
    intrinsics = cam_node.get("intrinsics")
    if not intrinsics or len(intrinsics) < 4:
        raise ValueError(f"{key_prefix}: missing intrinsics")

    distortion = cam_node.get("distortion_coeffs", [])
    if len(distortion) < 4:
        raise ValueError(f"{key_prefix}: need at least 4 distortion coeffs")

    resolution = cam_node.get("resolution")
    if not resolution or len(resolution) != 2:
        raise ValueError(f"{key_prefix}: missing resolution")

    return {
        "fx": float(intrinsics[0]),
        "fy": float(intrinsics[1]),
        "cx": float(intrinsics[2]),
        "cy": float(intrinsics[3]),
        "k1": float(distortion[0]),
        "k2": float(distortion[1]),
        "p1": float(distortion[2]),
        "p2": float(distortion[3]),
        "width": int(resolution[0]),
        "height": int(resolution[1]),
    }


def build_stereo_inertial_yaml(cam0, cam1, T_c1_c2, T_b_c1, imu_cfg, fps, th_depth, far_points):
    bf = abs(T_c1_c2[0][3]) * cam0["fx"]
    lines = [
        "%YAML:1.0",
        "",
        'File.version: "1.0"',
        "",
        'Camera.type: "PinHole"',
        f"Camera.width: {cam0['width']}",
        f"Camera.height: {cam0['height']}",
        f"Camera.fps: {fps}",
        "Camera.RGB: 0",
        "",
        f"Camera1.fx: {format_float(cam0['fx'])}",
        f"Camera1.fy: {format_float(cam0['fy'])}",
        f"Camera1.cx: {format_float(cam0['cx'])}",
        f"Camera1.cy: {format_float(cam0['cy'])}",
        f"Camera1.k1: {format_float(cam0['k1'])}",
        f"Camera1.k2: {format_float(cam0['k2'])}",
        f"Camera1.p1: {format_float(cam0['p1'])}",
        f"Camera1.p2: {format_float(cam0['p2'])}",
        "",
        f"Camera2.fx: {format_float(cam1['fx'])}",
        f"Camera2.fy: {format_float(cam1['fy'])}",
        f"Camera2.cx: {format_float(cam1['cx'])}",
        f"Camera2.cy: {format_float(cam1['cy'])}",
        f"Camera2.k1: {format_float(cam1['k1'])}",
        f"Camera2.k2: {format_float(cam1['k2'])}",
        f"Camera2.p1: {format_float(cam1['p1'])}",
        f"Camera2.p2: {format_float(cam1['p2'])}",
        "",
        f"Camera.bf: {format_float(bf)}",
        f"Stereo.ThDepth: {format_float(th_depth)}",
        f"System.thFarPoints: {format_float(far_points)}",
        "",
        format_opencv_mat4("Stereo.T_c1_c2", T_c1_c2).rstrip(),
        "",
        format_opencv_mat4("IMU.T_b_c1", T_b_c1).rstrip(),
        "",
        f"IMU.Frequency: {format_float(float(imu_cfg['update_rate']))}",
        f"IMU.NoiseGyro: {format_float(float(imu_cfg['gyroscope_noise_density']))}",
        f"IMU.NoiseAcc: {format_float(float(imu_cfg['accelerometer_noise_density']))}",
        f"IMU.GyroWalk: {format_float(float(imu_cfg['gyroscope_random_walk']))}",
        f"IMU.AccWalk: {format_float(float(imu_cfg['accelerometer_random_walk']))}",
        "",
        "ORBextractor.nFeatures: 1000",
        "ORBextractor.scaleFactor: 1.2",
        "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 20",
        "ORBextractor.minThFAST: 7",
        "",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -1.8",
        "Viewer.ViewpointF: 500.0",
        "",
    ]
    return "\n".join(lines)


def build_stereo_plain_yaml(cam0, cam1, T_c1_c2, T_b_c1, fps, th_depth, far_points):
    bf = abs(T_c1_c2[0][3]) * cam0["fx"]
    lines = [
        "%YAML:1.0",
        "",
        'File.version: "1.0"',
        "",
        'Camera.type: "PinHole"',
        f"Camera.width: {cam0['width']}",
        f"Camera.height: {cam0['height']}",
        f"Camera.fps: {fps}",
        "Camera.RGB: 0",
        "",
        f"Camera1.fx: {format_float(cam0['fx'])}",
        f"Camera1.fy: {format_float(cam0['fy'])}",
        f"Camera1.cx: {format_float(cam0['cx'])}",
        f"Camera1.cy: {format_float(cam0['cy'])}",
        f"Camera1.k1: {format_float(cam0['k1'])}",
        f"Camera1.k2: {format_float(cam0['k2'])}",
        f"Camera1.p1: {format_float(cam0['p1'])}",
        f"Camera1.p2: {format_float(cam0['p2'])}",
        "",
        f"Camera2.fx: {format_float(cam1['fx'])}",
        f"Camera2.fy: {format_float(cam1['fy'])}",
        f"Camera2.cx: {format_float(cam1['cx'])}",
        f"Camera2.cy: {format_float(cam1['cy'])}",
        f"Camera2.k1: {format_float(cam1['k1'])}",
        f"Camera2.k2: {format_float(cam1['k2'])}",
        f"Camera2.p1: {format_float(cam1['p1'])}",
        f"Camera2.p2: {format_float(cam1['p2'])}",
        "",
        f"Camera.bf: {format_float(bf)}",
        f"Stereo.ThDepth: {format_float(th_depth)}",
        f"System.thFarPoints: {format_float(far_points)}",
        "",
        format_opencv_mat4("Stereo.T_c1_c2", T_c1_c2).rstrip(),
        "",
        format_opencv_mat4("T_b_c1", T_b_c1).rstrip(),
        "",
        "ORBextractor.nFeatures: 1100",
        "ORBextractor.scaleFactor: 1.2",
        "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 18",
        "ORBextractor.minThFAST: 5",
        "",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -1.8",
        "Viewer.ViewpointF: 500.0",
        "",
    ]
    return "\n".join(lines)


def build_mono_inertial_right_yaml(cam1, T_b_c2, imu_cfg, fps, far_points):
    lines = [
        "%YAML:1.0",
        "",
        'File.version: "1.0"',
        "",
        'Camera.type: "PinHole"',
        f"Camera.width: {cam1['width']}",
        f"Camera.height: {cam1['height']}",
        f"Camera.fps: {fps}",
        "Camera.RGB: 0",
        "",
        f"Camera1.fx: {format_float(cam1['fx'])}",
        f"Camera1.fy: {format_float(cam1['fy'])}",
        f"Camera1.cx: {format_float(cam1['cx'])}",
        f"Camera1.cy: {format_float(cam1['cy'])}",
        f"Camera1.k1: {format_float(cam1['k1'])}",
        f"Camera1.k2: {format_float(cam1['k2'])}",
        f"Camera1.p1: {format_float(cam1['p1'])}",
        f"Camera1.p2: {format_float(cam1['p2'])}",
        "",
        f"System.thFarPoints: {format_float(far_points)}",
        "",
        format_opencv_mat4("IMU.T_b_c1", T_b_c2).rstrip(),
        "",
        f"IMU.Frequency: {format_float(float(imu_cfg['update_rate']))}",
        f"IMU.NoiseGyro: {format_float(float(imu_cfg['gyroscope_noise_density']))}",
        f"IMU.NoiseAcc: {format_float(float(imu_cfg['accelerometer_noise_density']))}",
        f"IMU.GyroWalk: {format_float(float(imu_cfg['gyroscope_random_walk']))}",
        f"IMU.AccWalk: {format_float(float(imu_cfg['accelerometer_random_walk']))}",
        "",
        "ORBextractor.nFeatures: 1000",
        "ORBextractor.scaleFactor: 1.2",
        "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 20",
        "ORBextractor.minThFAST: 7",
        "",
        "Viewer.KeyFrameSize: 0.05",
        "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9",
        "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08",
        "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0",
        "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -1.8",
        "Viewer.ViewpointF: 500.0",
        "",
    ]
    return "\n".join(lines)


def write_text(path: str, text: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def main():
    parser = argparse.ArgumentParser(description="Convert Kalibr YAML to SmartDrone YAML")
    parser.add_argument("--kalibr-root", help="Local or remote Kalibr result root, e.g. ltz@host:~/workspace/kalibr_data/calib_runs")
    parser.add_argument("--camchain", help="Kalibr camchain-imucam yaml")
    parser.add_argument("--imu", help="Kalibr imu yaml")
    parser.add_argument("--out-stereo-plain", default="config/stereo.yaml", help="Output stereo yaml")
    parser.add_argument("--out-stereo", default="config/stereo_inertial.yaml", help="Output stereo inertial yaml")
    parser.add_argument(
        "--out-mono-right",
        default="config/mono_inertial_right.yaml",
        help="Output right mono inertial yaml",
    )
    parser.add_argument("--fps", type=int, default=60, help="Camera fps to write into output yaml")
    parser.add_argument("--stereo-th-depth", type=float, default=40.0, help="Stereo.ThDepth value")
    parser.add_argument("--far-points", type=float, default=8.0, help="System.thFarPoints value")
    args = parser.parse_args()

    camchain_path = args.camchain
    imu_path = args.imu
    if args.kalibr_root:
        if not camchain_path:
            camchain_path = find_latest_matching_file(
                args.kalibr_root,
                ("-camchain-imucam.yaml", "camchain-imucam.yaml", "-camchain.yaml", "camchain.yaml"),
            )
        if not imu_path:
            root_parent = split_remote_path(args.kalibr_root)[1]
            if root_parent.endswith("/calib_runs"):
                base_root = args.kalibr_root[: -len("calib_runs")] if args.kalibr_root.endswith("calib_runs") else args.kalibr_root
                candidate = join_any_path(base_root.rstrip("/\\"), "imu.yaml")
                try:
                    read_text(candidate)
                    imu_path = candidate
                except Exception:
                    pass

    if not camchain_path or not imu_path:
        raise RuntimeError("Need --camchain and --imu, or provide --kalibr-root plus discoverable files")

    camchain = read_yaml(camchain_path)
    imu_cfg = normalize_imu_config(read_yaml(imu_path))

    if "cam0" not in camchain or "cam1" not in camchain:
        raise RuntimeError("camchain yaml must contain cam0 and cam1")

    cam0 = extract_camera(camchain["cam0"], "cam0")
    cam1 = extract_camera(camchain["cam1"], "cam1")

    T_c1_c2 = mat4_from_rows(camchain["cam1"]["T_cn_cnm1"])
    T_c0_imu = mat4_from_rows(camchain["cam0"]["T_cam_imu"])
    T_b_c1 = se3_inverse(T_c0_imu)
    T_b_c2 = se3_mul(T_b_c1, T_c1_c2)

    stereo_plain_text = build_stereo_plain_yaml(
        cam0, cam1, T_c1_c2, T_b_c1, args.fps, args.stereo_th_depth, args.far_points
    )
    stereo_text = build_stereo_inertial_yaml(
        cam0, cam1, T_c1_c2, T_b_c1, imu_cfg, args.fps, args.stereo_th_depth, args.far_points
    )
    mono_right_text = build_mono_inertial_right_yaml(
        cam1, T_b_c2, imu_cfg, args.fps, args.far_points
    )

    write_text(args.out_stereo_plain, stereo_plain_text)
    write_text(args.out_stereo, stereo_text)
    write_text(args.out_mono_right, mono_right_text)

    print(f"[info] camchain: {camchain_path}")
    print(f"[info] imu: {imu_path}")
    print(f"[done] wrote stereo yaml: {args.out_stereo_plain}")
    print(f"[done] wrote stereo yaml: {args.out_stereo}")
    print(f"[done] wrote mono right yaml: {args.out_mono_right}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        sys.exit(1)
