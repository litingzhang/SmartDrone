#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Make ROS1 bag for Kalibr from your recorder outputs.

Assumptions (matches your C++ recorder):
- cam0_dir / cam1_dir either contain `data.csv` with `timestamp_ns,filename`,
  or images named "<timestamp_ns>.png" (or .jpg/.jpeg/.bmp/.tiff).
- imu.csv columns (your C++ fprintf):
  #timestamp [ns],w_x [rad/s],w_y [rad/s],w_z [rad/s],a_x [m/s^2],a_y [m/s^2],a_z [m/s^2]
  i.e. gyro first, accel last.

Key fixes vs your buggy script:
1) IMU column order is corrected (gyro first, accel last).
2) Camera timestamps are preserved as recorded; no synthetic camera monotonic fixup.
3) Only enforce strict monotonicity for IMU if needed.
4) Use integer ns -> rospy.Time(secs,nsecs) to avoid float precision issues.
"""

import os, re, csv, sys
import cv2
import rosbag
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from cv_bridge import CvBridge
from collections import defaultdict

NS = 1_000_000_000

IMG_EXTS = (".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif")


def ns_to_rospy_time(dt_ns: int) -> rospy.Time:
    if dt_ns < 0:
        dt_ns = 0
    return rospy.Time(secs=int(dt_ns // NS), nsecs=int(dt_ns % NS))


def ts_from_name_ns(filename: str) -> int:
    base = os.path.splitext(os.path.basename(filename))[0]
    # accept "1234567890123456789" (ns) or "1234.56789" (sec)
    if re.fullmatch(r"\d+", base):
        return int(base)
    if re.fullmatch(r"\d+\.\d+", base):
        # seconds -> ns
        return int(float(base) * NS)
    raise ValueError(f"Cannot parse timestamp from filename: {filename}")


def list_images_ns(dirpath: str):
    items = []
    for name in os.listdir(dirpath):
        if name.lower().endswith(IMG_EXTS):
            t_ns = ts_from_name_ns(name)
            items.append((t_ns, os.path.join(dirpath, name)))
    items.sort(key=lambda x: x[0])
    return items


def list_images_from_csv(dirpath: str):
    csv_path = os.path.join(dirpath, "data.csv")
    if not os.path.isfile(csv_path):
        return None

    items = []
    with open(csv_path, "r", newline="") as f:
        r = csv.reader(f)
        for row in r:
            if not row:
                continue
            if row[0].startswith("#") or row[0].startswith("timestamp") or len(row) < 2:
                continue
            t_ns = int(float(row[0]))
            path = os.path.join(dirpath, row[1])
            items.append((t_ns, path))

    items.sort(key=lambda x: x[0])
    return items


def load_camera_items(dirpath: str):
    items = list_images_from_csv(dirpath)
    if items is not None:
        return items
    return list_images_ns(dirpath)


def read_imu_csv_ns(path: str):
    """
    Returns list of tuples:
      (t_ns, wx, wy, wz, ax, ay, az)
    matching your recorder header:
      t, w_x, w_y, w_z, a_x, a_y, a_z
    """
    out = []
    with open(path, "r", newline="") as f:
        r = csv.reader(f)
        for row in r:
            if not row:
                continue
            # skip header / comment lines
            if row[0].startswith("#") or row[0].startswith("timestamp") or row[0].startswith("time"):
                continue
            if len(row) < 7:
                continue

            # timestamp is ns integer in your data
            t_ns = int(float(row[0]))

            # IMPORTANT: gyro first, accel last (matches your C++)
            wx, wy, wz = map(float, row[1:4])
            ax, ay, az = map(float, row[4:7])

            out.append((t_ns, wx, wy, wz, ax, ay, az))

    out.sort(key=lambda x: x[0])
    return out


def ensure_dir(p: str):
    if not os.path.isdir(p):
        raise RuntimeError(f"Directory not found: {p}")


def main():
    # ---- edit these paths if needed ----
    cam0_dir = "/data/calib_B/cam0"
    cam1_dir = "/data/calib_B/cam1"
    imu_csv  = "/data/calib_B/imu.csv"
    out_bag  = "/data/calib_B_fixed.bag"
    # -----------------------------------

    ensure_dir(cam0_dir)
    ensure_dir(cam1_dir)
    if not os.path.isfile(imu_csv):
        raise RuntimeError(f"IMU csv not found: {imu_csv}")

    cam0 = load_camera_items(cam0_dir)
    cam1 = load_camera_items(cam1_dir)
    imu  = read_imu_csv_ns(imu_csv)

    if not cam0:
        raise RuntimeError(f"No images in {cam0_dir}")
    if not cam1:
        raise RuntimeError(f"No images in {cam1_dir}")
    if not imu:
        raise RuntimeError(f"No imu samples in {imu_csv}")

    # t0 in ns (use earliest among all)
    t0_ns = min(cam0[0][0], cam1[0][0], imu[0][0])

    bridge = CvBridge()

    # Only enforce monotonicity for IMU if needed
    last_ts = defaultdict(lambda: -1)
    skipped_cam_dups = defaultdict(int)

    def imu_make_strictly_increasing(dt_ns: int) -> int:
        if dt_ns <= last_ts["/imu0"]:
            dt_ns = last_ts["/imu0"] + 1
        last_ts["/imu0"] = dt_ns
        return dt_ns

    print(f"[info] cam0 imgs={len(cam0)} cam1 imgs={len(cam1)} imu={len(imu)}")
    print(f"[info] t0_ns={t0_ns}")
    print(f"[info] writing: {out_bag}")

    with rosbag.Bag(out_bag, "w") as bag:
        # ---- write cameras (preserve recorder timestamps as-is) ----
        for topic, items, frame_id in [
            ("/cam0/image_raw", cam0, "cam0"),
            ("/cam1/image_raw", cam1, "cam1"),
        ]:
            for t_ns, path in items:
                dt_ns = t_ns - t0_ns
                if dt_ns < 0:
                    continue

                if dt_ns <= last_ts[topic]:
                    skipped_cam_dups[topic] += 1
                    print(
                        f"[warn] skip non-increasing {topic} stamp={dt_ns} "
                        f"last={last_ts[topic]} path={path}"
                    )
                    continue

                ts = ns_to_rospy_time(dt_ns)

                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    raise RuntimeError(f"Failed to read image: {path}")

                msg = bridge.cv2_to_imgmsg(img, encoding="mono8")
                msg.header = Header(stamp=ts, frame_id=frame_id)

                bag.write(topic, msg, ts)
                last_ts[topic] = dt_ns

        # ---- write IMU ----
        for t_ns, wx, wy, wz, ax, ay, az in imu:
            dt_ns = t_ns - t0_ns
            if dt_ns < 0:
                continue

            dt_ns = imu_make_strictly_increasing(dt_ns)
            ts = ns_to_rospy_time(dt_ns)

            m = Imu()
            m.header = Header(stamp=ts, frame_id="imu0")
            m.angular_velocity.x = wx
            m.angular_velocity.y = wy
            m.angular_velocity.z = wz
            m.linear_acceleration.x = ax
            m.linear_acceleration.y = ay
            m.linear_acceleration.z = az

            bag.write("/imu0", m, ts)

    for topic in ["/cam0/image_raw", "/cam1/image_raw"]:
        if skipped_cam_dups[topic]:
            print(f"[warn] skipped {skipped_cam_dups[topic]} duplicate frames on {topic}")

    print("[done] wrote:", out_bag)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[error]", e, file=sys.stderr)
        sys.exit(1)
