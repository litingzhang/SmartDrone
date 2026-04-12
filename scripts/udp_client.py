import socket, struct, time
import numpy as np
import cv2

HDR_FMT = "<I H B B I d I H H I I"  # must match PacketHeader packed layout
HDR_SIZE = struct.calcsize(HDR_FMT)
MAGIC = 0x5643494D

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5000))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)

# key: (cam, frame_id) -> dict
frames = {}

def try_assemble(cam, fid):
    key = (cam, fid)
    f = frames.get(key)
    if not f: return None
    if len(f["chunks"]) != f["chunk_cnt"]:
        return None
    data = bytearray(f["total_size"])
    for ci, payload in f["chunks"].items():
        off = ci * f["max_payload"]
        data[off:off+len(payload)] = payload
    del frames[key]
    return bytes(data), f["frame_t"], f["seq"]

while True:
    pkt, addr = sock.recvfrom(65535)
    if len(pkt) < HDR_SIZE: 
        continue
    h = struct.unpack(HDR_FMT, pkt[:HDR_SIZE])
    magic, ver, cam, flags, seq, frame_t, fid, ci, ccnt, total, csz = h
    if magic != MAGIC or ver != 1: 
        continue
    payload = pkt[HDR_SIZE:HDR_SIZE+csz]
    key = (cam, fid)
    f = frames.get(key)
    if f is None:
        frames[key] = f = {
            "chunk_cnt": ccnt,
            "total_size": total,
            "chunks": {},
            "frame_t": frame_t,
            "seq": seq,
            "ts": time.time(),
            "max_payload": 1200,  # keep consistent with sender for offset calc
        }
    f["chunks"][ci] = payload

    out = try_assemble(cam, fid)
    if out:
        jpeg_bytes, t, s = out
        img = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            cv2.imshow("L" if cam==0 else "R", img)
            cv2.waitKey(1)

    # cleanup old
    now = time.time()
    for k in list(frames.keys()):
        if now - frames[k]["ts"] > 1.0:
            del frames[k]
