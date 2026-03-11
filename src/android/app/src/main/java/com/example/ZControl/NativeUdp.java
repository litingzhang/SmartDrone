package com.example.ZControl;

public class NativeUdp {
    static { System.loadLibrary("cm5udp"); }

    public static native boolean init(String ip, int cmdPort, int videoPort);
    public static native void close();

    public static native int sendCmd(int cmd);
    // MOVE position mode payload semantics (FRAME_NED): x, y, z, yaw, maxV
    public static native int sendMove(int frameType, float x, float y, float z, float yaw, float maxV);
    // MOVE velocity mode payload semantics (FRAME_NED): vx, vy, vz, yawRate, maxV
    public static native int sendMoveVelocity(int frameType, float vx, float vy, float vz, float yawRate, float maxV);
    public static native byte[] pollRecv();
}
