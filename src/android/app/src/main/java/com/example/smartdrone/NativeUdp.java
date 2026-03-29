package com.example.smartdrone;

public class NativeUdp {
    static { System.loadLibrary("cm5udp"); }

    public static native boolean init(String ip, int cmdPort, int videoPort);
    public static native void close();

    public static native int sendCmd(int cmd);
    // MOVE position mode payload semantics (FRAME_NED): x, y, z, yaw, maxV
    public static native int sendMove(int frameType, float x, float y, float z, float yaw, float maxV);
    // MOVE velocity mode payload semantics (FRAME_NED): vx, vy, vz, yawRate, maxV
    public static native int sendMoveVelocity(int frameType, float vx, float vy, float vz, float yawRate, float maxV);
    // MOVE rc joystick payload semantics (FRAME_NED): throttle, yaw, pitch, roll, maxV
    public static native int sendMoveRcJoystick(
            int frameType,
            float throttle,
            float yaw,
            float pitch,
            float roll,
            float maxV);
    public static native int sendRuntimeMode(int mode);
    public static native int sendGetCapabilities();
    public static native int sendGetConfig();
    public static native int sendRuntimeConfig(
            int exposureUs,
            float gain,
            int pairMs,
            int slamFps,
            int sensorMode,
            boolean sendImage,
            boolean sendFeature,
            boolean sendMap);
    public static native byte[] pollRecv();
}
