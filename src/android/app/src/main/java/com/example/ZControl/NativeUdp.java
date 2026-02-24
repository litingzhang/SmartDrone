package com.example.ZControl;

public class NativeUdp {
    static { System.loadLibrary("cm5udp"); }

    public static native boolean init(String ip, int port);
    public static native void close();

    public static native int sendCmd(int cmd);
    public static native int sendMove(int frameType, float x, float y, float z, float yaw, float maxV);
    public static native byte[] pollRecv();
}
