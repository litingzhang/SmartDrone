package com.example.ZControl;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;

import java.util.Arrays;
import java.util.Locale;

public class MainActivity extends Activity {

    private static final int CMD_ARM = 0x10;
    private static final int CMD_DISARM = 0x11;
    private static final int CMD_OFFBOARD = 0x12;
    private static final int CMD_HOLD = 0x13;
    private static final int CMD_LAND = 0x14;
    private static final int CMD_CALIB_CLEAN = 0x32;
    private static final int CMD_ACK = 0xF0;
    private static final int CMD_STATE = 0xF1;

    private static final int MODE_IDLE = 0;
    private static final int MODE_SLAM = 1;
    private static final int MODE_CALIB = 2;

    private static final int FRAME_NED = 2;
    private static final long JOYSTICK_PERIOD_MS = 50L;
    private static final long RX_POLL_PERIOD_MS = 5L;
    private static final int VIDEO_MAGIC = 0x5643494D;
    private static final int VIDEO_HEADER_LEN = 36;
    private static final int MAX_RX_PACKETS_PER_TICK = 96;
    private static final int MAX_VIDEO_JPEG_BYTES = 2 * 1024 * 1024;
    private static final int VIDEO_FLAG_FEATURE_POINTS = 0x01;
    private static final double FRAME_MATCH_TOLERANCE_SEC = 0.002;
    private static final float DEADZONE = 0.08f;
    private static final float XY_SPEED_SCALE_MPS = 1.0f;
    private static final float Z_SPEED_SCALE_MPS = 0.8f;
    private static final float YAW_RATE_SCALE_RADPS = 1.0f;
    private static final String KEY_MANUAL_MODE = "manualMode";

    private ImageView m_ivVideoLeft;
    private ImageView m_ivVideoRight;
    private TextView m_tvStatus;
    private TextView m_tvPose;
    private TextView m_tvVideoStats;
    private TextView m_tvJoystickState;
    private View m_pageManual;
    private View m_pageCommand;
    private Button m_btnModeToggle;
    private Button m_btnModeToggleCommand;
    private Button m_btnArmToggle;
    private Button m_btnAltitude;
    private Button m_btnPosition;
    private Button m_btnOffboard;
    private Button m_btnHold;
    private Button m_btnLand;
    private Button m_btnToggleSlam;
    private Button m_btnToggleCalib;

    private EditText m_etVehicleIp;
    private EditText m_etCfgExposure;
    private EditText m_etCfgGain;

    private JoystickView m_joystickLeft;
    private JoystickView m_joystickRight;

    private final Handler m_handler = new Handler(Looper.getMainLooper());

    private volatile float m_leftX;
    private volatile float m_leftY;
    private volatile float m_rightX;
    private volatile float m_rightY;
    private volatile boolean m_leftActive;
    private volatile boolean m_rightActive;

    private long m_lastJoystickTickMs;
    private boolean m_joystickLoopRunning;
    private boolean m_lastJoystickActive;
    private boolean m_isManualMode = false;
    private boolean m_rxLoopRunning;
    private int m_runtimeMode = MODE_IDLE;
    private String m_vehicleIp = "192.168.0.103";
    private boolean m_armLatched = false;
    private String m_flightAction = "";
    private int m_videoPktCount = 0;
    private int m_videoFrameOk = 0;
    private int m_videoDecodeFail = 0;
    private int m_videoInvalidPkt = 0;
    private int m_videoCamFrameOk0 = 0;
    private int m_videoCamFrameOk1 = 0;
    private long m_lastVideoStatsMs = 0L;
    private long m_lastVideoPacketMs = 0L;
    private int m_featurePktCount = 0;
    private int m_featureMatchCount = 0;

    private final Paint m_featurePaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private static final class VideoAssembly {
        int frameId = -1;
        double frameTimeSec = Double.NaN;
        int chunkCount;
        int totalSize;
        byte[][] chunks;
        boolean[] chunkSeen;
        int chunkReceived;
        int byteReceived;

        void reset() {
            frameId = -1;
            frameTimeSec = Double.NaN;
            chunkCount = 0;
            totalSize = 0;
            chunks = null;
            chunkSeen = null;
            chunkReceived = 0;
            byteReceived = 0;
        }
    }

    private static final class DisplayFrame {
        int frameId = -1;
        double frameTimeSec = Double.NaN;
        Bitmap bitmap;
        int overlayFrameId = -1;
    }

    private static final class FeatureFrame {
        int frameId = -1;
        double frameTimeSec = Double.NaN;
        int width;
        int height;
        int[] xs;
        int[] ys;
        int count;

        void reset() {
            frameId = -1;
            frameTimeSec = Double.NaN;
            width = 0;
            height = 0;
            xs = null;
            ys = null;
            count = 0;
        }
    }

    private final VideoAssembly[] m_videoAssemblies =
            new VideoAssembly[]{new VideoAssembly(), new VideoAssembly()};
    private final DisplayFrame[] m_displayFrames =
            new DisplayFrame[]{new DisplayFrame(), new DisplayFrame()};
    private final FeatureFrame[] m_featureFrames =
            new FeatureFrame[]{new FeatureFrame(), new FeatureFrame()};

    private final Runnable m_joystickLoop = new Runnable() {
        @Override
        public void run() {
            if (!m_joystickLoopRunning) {
                return;
            }
            tickJoystickControl();
            m_handler.postDelayed(this, JOYSTICK_PERIOD_MS);
        }
    };

    private final Runnable m_rxLoop = new Runnable() {
        @Override
        public void run() {
            if (!m_rxLoopRunning) {
                return;
            }
            tickRxLoop();
            m_handler.postDelayed(this, RX_POLL_PERIOD_MS);
        }
    };

    private static float parseF(EditText et, float defVal) {
        try {
            String s = et.getText().toString().trim();
            if (s.isEmpty()) {
                return defVal;
            }
            return Float.parseFloat(s);
        } catch (Throwable t) {
            return defVal;
        }
    }

    private static int parseI(EditText et, int defVal) {
        try {
            String s = et.getText().toString().trim();
            if (s.isEmpty()) {
                return defVal;
            }
            return Integer.parseInt(s);
        } catch (Throwable t) {
            return defVal;
        }
    }

    private static float applyDeadzone(float v) {
        return (Math.abs(v) < DEADZONE) ? 0f : v;
    }

    private static float clamp01(float v) {
        return Math.max(0f, Math.min(1f, v));
    }

    private void sendSimpleCmd(String name, int cmd) {
        try {
            int seq = NativeUdp.sendCmd(cmd);
            m_tvStatus.setText(name + " sent seq=" + seq + " cmd=0x" + Integer.toHexString(cmd));
        } catch (Throwable t) {
            m_tvStatus.setText(name + " error: " + t.getMessage());
        }
    }

    private void sendHoldBurst(int count, String reason) {
        for (int i = 0; i < count; ++i) {
            sendSimpleCmd(reason + "[" + (i + 1) + "/" + count + "]", CMD_HOLD);
        }
    }

    private void sendMoveVelocityCommand(float vx, float vy, float vz, float yawRate, float maxV, String reason) {
        try {
            int seq = NativeUdp.sendMoveVelocity(FRAME_NED, vx, vy, vz, yawRate, maxV);
            m_tvStatus.setText(String.format(Locale.US,
                    "%s seq=%d vx=%.2f vy=%.2f vz=%.2f yawRate=%.2f maxV=%.2f",
                    reason, seq, vx, vy, vz, yawRate, maxV));
        } catch (Throwable t) {
            m_tvStatus.setText(reason + " error: " + t.getMessage());
        }
    }

    private int sendRuntimeConfig() {
        int exposureUs = parseI(m_etCfgExposure, 6000);
        float gain = parseF(m_etCfgGain, 4.0f);
        try {
            int seq = NativeUdp.sendRuntimeConfig(exposureUs, gain);
            m_tvStatus.setText(String.format(Locale.US,
                    "CFG seq=%d exp=%d gain=%.1f",
                    seq, exposureUs, gain));
            return seq;
        } catch (Throwable t) {
            m_tvStatus.setText("CFG error: " + t.getMessage());
            return -1;
        }
    }

    private boolean ensureVehicleConnection() {
        String vehicleIp = m_etVehicleIp.getText().toString().trim();
        if (vehicleIp.isEmpty()) {
            m_tvStatus.setText("Vehicle IP is empty");
            return false;
        }
        if (vehicleIp.equals(m_vehicleIp)) {
            return true;
        }
        try {
            NativeUdp.close();
            boolean ok = NativeUdp.init(vehicleIp, 14550, 5000);
            if (!ok) {
                m_tvStatus.setText("Reconnect failed: " + vehicleIp);
                return false;
            }
            m_vehicleIp = vehicleIp;
            return true;
        } catch (Throwable t) {
            m_tvStatus.setText("Reconnect error: " + t.getMessage());
            return false;
        }
    }

    private void setButtonState(Button button, boolean active, String color) {
        if (button == null) {
            return;
        }
        button.setBackgroundColor(Color.parseColor(color));
        button.setAlpha(active ? 1.0f : 0.35f);
        button.setTextColor(Color.WHITE);
    }

    private void updateRuntimeButtons() {
        if (m_btnToggleSlam != null) {
            m_btnToggleSlam.setText(m_runtimeMode == MODE_SLAM ? "Stop VIO" : "Start VIO");
            setButtonState(m_btnToggleSlam, m_runtimeMode == MODE_SLAM, "#2E7D32");
        }
        if (m_btnToggleCalib != null) {
            m_btnToggleCalib.setText(m_runtimeMode == MODE_CALIB ? "Stop Calib" : "Start Calib");
            setButtonState(m_btnToggleCalib, m_runtimeMode == MODE_CALIB, "#1565C0");
        }
    }

    private void updateFlightButtons() {
        if (m_btnArmToggle != null) {
            m_btnArmToggle.setText(m_armLatched ? "DISARM" : "ARM");
            setButtonState(m_btnArmToggle, m_armLatched, "#C62828");
        }
        if (m_btnAltitude != null) {
            setButtonState(m_btnAltitude, "ALTITUDE".equals(m_flightAction), "#3949AB");
        }
        if (m_btnPosition != null) {
            setButtonState(m_btnPosition, "POSITION".equals(m_flightAction), "#283593");
        }
        if (m_btnOffboard != null) {
            setButtonState(m_btnOffboard, "OFFBOARD".equals(m_flightAction), "#6A1B9A");
        }
        if (m_btnHold != null) {
            setButtonState(m_btnHold, "HOLD".equals(m_flightAction), "#00897B");
        }
        if (m_btnLand != null) {
            setButtonState(m_btnLand, "LAND".equals(m_flightAction), "#EF6C00");
        }
    }

    private void sendRuntimeMode(int mode, String label) {
        if (!ensureVehicleConnection()) {
            return;
        }
        int cfgSeq = sendRuntimeConfig();
        if (cfgSeq < 0) {
            return;
        }
        try {
            int seq = NativeUdp.sendRuntimeMode(mode);
            m_runtimeMode = mode;
            updateRuntimeButtons();
            m_tvStatus.setText(label + " seq=" + seq + " after cfg=" + cfgSeq);
        } catch (Throwable t) {
            m_tvStatus.setText(label + " error: " + t.getMessage());
        }
    }

    private void stopRuntime(String label) {
        if (!ensureVehicleConnection()) {
            return;
        }
        try {
            int seq = NativeUdp.sendRuntimeMode(MODE_IDLE);
            m_runtimeMode = MODE_IDLE;
            updateRuntimeButtons();
            m_tvStatus.setText(label + " seq=" + seq);
        } catch (Throwable t) {
            m_tvStatus.setText(label + " error: " + t.getMessage());
        }
    }

    private static int readU16Le(byte[] data, int offset) {
        return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8);
    }

    private static long readU32Le(byte[] data, int offset) {
        return ((long) data[offset] & 0xFFL)
                | (((long) data[offset + 1] & 0xFFL) << 8)
                | (((long) data[offset + 2] & 0xFFL) << 16)
                | (((long) data[offset + 3] & 0xFFL) << 24);
    }

    private static int readI16Le(byte[] data, int offset) {
        int v = readU16Le(data, offset);
        return (v >= 0x8000) ? (v - 0x10000) : v;
    }

    private static int readI32Le(byte[] data, int offset) {
        return (data[offset] & 0xFF)
                | ((data[offset + 1] & 0xFF) << 8)
                | ((data[offset + 2] & 0xFF) << 16)
                | ((data[offset + 3] & 0xFF) << 24);
    }

    private static long readI64Le(byte[] data, int offset) {
        return ((long) data[offset] & 0xFFL)
                | (((long) data[offset + 1] & 0xFFL) << 8)
                | (((long) data[offset + 2] & 0xFFL) << 16)
                | (((long) data[offset + 3] & 0xFFL) << 24)
                | (((long) data[offset + 4] & 0xFFL) << 32)
                | (((long) data[offset + 5] & 0xFFL) << 40)
                | (((long) data[offset + 6] & 0xFFL) << 48)
                | (((long) data[offset + 7] & 0xFFL) << 56);
    }

    private static float readF32Le(byte[] data, int offset) {
        return Float.intBitsToFloat(readI32Le(data, offset));
    }

    private static double readF64Le(byte[] data, int offset) {
        return Double.longBitsToDouble(readI64Le(data, offset));
    }

    private static String ackStatusToText(int status) {
        switch (status) {
            case 0:
                return "ACK_OK";
            case -1:
                return "ACK_E_BAD_CRC";
            case -2:
                return "ACK_E_BAD_LEN";
            case -3:
                return "ACK_E_BAD_ARGS";
            case -4:
                return "ACK_E_BAD_STATE";
            case -5:
                return "ACK_E_UNKNOWN";
            case -6:
                return "ACK_E_INTERNAL";
            default:
                return "STATUS(" + status + ")";
        }
    }

    private String decodeTlvAck(byte[] rx) {
        if (rx == null) {
            return "RX: null";
        }
        if (rx.length < 17) {
            return "RX short: " + rx.length;
        }
        if ((rx[0] & 0xFF) != 0xAA || (rx[1] & 0xFF) != 0x55) {
            return "RX(no sync): " + rx.length;
        }
        int ver = rx[2] & 0xFF;
        int cmd = rx[3] & 0xFF;
        int flags = rx[4] & 0xFF;
        int len = readU16Le(rx, 5);
        long seq = readU32Le(rx, 7);
        long tMs = readU32Le(rx, 11);
        int total = 2 + (1 + 1 + 1 + 2 + 4 + 4) + len + 2;
        if (rx.length < total) {
            return String.format(Locale.US,
                    "RX partial ver=%d cmd=0x%02X len=%d bytes=%d need=%d",
                    ver, cmd, len, rx.length, total);
        }
        if (cmd != CMD_ACK || len < 9) {
            return String.format(Locale.US,
                    "RX TLV ver=%d cmd=0x%02X flags=%d len=%d seq=%d tMs=%d",
                    ver, cmd, flags, len, seq, tMs);
        }
        int payloadOffset = 15;
        int ackCmd = rx[payloadOffset] & 0xFF;
        long ackSeq = readU32Le(rx, payloadOffset + 1);
        int status = readI16Le(rx, payloadOffset + 5);
        int reserved = readU16Le(rx, payloadOffset + 7);
        return String.format(Locale.US,
                "ACK ver=%d reqSeq=%d tMs=%d ackCmd=0x%02X ackSeq=%d status=%s reserved=%d",
                ver, seq, tMs, ackCmd, ackSeq, ackStatusToText(status), reserved);
    }

    private String runtimeModeToText(int mode) {
        switch (mode) {
            case MODE_SLAM:
                return "SLAM";
            case MODE_CALIB:
                return "CALIB";
            default:
                return "IDLE";
        }
    }

    private boolean tryHandleStatePacket(byte[] rx) {
        if (!isTlvPacket(rx) || rx.length < 15) {
            return false;
        }
        int cmd = rx[3] & 0xFF;
        int len = readU16Le(rx, 5);
        if (cmd != CMD_STATE || len != 32 || rx.length < 15 + len + 2) {
            return false;
        }
        int payloadOffset = 15;
        int runtimeMode = rx[payloadOffset] & 0xFF;
        int trackingState = rx[payloadOffset + 1] & 0xFF;
        float x = readF32Le(rx, payloadOffset + 4);
        float y = readF32Le(rx, payloadOffset + 8);
        float z = readF32Le(rx, payloadOffset + 12);
        float qw = readF32Le(rx, payloadOffset + 16);
        float qx = readF32Le(rx, payloadOffset + 20);
        float qy = readF32Le(rx, payloadOffset + 24);
        float qz = readF32Le(rx, payloadOffset + 28);
        if (m_tvPose != null) {
            if (runtimeMode == MODE_SLAM) {
                m_tvPose.setText(String.format(Locale.US,
                        "Pose %s trk=%d\np[%.2f %.2f %.2f]\nq[%.2f %.2f %.2f %.2f]",
                        runtimeModeToText(runtimeMode), trackingState, x, y, z, qw, qx, qy, qz));
            } else if (runtimeMode == MODE_CALIB) {
                m_tvPose.setText("Pose hidden in CALIB");
            } else {
                m_tvPose.setText("Pose idle");
            }
        }
        return true;
    }

    private boolean tryHandleVideoPacket(byte[] rx) {
        if (rx == null || rx.length < VIDEO_HEADER_LEN) {
            return false;
        }
        if (readI32Le(rx, 0) != VIDEO_MAGIC) {
            return false;
        }
        m_lastVideoPacketMs = System.currentTimeMillis();
        if (readU16Le(rx, 4) != 1) {
            m_videoInvalidPkt++;
            return false;
        }
        int camIndex = rx[6] & 0xFF;
        int flags = rx[7] & 0xFF;
        if (camIndex < 0 || camIndex >= m_videoAssemblies.length) {
            m_videoInvalidPkt++;
            return true;
        }
        if ((flags & VIDEO_FLAG_FEATURE_POINTS) != 0) {
            return tryHandleFeaturePacket(rx, camIndex);
        }
        m_videoPktCount++;
        double frameTimeSec = readF64Le(rx, 12);
        int frameId = readI32Le(rx, 20);
        int chunkIdx = readU16Le(rx, 24);
        int chunkCnt = readU16Le(rx, 26);
        int totalSize = readI32Le(rx, 28);
        int chunkSize = readI32Le(rx, 32);
        int payloadLen = rx.length - VIDEO_HEADER_LEN;
        if (chunkCnt <= 0 || chunkCnt > 256 || chunkIdx < 0 || chunkIdx >= chunkCnt) {
            m_videoInvalidPkt++;
            return true;
        }
        if (totalSize <= 0 || totalSize > MAX_VIDEO_JPEG_BYTES || chunkSize != payloadLen) {
            m_videoInvalidPkt++;
            return true;
        }

        VideoAssembly assembly = m_videoAssemblies[camIndex];
        boolean needReset = frameId != assembly.frameId
                || assembly.chunks == null
                || assembly.chunkCount != chunkCnt
                || assembly.totalSize != totalSize;
        if (needReset) {
            assembly.frameId = frameId;
            assembly.frameTimeSec = frameTimeSec;
            assembly.chunkCount = chunkCnt;
            assembly.totalSize = totalSize;
            assembly.chunks = new byte[chunkCnt][];
            assembly.chunkSeen = new boolean[chunkCnt];
            assembly.chunkReceived = 0;
            assembly.byteReceived = 0;
        }

        if (!assembly.chunkSeen[chunkIdx]) {
            byte[] payload = Arrays.copyOfRange(rx, VIDEO_HEADER_LEN, rx.length);
            assembly.chunks[chunkIdx] = payload;
            assembly.chunkSeen[chunkIdx] = true;
            assembly.chunkReceived += 1;
            assembly.byteReceived += payload.length;
        }
        if (assembly.chunkReceived != assembly.chunkCount || assembly.byteReceived <= 0) {
            return true;
        }

        byte[] jpeg = new byte[assembly.byteReceived];
        int offset = 0;
        for (int i = 0; i < assembly.chunkCount; ++i) {
            byte[] chunk = assembly.chunks[i];
            if (chunk == null) {
                return true;
            }
            System.arraycopy(chunk, 0, jpeg, offset, chunk.length);
            offset += chunk.length;
        }

        Bitmap bitmap = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
        ImageView target = (camIndex == 0) ? m_ivVideoLeft : m_ivVideoRight;
        if (bitmap != null && target != null) {
            DisplayFrame displayFrame = m_displayFrames[camIndex];
            displayFrame.frameId = frameId;
            displayFrame.frameTimeSec = frameTimeSec;
            displayFrame.bitmap = bitmap;
            displayFrame.overlayFrameId = -1;
            renderVideoFrame(camIndex);
            m_videoFrameOk++;
            if (camIndex == 0) {
                m_videoCamFrameOk0++;
            } else {
                m_videoCamFrameOk1++;
            }
        } else {
            m_videoDecodeFail++;
        }
        assembly.reset();
        return true;
    }

    private boolean tryHandleFeaturePacket(byte[] rx, int camIndex) {
        int frameId = readI32Le(rx, 20);
        int chunkIdx = readU16Le(rx, 24);
        int chunkCnt = readU16Le(rx, 26);
        int totalSize = readI32Le(rx, 28);
        int payloadLen = rx.length - VIDEO_HEADER_LEN;
        if (chunkIdx != 0 || chunkCnt != 1 || totalSize != payloadLen || payloadLen < 6) {
            m_videoInvalidPkt++;
            return true;
        }
        int width = readU16Le(rx, VIDEO_HEADER_LEN);
        int height = readU16Le(rx, VIDEO_HEADER_LEN + 2);
        int count = readU16Le(rx, VIDEO_HEADER_LEN + 4);
        if (width <= 0 || height <= 0 || count < 0 || payloadLen != 6 + count * 4) {
            m_videoInvalidPkt++;
            return true;
        }
        FeatureFrame featureFrame = m_featureFrames[camIndex];
        featureFrame.frameId = frameId;
        featureFrame.frameTimeSec = readF64Le(rx, 12);
        featureFrame.width = width;
        featureFrame.height = height;
        featureFrame.count = count;
        featureFrame.xs = new int[count];
        featureFrame.ys = new int[count];
        int cursor = VIDEO_HEADER_LEN + 6;
        for (int i = 0; i < count; ++i) {
            featureFrame.xs[i] = readU16Le(rx, cursor);
            featureFrame.ys[i] = readU16Le(rx, cursor + 2);
            cursor += 4;
        }
        m_featurePktCount++;
        renderVideoFrame(camIndex);
        return true;
    }

    private void renderVideoFrame(int camIndex) {
        DisplayFrame displayFrame = m_displayFrames[camIndex];
        ImageView target = (camIndex == 0) ? m_ivVideoLeft : m_ivVideoRight;
        if (displayFrame.bitmap == null || target == null) {
            return;
        }
        Bitmap output = displayFrame.bitmap;
        FeatureFrame featureFrame = m_featureFrames[camIndex];
        if (featureFrame.xs != null
                && Math.abs(featureFrame.frameTimeSec - displayFrame.frameTimeSec) <= FRAME_MATCH_TOLERANCE_SEC) {
            output = overlayFeaturePoints(displayFrame.bitmap, featureFrame);
            if (displayFrame.overlayFrameId != featureFrame.frameId) {
                displayFrame.overlayFrameId = featureFrame.frameId;
                m_featureMatchCount++;
            }
        }
        target.setImageBitmap(output);
    }

    private Bitmap overlayFeaturePoints(Bitmap source, FeatureFrame featureFrame) {
        Bitmap mutable = source.copy(Bitmap.Config.ARGB_8888, true);
        if (mutable == null) {
            return source;
        }
        Canvas canvas = new Canvas(mutable);
        float scaleX = (featureFrame.width > 0) ? ((float) mutable.getWidth() / (float) featureFrame.width) : 1.0f;
        float scaleY = (featureFrame.height > 0) ? ((float) mutable.getHeight() / (float) featureFrame.height) : 1.0f;
        for (int i = 0; i < featureFrame.count; ++i) {
            float x = featureFrame.xs[i] * scaleX;
            float y = featureFrame.ys[i] * scaleY;
            canvas.drawCircle(x, y, 5.0f, m_featurePaint);
        }
        return mutable;
    }

    private boolean isTlvPacket(byte[] rx) {
        return rx != null && rx.length >= 4
                && (rx[0] & 0xFF) == 0xAA
                && (rx[1] & 0xFF) == 0x55;
    }

    private void tickRxLoop() {
        for (int i = 0; i < MAX_RX_PACKETS_PER_TICK; ++i) {
            byte[] rx;
            try {
                rx = NativeUdp.pollRecv();
            } catch (Throwable t) {
                m_tvStatus.setText("rx error: " + t.getMessage());
                updateVideoStatsView();
                return;
            }
            if (rx == null) {
                updateVideoStatsView();
                return;
            }
            if (tryHandleVideoPacket(rx)) {
                continue;
            }
            if (tryHandleStatePacket(rx)) {
                continue;
            }
            if (isTlvPacket(rx)) {
                m_tvStatus.setText(decodeTlvAck(rx));
            }
        }
        updateVideoStatsView();
    }

    private void updateVideoStatsView() {
        if (m_tvVideoStats == null) {
            return;
        }
        long nowMs = System.currentTimeMillis();
        if (nowMs - m_lastVideoStatsMs < 250L) {
            return;
        }
        m_lastVideoStatsMs = nowMs;
        final String lastSeen = (m_lastVideoPacketMs == 0L)
                ? "never"
                : String.format(Locale.US, "%dms", (nowMs - m_lastVideoPacketMs));
        m_tvVideoStats.setText(String.format(Locale.US,
                "Video pkt=%d feat=%d fuse=%d ok=%d fail=%d bad=%d L=%d R=%d last=%s",
                m_videoPktCount, m_featurePktCount, m_featureMatchCount,
                m_videoFrameOk, m_videoDecodeFail,
                m_videoInvalidPkt, m_videoCamFrameOk0, m_videoCamFrameOk1, lastSeen));
    }

    private void tickJoystickControl() {
        long nowMs = System.currentTimeMillis();
        if (m_lastJoystickTickMs == 0L) {
            m_lastJoystickTickMs = nowMs;
            return;
        }
        float dtSec = (nowMs - m_lastJoystickTickMs) / 1000.0f;
        m_lastJoystickTickMs = nowMs;
        if (dtSec <= 0f || dtSec > 0.5f) {
            dtSec = JOYSTICK_PERIOD_MS / 1000.0f;
        }

        float leftX = applyDeadzone(m_leftX);
        float leftY = applyDeadzone(m_leftY);
        float rightX = applyDeadzone(m_rightX);
        float rightY = applyDeadzone(m_rightY);

        float leftMag = clamp01((float) Math.hypot(leftX, leftY));
        float rightMag = clamp01((float) Math.hypot(rightX, rightY));
        float rightVerticalMag = clamp01(Math.abs(rightY));
        boolean active = m_leftActive || m_rightActive
                || leftX != 0f || leftY != 0f || rightX != 0f || rightY != 0f;

        m_tvJoystickState.setText(String.format(Locale.US,
                "L[x=%.2f y=%.2f mag=%.2f] R[x=%.2f y=%.2f mag=%.2f] %s",
                leftX, leftY, leftMag, rightX, rightY, rightMag, active ? "ACTIVE" : "CENTER"));

        if (!active) {
            if (m_lastJoystickActive) {
                m_lastJoystickActive = false;
                sendHoldBurst(3, "HOLD(center)");
            }
            return;
        }
        m_lastJoystickActive = true;

        float baseMaxV = 0.6f;
        float dynamicMaxV = baseMaxV * clamp01(Math.max(leftMag, Math.max(rightVerticalMag, Math.abs(rightX))));
        if (dynamicMaxV < 0.05f) {
            dynamicMaxV = 0.05f;
        }

        float vx = leftY * XY_SPEED_SCALE_MPS * baseMaxV;
        float vy = leftX * XY_SPEED_SCALE_MPS * baseMaxV;
        float vz = (-rightY) * Z_SPEED_SCALE_MPS * baseMaxV;
        float yawRate = rightX * YAW_RATE_SCALE_RADPS;
        sendMoveVelocityCommand(vx, vy, vz, yawRate, dynamicMaxV, "JOY VEL");
    }

    private void startJoystickLoop() {
        if (m_joystickLoopRunning) {
            return;
        }
        m_joystickLoopRunning = true;
        m_lastJoystickTickMs = 0L;
        m_handler.post(m_joystickLoop);
    }

    private void stopJoystickLoop() {
        m_joystickLoopRunning = false;
        m_handler.removeCallbacks(m_joystickLoop);
    }

    private void startRxLoop() {
        if (m_rxLoopRunning) {
            return;
        }
        m_rxLoopRunning = true;
        m_handler.post(m_rxLoop);
    }

    private void stopRxLoop() {
        m_rxLoopRunning = false;
        m_handler.removeCallbacks(m_rxLoop);
    }

    private void resetJoysticksAndHold() {
        if (m_joystickLeft != null) {
            m_joystickLeft.Reset();
        }
        if (m_joystickRight != null) {
            m_joystickRight.Reset();
        }
        m_leftX = 0f;
        m_leftY = 0f;
        m_rightX = 0f;
        m_rightY = 0f;
        m_leftActive = false;
        m_rightActive = false;
        m_lastJoystickActive = false;
        sendHoldBurst(3, "HOLD(page)");
    }

    private void setManualPage(boolean manualMode) {
        if (m_pageManual == null || m_pageCommand == null) {
            return;
        }
        m_isManualMode = manualMode;
        if (!manualMode) {
            resetJoysticksAndHold();
        }
        m_pageManual.setVisibility(manualMode ? View.VISIBLE : View.GONE);
        m_pageCommand.setVisibility(manualMode ? View.GONE : View.VISIBLE);
        if (m_btnModeToggle != null) {
            m_btnModeToggle.setText("COMMAND");
        }
        if (m_btnModeToggleCommand != null) {
            m_btnModeToggleCommand.setText("MANUAL");
        }
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setFlags(
                WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);

        m_ivVideoLeft = findViewById(R.id.ivVideoLeft);
        m_ivVideoRight = findViewById(R.id.ivVideoRight);
        m_tvStatus = findViewById(R.id.tvStatus);
        m_tvPose = findViewById(R.id.tvPose);
        m_tvVideoStats = findViewById(R.id.tvVideoStats);
        m_tvJoystickState = findViewById(R.id.tvJoystickState);
        m_pageManual = findViewById(R.id.pageManual);
        m_pageCommand = findViewById(R.id.pageCommand);
        m_btnModeToggle = findViewById(R.id.btnModeToggle);
        m_btnModeToggleCommand = findViewById(R.id.btnModeToggleCommand);
        m_btnArmToggle = findViewById(R.id.btnArm);
        m_btnAltitude = findViewById(R.id.btnAltitude);
        m_btnPosition = findViewById(R.id.btnPosition);
        m_btnOffboard = findViewById(R.id.btnOffboard);
        m_btnHold = findViewById(R.id.btnHold);
        m_btnLand = findViewById(R.id.btnLand);
        m_btnToggleSlam = findViewById(R.id.btnToggleSlam);
        m_btnToggleCalib = findViewById(R.id.btnToggleCalib);
        m_featurePaint.setColor(Color.GREEN);
        m_featurePaint.setStyle(Paint.Style.STROKE);
        m_featurePaint.setStrokeWidth(2.0f);

        Button btnCleanCalib = findViewById(R.id.btnCleanCalib);

        m_etVehicleIp = findViewById(R.id.etVehicleIp);
        m_etCfgExposure = findViewById(R.id.etCfgExposure);
        m_etCfgGain = findViewById(R.id.etCfgGain);

        m_joystickLeft = findViewById(R.id.joystickLeft);
        m_joystickRight = findViewById(R.id.joystickRight);

        final String cm5Ip = "192.168.0.103";
        final int cm5CmdPort = 14550;
        final int phoneVideoPort = 5000;
        m_vehicleIp = cm5Ip;
        boolean ok;
        try {
            ok = NativeUdp.init(cm5Ip, cm5CmdPort, phoneVideoPort);
        } catch (Throwable t) {
            ok = false;
            m_tvStatus.setText("Native init error: " + t.getMessage());
        }
        if (ok) {
            m_tvStatus.setText("UDP ready cmd-> " + cm5Ip + ":" + cm5CmdPort + " video<-" + phoneVideoPort);
        }

        m_joystickLeft.SetOnStickChangedListener((xNorm, yNorm, active) -> {
            m_leftX = xNorm;
            m_leftY = yNorm;
            m_leftActive = active;
        });
        m_joystickRight.SetOnStickChangedListener((xNorm, yNorm, active) -> {
            m_rightX = xNorm;
            m_rightY = yNorm;
            m_rightActive = active;
        });

        m_btnModeToggle.setOnClickListener(v -> setManualPage(!m_isManualMode));
        if (m_btnModeToggleCommand != null) {
            m_btnModeToggleCommand.setOnClickListener(v -> setManualPage(!m_isManualMode));
        }
        if (savedInstanceState != null) {
            m_isManualMode = savedInstanceState.getBoolean(KEY_MANUAL_MODE, false);
        }
        setManualPage(m_isManualMode);

        if (m_btnArmToggle != null) {
            m_btnArmToggle.setOnClickListener(v -> {
                final boolean nextArm = !m_armLatched;
                sendSimpleCmd(nextArm ? "ARM" : "DISARM", nextArm ? CMD_ARM : CMD_DISARM);
                m_armLatched = nextArm;
                updateFlightButtons();
            });
        }
        if (m_btnOffboard != null) {
            m_btnOffboard.setOnClickListener(v -> {
                sendSimpleCmd("OFFBOARD", CMD_OFFBOARD);
                m_flightAction = "OFFBOARD";
                updateFlightButtons();
            });
        }
        if (m_btnAltitude != null) {
            m_btnAltitude.setOnClickListener(v -> {
                m_flightAction = "ALTITUDE";
                updateFlightButtons();
                m_tvStatus.setText("ALTITUDE mode selected");
            });
        }
        if (m_btnPosition != null) {
            m_btnPosition.setOnClickListener(v -> {
                m_flightAction = "POSITION";
                updateFlightButtons();
                m_tvStatus.setText("POSITION mode selected");
            });
        }
        if (m_btnHold != null) {
            m_btnHold.setOnClickListener(v -> {
                sendSimpleCmd("HOLD", CMD_HOLD);
                m_flightAction = "HOLD";
                updateFlightButtons();
            });
        }
        if (m_btnLand != null) {
            m_btnLand.setOnClickListener(v -> {
                sendSimpleCmd("LAND", CMD_LAND);
                m_flightAction = "LAND";
                updateFlightButtons();
            });
        }
        btnCleanCalib.setOnClickListener(v -> {
            if (!ensureVehicleConnection()) {
                return;
            }
            sendSimpleCmd("CLEAN_CALIB", CMD_CALIB_CLEAN);
        });

        m_btnToggleSlam.setOnClickListener(v -> {
            if (m_runtimeMode == MODE_SLAM) {
                stopRuntime("Stop VIO");
            } else {
                sendRuntimeMode(MODE_SLAM, "Start VIO");
            }
        });
        m_btnToggleCalib.setOnClickListener(v -> {
            if (m_runtimeMode == MODE_CALIB) {
                stopRuntime("Stop Calib");
            } else {
                sendRuntimeMode(MODE_CALIB, "Start Calib");
            }
        });
        updateRuntimeButtons();
        updateFlightButtons();

        startJoystickLoop();
        startRxLoop();
    }

    @Override
    protected void onResume() {
        super.onResume();
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    @Override
    protected void onDestroy() {
        stopRxLoop();
        stopJoystickLoop();
        super.onDestroy();
        try {
            NativeUdp.close();
        } catch (Throwable ignored) {
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        outState.putBoolean(KEY_MANUAL_MODE, m_isManualMode);
        super.onSaveInstanceState(outState);
    }
}
