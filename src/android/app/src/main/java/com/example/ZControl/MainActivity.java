package com.example.ZControl;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.WindowManager;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;

import java.util.Arrays;
import java.util.Locale;

public class MainActivity extends Activity {

    private static final int CMD_PING = 0x01;
    private static final int CMD_ARM = 0x10;
    private static final int CMD_DISARM = 0x11;
    private static final int CMD_OFFBOARD = 0x12;
    private static final int CMD_HOLD = 0x13;
    private static final int CMD_LAND = 0x14;
    private static final int CMD_ACK = 0xF0;

    private static final int FRAME_NED = 2;
    private static final long JOYSTICK_PERIOD_MS = 50L;
    private static final float DEADZONE = 0.08f;
    private static final float XY_SPEED_SCALE_MPS = 1.0f;
    private static final float Z_SPEED_SCALE_MPS = 0.8f;
    private static final float YAW_RATE_SCALE_RADPS = 1.0f;
    private static final String KEY_MANUAL_MODE = "manualMode";
    private static final long RX_POLL_PERIOD_MS = 20L;
    private static final int VIDEO_MAGIC = 0x5643494D;
    private static final int VIDEO_HEADER_LEN = 36;
    private static final int MAX_RX_PACKETS_PER_TICK = 16;
    private static final int MAX_VIDEO_JPEG_BYTES = 2 * 1024 * 1024;

    private ImageView m_ivVideoLeft;
    private ImageView m_ivVideoRight;
    private TextView m_tvStatus;
    private TextView m_tvJoystickState;

    private View m_pageManual;
    private View m_pageCommand;
    private Button m_btnModeToggle;

    private EditText m_etX;
    private EditText m_etY;
    private EditText m_etZ;
    private EditText m_etYaw;
    private EditText m_etMaxV;

    private JoystickView m_joystickLeft;
    private JoystickView m_joystickRight;

    private final Handler m_handler = new Handler(Looper.getMainLooper());

    private volatile float m_leftX;
    private volatile float m_leftY;
    private volatile float m_rightX;
    private volatile float m_rightY;
    private volatile boolean m_leftActive;
    private volatile boolean m_rightActive;

    private float m_lastCmdX;
    private float m_lastCmdY;
    private float m_lastCmdZ;
    private float m_lastCmdYaw;
    private long m_lastJoystickTickMs;

    private boolean m_joystickLoopRunning;
    private boolean m_lastJoystickActive;
    private boolean m_isManualMode = true;
    private boolean m_rxLoopRunning;
    private byte[] m_lastAckPacket;

    private static final class VideoAssembly {
        int frameId = -1;
        int chunkCount;
        int totalSize;
        byte[][] chunks;
        boolean[] chunkSeen;
        int chunkReceived;
        int byteReceived;

        void Reset() {
            frameId = -1;
            chunkCount = 0;
            totalSize = 0;
            chunks = null;
            chunkSeen = null;
            chunkReceived = 0;
            byteReceived = 0;
        }
    }

    private final VideoAssembly[] m_videoAssemblies =
            new VideoAssembly[]{new VideoAssembly(), new VideoAssembly()};

    private final Runnable m_joystickLoop = new Runnable() {
        @Override
        public void run() {
            if (!m_joystickLoopRunning) {
                return;
            }
            TickJoystickControl();
            m_handler.postDelayed(this, JOYSTICK_PERIOD_MS);
        }
    };

    private final Runnable m_rxLoop = new Runnable() {
        @Override
        public void run() {
            if (!m_rxLoopRunning) {
                return;
            }
            TickRxLoop();
            m_handler.postDelayed(this, RX_POLL_PERIOD_MS);
        }
    };

    private static float ParseF(EditText et, float defVal) {
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

    private static String Hex(byte[] bytes) {
        if (bytes == null) {
            return "null";
        }
        StringBuilder sb = new StringBuilder(bytes.length * 2);
        for (byte v : bytes) {
            sb.append(String.format(Locale.US, "%02X", v));
        }
        return sb.toString();
    }

    private static float ApplyDeadzone(float v) {
        return (Math.abs(v) < DEADZONE) ? 0f : v;
    }

    private static float Clamp01(float v) {
        return Math.max(0f, Math.min(1f, v));
    }

    private void SendHoldBurst(int count, String reason) {
        for (int i = 0; i < count; ++i) {
            SendSimpleCmd(reason + "[" + (i + 1) + "/" + count + "]", CMD_HOLD);
        }
    }

    private void SendSimpleCmd(String name, int cmd) {
        try {
            int seq = NativeUdp.sendCmd(cmd);
            m_tvStatus.setText(name + " sent seq=" + seq + " cmd=0x" + Integer.toHexString(cmd));
        } catch (Throwable t) {
            m_tvStatus.setText(name + " error: " + t.getMessage());
        }
    }

    private void SendMoveCommand(float x, float y, float z, float yaw, float maxV, String reason) {
        try {
            int seq = NativeUdp.sendMove(FRAME_NED, x, y, z, yaw, maxV);
            m_tvStatus.setText(String.format(Locale.US,
                    "%s seq=%d x=%.2f y=%.2f z=%.2f yaw=%.2f maxV=%.2f",
                    reason, seq, x, y, z, yaw, maxV));
        } catch (Throwable t) {
            m_tvStatus.setText(reason + " error: " + t.getMessage());
        }
    }

    private void SendMoveVelocityCommand(float vx, float vy, float vz, float yawRate, float maxV, String reason) {
        try {
            int seq = NativeUdp.sendMoveVelocity(FRAME_NED, vx, vy, vz, yawRate, maxV);
            m_tvStatus.setText(String.format(Locale.US,
                    "%s seq=%d vx=%.2f vy=%.2f vz=%.2f yawRate=%.2f maxV=%.2f",
                    reason, seq, vx, vy, vz, yawRate, maxV));
        } catch (Throwable t) {
            m_tvStatus.setText(reason + " error: " + t.getMessage());
        }
    }

    private void SyncPositionInputsToCache() {
        m_lastCmdX = ParseF(m_etX, 0f);
        m_lastCmdY = ParseF(m_etY, 0f);
        m_lastCmdZ = ParseF(m_etZ, 1.2f);
        m_lastCmdYaw = ParseF(m_etYaw, 0f);
    }

    private static int ReadU16Le(byte[] data, int offset) {
        return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8);
    }

    private static long ReadU32Le(byte[] data, int offset) {
        return ((long) data[offset] & 0xFFL)
                | (((long) data[offset + 1] & 0xFFL) << 8)
                | (((long) data[offset + 2] & 0xFFL) << 16)
                | (((long) data[offset + 3] & 0xFFL) << 24);
    }

    private static int ReadI16Le(byte[] data, int offset) {
        int v = ReadU16Le(data, offset);
        return (v >= 0x8000) ? (v - 0x10000) : v;
    }

    private static int ReadI32Le(byte[] data, int offset) {
        return (data[offset] & 0xFF)
                | ((data[offset + 1] & 0xFF) << 8)
                | ((data[offset + 2] & 0xFF) << 16)
                | ((data[offset + 3] & 0xFF) << 24);
    }

    private static String AckStatusToText(int status) {
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

    private String DecodeTlvAck(byte[] rx) {
        if (rx == null) {
            return "RX: null";
        }
        if (rx.length < 17) {
            return "RX short: " + Hex(rx);
        }
        if ((rx[0] & 0xFF) != 0xAA || (rx[1] & 0xFF) != 0x55) {
            return "RX(no sync): " + Hex(rx);
        }

        int ver = rx[2] & 0xFF;
        int cmd = rx[3] & 0xFF;
        int flags = rx[4] & 0xFF;
        int len = ReadU16Le(rx, 5);
        long seq = ReadU32Le(rx, 7);
        long tMs = ReadU32Le(rx, 11);
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
        long ackSeq = ReadU32Le(rx, payloadOffset + 1);
        int status = ReadI16Le(rx, payloadOffset + 5);
        int reserved = ReadU16Le(rx, payloadOffset + 7);

        return String.format(Locale.US,
                "ACK ver=%d reqSeq=%d tMs=%d ackCmd=0x%02X ackSeq=%d status=%s reserved=%d",
                ver, seq, tMs, ackCmd, ackSeq, AckStatusToText(status), reserved);
    }

    private boolean TryHandleVideoPacket(byte[] rx) {
        if (rx == null || rx.length < VIDEO_HEADER_LEN) {
            return false;
        }
        if (ReadI32Le(rx, 0) != VIDEO_MAGIC) {
            return false;
        }
        int version = ReadU16Le(rx, 4);
        if (version != 1) {
            return false;
        }
        int camIndex = rx[6] & 0xFF;
        if (camIndex < 0 || camIndex >= m_videoAssemblies.length) {
            return true;
        }
        int frameId = ReadI32Le(rx, 20);
        int chunkIdx = ReadU16Le(rx, 24);
        int chunkCnt = ReadU16Le(rx, 26);
        int totalSize = ReadI32Le(rx, 28);
        int chunkSize = ReadI32Le(rx, 32);
        int payloadLen = rx.length - VIDEO_HEADER_LEN;

        if (chunkCnt <= 0 || chunkCnt > 256 || chunkIdx < 0 || chunkIdx >= chunkCnt) {
            return true;
        }
        if (totalSize <= 0 || totalSize > MAX_VIDEO_JPEG_BYTES) {
            return true;
        }
        if (chunkSize < 0 || chunkSize != payloadLen) {
            return true;
        }

        VideoAssembly assembly = m_videoAssemblies[camIndex];
        boolean needReset = (frameId != assembly.frameId)
                || (assembly.chunks == null)
                || (assembly.chunkCount != chunkCnt)
                || (assembly.totalSize != totalSize);
        if (needReset) {
            assembly.frameId = frameId;
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
        ImageView targetView = (camIndex == 0) ? m_ivVideoLeft : m_ivVideoRight;
        if (bitmap != null && targetView != null) {
            targetView.setImageBitmap(bitmap);
        }
        assembly.Reset();
        return true;
    }

    private boolean IsTlvPacket(byte[] rx) {
        return rx != null
                && rx.length >= 4
                && (rx[0] & 0xFF) == 0xAA
                && (rx[1] & 0xFF) == 0x55;
    }

    private void TickRxLoop() {
        for (int i = 0; i < MAX_RX_PACKETS_PER_TICK; ++i) {
            byte[] rx;
            try {
                rx = NativeUdp.pollRecv();
            } catch (Throwable t) {
                m_tvStatus.setText("rx error: " + t.getMessage());
                return;
            }
            if (rx == null) {
                return;
            }
            if (TryHandleVideoPacket(rx)) {
                continue;
            }
            if (IsTlvPacket(rx)) {
                m_lastAckPacket = rx;
            }
        }
    }

    private void TickJoystickControl() {
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

        float leftX = ApplyDeadzone(m_leftX);
        float leftY = ApplyDeadzone(m_leftY);
        float rightX = ApplyDeadzone(m_rightX);
        float rightY = ApplyDeadzone(m_rightY);

        float leftMag = Clamp01((float) Math.hypot(leftX, leftY));
        float rightMag = Clamp01((float) Math.hypot(rightX, rightY));
        float rightVerticalMag = Clamp01(Math.abs(rightY));

        boolean active = m_leftActive || m_rightActive
                || leftX != 0f || leftY != 0f || rightX != 0f || rightY != 0f;

        m_tvJoystickState.setText(String.format(Locale.US,
                "L[x=%.2f y=%.2f mag=%.2f] R[x=%.2f y=%.2f mag=%.2f] %s",
                leftX, leftY, leftMag, rightX, rightY, rightMag, active ? "ACTIVE" : "CENTER"));

        if (!active) {
            if (m_lastJoystickActive) {
                m_lastJoystickActive = false;
                SendHoldBurst(3, "HOLD(center)");
            }
            return;
        }
        m_lastJoystickActive = true;

        float baseMaxV = Math.max(0.1f, ParseF(m_etMaxV, 0.6f));
        float dynamicMaxV = baseMaxV * Clamp01(Math.max(leftMag, Math.max(rightVerticalMag, Math.abs(rightX))));
        if (dynamicMaxV < 0.05f) {
            dynamicMaxV = 0.05f;
        }

        // PX4-style local NED velocity setpoint semantics.
        float vx = leftY * XY_SPEED_SCALE_MPS * baseMaxV;
        float vy = leftX * XY_SPEED_SCALE_MPS * baseMaxV;
        float vz = (-rightY) * Z_SPEED_SCALE_MPS * baseMaxV;
        float yawRate = rightX * YAW_RATE_SCALE_RADPS;

        SendMoveVelocityCommand(vx, vy, vz, yawRate, dynamicMaxV, "JOY VEL");
    }

    private void StartJoystickLoop() {
        if (m_joystickLoopRunning) {
            return;
        }
        m_joystickLoopRunning = true;
        m_lastJoystickTickMs = 0L;
        m_handler.post(m_joystickLoop);
    }

    private void StopJoystickLoop() {
        m_joystickLoopRunning = false;
        m_handler.removeCallbacks(m_joystickLoop);
    }

    private void StartRxLoop() {
        if (m_rxLoopRunning) {
            return;
        }
        m_rxLoopRunning = true;
        m_handler.post(m_rxLoop);
    }

    private void StopRxLoop() {
        m_rxLoopRunning = false;
        m_handler.removeCallbacks(m_rxLoop);
    }

    private void ResetJoysticksAndHold() {
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

        SendHoldBurst(3, "HOLD(page)");
    }

    private void SetManualPage(boolean manualMode) {
        if (m_pageManual == null || m_pageCommand == null) {
            return;
        }
        m_isManualMode = manualMode;
        if (!manualMode) {
            ResetJoysticksAndHold();
        }
        m_pageManual.setVisibility(manualMode ? View.VISIBLE : View.GONE);
        m_pageCommand.setVisibility(manualMode ? View.GONE : View.VISIBLE);
        if (m_btnModeToggle != null) {
            m_btnModeToggle.setText(manualMode ? "MANUAL" : "COMMAND");
        }
        int targetOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE;
        if (getRequestedOrientation() != targetOrientation) {
            setRequestedOrientation(targetOrientation);
        }
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
        m_tvJoystickState = findViewById(R.id.tvJoystickState);
        m_pageManual = findViewById(R.id.pageManual);
        m_pageCommand = findViewById(R.id.pageCommand);
        m_btnModeToggle = findViewById(R.id.btnModeToggle);

        final String cm5Ip = "192.168.0.103";
        final int cm5Port = 14550;

        boolean ok;
        try {
            ok = NativeUdp.init(cm5Ip, cm5Port);
        } catch (Throwable t) {
            m_tvStatus.setText("Native init error: " + t.getMessage());
            ok = false;
        }
        m_tvStatus.setText(ok ? ("UDP ready -> " + cm5Ip + ":" + cm5Port) : "UDP init failed");

        Button btnArm = findViewById(R.id.btnArm);
        Button btnDisarm = findViewById(R.id.btnDisarm);
        Button btnOffboard = findViewById(R.id.btnOffboard);
        Button btnHold = findViewById(R.id.btnHold);
        Button btnLand = findViewById(R.id.btnLand);
        Button btnPing = findViewById(R.id.btnPing);
        Button btnMove = findViewById(R.id.btnMove);
        Button btnPoll = findViewById(R.id.btnPoll);

        m_etX = findViewById(R.id.etX);
        m_etY = findViewById(R.id.etY);
        m_etZ = findViewById(R.id.etZ);
        m_etYaw = findViewById(R.id.etYaw);
        m_etMaxV = findViewById(R.id.etMaxV);

        m_joystickLeft = findViewById(R.id.joystickLeft);
        m_joystickRight = findViewById(R.id.joystickRight);

        SyncPositionInputsToCache();

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

        m_btnModeToggle.setOnClickListener(v -> SetManualPage(!m_isManualMode));
        if (savedInstanceState != null) {
            m_isManualMode = savedInstanceState.getBoolean(KEY_MANUAL_MODE, true);
        }
        SetManualPage(m_isManualMode);

        btnArm.setOnClickListener(v -> SendSimpleCmd("ARM", CMD_ARM));
        btnDisarm.setOnClickListener(v -> SendSimpleCmd("DISARM", CMD_DISARM));
        btnOffboard.setOnClickListener(v -> SendSimpleCmd("OFFBOARD", CMD_OFFBOARD));
        btnHold.setOnClickListener(v -> SendSimpleCmd("HOLD", CMD_HOLD));
        btnLand.setOnClickListener(v -> SendSimpleCmd("LAND", CMD_LAND));
        btnPing.setOnClickListener(v -> SendSimpleCmd("PING", CMD_PING));

        btnMove.setOnClickListener(v -> {
            SyncPositionInputsToCache();
            float maxV = ParseF(m_etMaxV, 0.6f);
            SendMoveCommand(m_lastCmdX, m_lastCmdY, m_lastCmdZ, m_lastCmdYaw, maxV, "MOVE(pos) sent");
        });

        btnPoll.setOnClickListener(v -> {
            byte[] cached = m_lastAckPacket;
            if (cached == null) {
                m_tvStatus.setText("no cached ACK/TLV");
            } else {
                m_tvStatus.setText(DecodeTlvAck(cached));
            }
        });

        StartJoystickLoop();
        StartRxLoop();
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
        StopRxLoop();
        StopJoystickLoop();
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

