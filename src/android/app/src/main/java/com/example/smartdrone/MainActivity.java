package com.example.smartdrone;

import com.example.smartdrone.R;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.AutoCompleteTextView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MainActivity extends Activity {

    private static final int CMD_ARM = 0x10;
    private static final int CMD_DISARM = 0x11;
    private static final int CMD_OFFBOARD = 0x12;
    private static final int CMD_HOLD = 0x13;
    private static final int CMD_LAND = 0x14;
    private static final int CMD_EMERGENCY_STOP = 0x15;
    private static final int CMD_RUNTIME_MODE = 0x30;
    private static final int CMD_RUNTIME_CONFIG = 0x31;
    private static final int CMD_CALIB_CLEAN = 0x32;
    private static final int CMD_ACK = 0xF0;
    private static final int CMD_STATE = 0xF1;
    private static final int CMD_POINT_CLOUD = 0xF2;

    private static final int MODE_IDLE = 0;
    private static final int MODE_SLAM = 1;
    private static final int MODE_CALIB = 2;
    private static final int SENSOR_STEREO = 0;
    private static final int SENSOR_STEREO_IMU = 1;
    private static final long ACK_PENDING_TIMEOUT_MS = 3000L;
    private static final String PENDING_ARM = "arm";
    private static final String PENDING_EMERGENCY_STOP = "emergency_stop";
    private static final String PENDING_OFFBOARD = "offboard";
    private static final String PENDING_HOLD = "hold";
    private static final String PENDING_LAND = "land";
    private static final String PENDING_RUNTIME = "runtime";
    private static final String PENDING_SENSOR = "sensor";
    private static final String PENDING_CONFIG = "config";
    private static final String PENDING_CLEAN_CALIB = "clean_calib";
    private static final int EXPOSURE_MIN_US = 500;
    private static final int EXPOSURE_MAX_US = 20000;
    private static final int EXPOSURE_STEP_US = 500;
    private static final int GAIN_MIN = 1;
    private static final int GAIN_MAX = 32;

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
    private static final String KEY_SETTINGS_VISIBLE = "settingsVisible";
    private static final String KEY_DEBUG_VISIBLE = "debugVisible";
    private static final String KEY_REMOTE_VISIBLE = "remoteVisible";
    private static final String[] DEFAULT_VEHICLE_IPS = new String[]{
            "10.42.0.1",
            "192.168.0.105"
    };
    private ImageView m_ivVideoLeft;
    private ImageView m_ivVideoRight;
    private Map3DView m_map3dView;
    private TextView m_tvStatus;
    private TextView m_tvPose;
    private TextView m_tvVideoStats;
    private TextView m_tvJoystickState;
    private View m_debugPanel;
    private View m_remoteControlsBar;
    private View m_pageCommand;
    private View m_mapPanel;
    private ImageButton m_btnModeToggle;
    private Button m_btnArmToggle;
    private Button m_btnEmergencyStop;
    private Button m_btnOffboard;
    private Button m_btnHold;
    private Button m_btnLand;
    private Switch m_btnToggleSlam;
    private Switch m_btnToggleCalib;
    private Switch m_btnSensorMode;
    private Button m_btnCleanCalib;
    private Switch m_btnRemoteToggle;
    private Switch m_btnDebugToggle;
    private ImageButton m_btnMapClear;
    private ImageButton m_btnMapZoom;
    private Switch m_btnImageToggle;
    private Switch m_btnMapToggle;
    private Switch m_btnFeatureToggle;

    private AutoCompleteTextView m_etVehicleIp;
    private TextView m_tvCfgExposureValue;
    private TextView m_tvCfgGainValue;
    private SeekBar m_sbCfgExposure;
    private SeekBar m_sbCfgGain;

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
    private boolean m_settingsVisible = false;
    private boolean m_debugVisible = false;
    private boolean m_remoteVisible = false;
    private boolean m_updatingToggleUi = false;
    private boolean m_rxLoopRunning;
    private int m_runtimeMode = MODE_IDLE;
    private String m_vehicleIp = "10.42.0.1";
    private boolean m_armLatched = false;
    private String m_lastFlightCommand = "";
    private int m_sensorMode = SENSOR_STEREO;
    private int m_cfgExposureUs = 3000;
    private int m_cfgGain = 2;
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
    private int m_featurePktCount0 = 0;
    private int m_featurePktCount1 = 0;
    private int m_featureMatchCount0 = 0;
    private int m_featureMatchCount1 = 0;
    private boolean m_sendImage = true;
    private boolean m_sendFeature = true;
    private boolean m_sendMap = true;
    private boolean m_showFeaturePoints = true;

    private final Paint m_featurePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Map<Long, PendingAckAction> m_pendingAckActions = new HashMap<>();
    private final Set<String> m_pendingUiKeys = new HashSet<>();
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

    private static int clampInt(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private static int quantizeExposureUs(int exposureUs) {
        int clamped = clampInt(exposureUs, EXPOSURE_MIN_US, EXPOSURE_MAX_US);
        int steps = Math.round((clamped - EXPOSURE_MIN_US) / (float) EXPOSURE_STEP_US);
        return EXPOSURE_MIN_US + steps * EXPOSURE_STEP_US;
    }

    private static int quantizeGain(int gain) {
        return clampInt(gain, GAIN_MIN, GAIN_MAX);
    }

    private static Float findPoseField(String text, String... keys) {
        if (text == null) {
            return null;
        }
        for (String key : keys) {
            Pattern pattern = Pattern.compile("(?i)\\b" + Pattern.quote(key)
                    + "\\s*[:=]\\s*(-?\\d+(?:\\.\\d+)?)");
            Matcher matcher = pattern.matcher(text);
            if (matcher.find()) {
                try {
                    return Float.parseFloat(matcher.group(1));
                } catch (Throwable ignored) {
                }
            }
        }
        return null;
    }

    private static int readLeU16(byte[] data, int offset) {
        return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8);
    }

    private static int readFramePayloadLen(byte[] data) {
        if (data == null || data.length < 17) {
            return -1;
        }
        return readLeU16(data, 5);
    }

    private static float readLeF32(byte[] data, int offset) {
        int bits = (data[offset] & 0xFF)
                | ((data[offset + 1] & 0xFF) << 8)
                | ((data[offset + 2] & 0xFF) << 16)
                | ((data[offset + 3] & 0xFF) << 24);
        return Float.intBitsToFloat(bits);
    }

    private static float clampUnit(float value) {
        return Math.max(-1.0f, Math.min(1.0f, value));
    }

    private static float quatYawDeg(float qw, float qx, float qy, float qz) {
        double siny = 2.0 * (qw * qz + qx * qy);
        double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
        return (float) Math.toDegrees(Math.atan2(siny, cosy));
    }

    private static float quatPitchDeg(float qw, float qx, float qy, float qz) {
        double sinp = 2.0 * (qw * qy - qz * qx);
        return (float) Math.toDegrees(Math.asin(clampUnit((float) sinp)));
    }

    private static float quatRollDeg(float qw, float qx, float qy, float qz) {
        double sinr = 2.0 * (qw * qx + qy * qz);
        double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
        return (float) Math.toDegrees(Math.atan2(sinr, cosr));
    }

    private boolean tryHandleStatePoseForMap(byte[] rx) {
        if (!m_sendMap) {
            return false;
        }
        if (rx == null || rx.length < 43) {
            return false;
        }
        if ((rx[0] & 0xFF) != 0xAA || (rx[1] & 0xFF) != 0x55 || (rx[3] & 0xFF) != CMD_STATE) {
            return false;
        }
        final int payloadLen = readFramePayloadLen(rx);
        if (payloadLen < 34 || rx.length < 17 + payloadLen) {
            return false;
        }
        final int payloadOffset = 15;
        final float x = readLeF32(rx, payloadOffset + 6);
        final float y = readLeF32(rx, payloadOffset + 10);
        final float z = readLeF32(rx, payloadOffset + 14);
        final float qw = readLeF32(rx, payloadOffset + 18);
        final float qx = readLeF32(rx, payloadOffset + 22);
        final float qy = readLeF32(rx, payloadOffset + 26);
        final float qz = readLeF32(rx, payloadOffset + 30);
        float mapX = x;
        float mapY = y;
        float mapZ = z;
        float rollDeg = quatRollDeg(qw, qx, qy, qz);
        float pitchDeg = quatPitchDeg(qw, qx, qy, qz);
        float yawDeg = quatYawDeg(qw, qx, qy, qz);
        if (m_map3dView != null) {
            m_map3dView.setPose(
                    mapX,
                    mapY,
                    mapZ,
                    rollDeg,
                    pitchDeg,
                    yawDeg,
                    true);
        }
        return true;
    }

    private boolean tryHandlePointCloudPacket(byte[] rx) {
        if (!m_sendMap) {
            return false;
        }
        if (rx == null || rx.length < 21) {
            return false;
        }
        if ((rx[0] & 0xFF) != 0xAA || (rx[1] & 0xFF) != 0x55 || (rx[3] & 0xFF) != CMD_POINT_CLOUD) {
            return false;
        }
        final int payloadLen = readFramePayloadLen(rx);
        if (payloadLen < 4 || rx.length < 17 + payloadLen) {
            return false;
        }
        final int payloadOffset = 15;
        final int pointCount = readLeU16(rx, payloadOffset);
        final int actualPointCount = Math.min(pointCount, (payloadLen - 4) / 12);
        if (actualPointCount <= 0) {
            return true;
        }
        float[] xyz = new float[actualPointCount * 3];
        int src = payloadOffset + 4;
        for (int i = 0; i < actualPointCount; ++i) {
            int dst = i * 3;
            xyz[dst] = readLeF32(rx, src);
            xyz[dst + 1] = readLeF32(rx, src + 4);
            xyz[dst + 2] = readLeF32(rx, src + 8);
            src += 12;
        }
        if (m_map3dView != null) {
            m_map3dView.setPointCloud(xyz, actualPointCount);
        }
        return true;
    }

    private void updatePoseMapFromText() {
        if (m_map3dView == null || m_tvPose == null) {
            return;
        }
        String poseText = m_tvPose.getText() != null ? m_tvPose.getText().toString() : "";
        Float x = findPoseField(poseText, "x", "px", "tx", "posx");
        Float y = findPoseField(poseText, "y", "py", "ty", "posy");
        Float z = findPoseField(poseText, "z", "pz", "tz", "posz");
        Float roll = findPoseField(poseText, "roll", "r");
        Float pitch = findPoseField(poseText, "pitch", "p");
        Float yaw = findPoseField(poseText, "yaw");
        if (yaw == null) {
            yaw = findPoseField(poseText, "heading");
        }
        if (x == null || y == null || z == null) {
            return;
        }
        m_map3dView.setPose(
                x,
                y,
                z,
                roll != null ? roll : 0f,
                pitch != null ? pitch : 0f,
                yaw != null ? yaw : 0f,
                true);
    }

    private void updateConfigViews() {
        if (m_tvCfgExposureValue != null) {
            m_tvCfgExposureValue.setText(String.format(Locale.US, "Exposure: %d us", m_cfgExposureUs));
        }
        if (m_tvCfgGainValue != null) {
            m_tvCfgGainValue.setText(String.format(Locale.US, "Gain: %d", m_cfgGain));
        }
        if (m_sbCfgExposure != null) {
            int progress = (m_cfgExposureUs - EXPOSURE_MIN_US) / EXPOSURE_STEP_US;
            if (m_sbCfgExposure.getProgress() != progress) {
                m_sbCfgExposure.setProgress(progress);
            }
        }
        if (m_sbCfgGain != null) {
            int progress = m_cfgGain - GAIN_MIN;
            if (m_sbCfgGain.getProgress() != progress) {
                m_sbCfgGain.setProgress(progress);
            }
        }
    }

    private void updateMapButtons() {
        if (m_btnMapZoom != null && m_map3dView != null) {
            m_btnMapZoom.setImageResource(
                    m_map3dView.isZoomedIn()
                            ? android.R.drawable.ic_menu_zoom
                            : android.R.drawable.ic_menu_search);
        }
    }

    private void updateStreamToggleButtons() {
        m_updatingToggleUi = true;
        if (m_btnImageToggle != null) {
            m_btnImageToggle.setChecked(m_sendImage);
            m_btnImageToggle.setAlpha(m_sendImage ? 1.0f : 0.65f);
        }
        if (m_btnMapToggle != null) {
            m_btnMapToggle.setChecked(m_sendMap);
            m_btnMapToggle.setAlpha(m_sendMap ? 1.0f : 0.65f);
        }
        if (m_btnFeatureToggle != null) {
            m_btnFeatureToggle.setChecked(m_sendFeature);
            m_btnFeatureToggle.setAlpha(m_sendFeature ? 1.0f : 0.65f);
        }
        if (m_mapPanel != null) {
            m_mapPanel.setVisibility(m_sendMap ? View.VISIBLE : View.GONE);
        }
        m_updatingToggleUi = false;
    }

    private void updateFeatureToggleButton() {
        m_showFeaturePoints = m_sendFeature;
        updateStreamToggleButtons();
    }

    private void updateDebugPanelVisibility() {
        if (m_debugPanel != null) {
            m_debugPanel.setVisibility(m_debugVisible && !m_settingsVisible ? View.VISIBLE : View.GONE);
        }
        if (m_btnDebugToggle != null) {
            m_updatingToggleUi = true;
            m_btnDebugToggle.setChecked(m_debugVisible);
            m_btnDebugToggle.setAlpha(m_debugVisible ? 1.0f : 0.65f);
            m_updatingToggleUi = false;
        }
    }

    private void updateRemoteControlsVisibility() {
        if (m_remoteControlsBar != null) {
            m_remoteControlsBar.setVisibility(m_remoteVisible ? View.VISIBLE : View.GONE);
        }
        if (m_joystickLeft != null) {
            m_joystickLeft.setVisibility(m_remoteVisible ? View.VISIBLE : View.GONE);
        }
        if (m_joystickRight != null) {
            m_joystickRight.setVisibility(m_remoteVisible ? View.VISIBLE : View.GONE);
        }
        if (m_btnRemoteToggle != null) {
            m_updatingToggleUi = true;
            m_btnRemoteToggle.setChecked(m_remoteVisible);
            m_btnRemoteToggle.setAlpha(m_remoteVisible ? 1.0f : 0.65f);
            m_updatingToggleUi = false;
        }
    }

    private void resetRemoteInputs() {
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
    }

    private void refreshVideoFrames() {
        renderVideoFrame(0);
        renderVideoFrame(1);
    }

    private void sendCurrentRuntimeConfig(String label, String pendingKey, AckSuccess onSuccess) {
        sendRuntimeConfigAwaitAck(
                m_cfgExposureUs,
                (float) m_cfgGain,
                m_sensorMode,
                m_sendImage,
                m_sendFeature,
                m_sendMap,
                label,
                pendingKey,
                onSuccess);
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

    private void sendSimpleCmdAwaitAck(String name, int cmd, String pendingKey, AckSuccess onSuccess) {
        if (!ensureVehicleConnection()) {
            return;
        }
        if (isPending(pendingKey)) {
            return;
        }
        try {
            int seq = NativeUdp.sendCmd(cmd);
            if (seq < 0) {
                m_tvStatus.setText(name + " send failed");
                return;
            }
            registerPendingAck(seq, cmd, name, pendingKey, onSuccess);
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

    private void sendMoveRcJoystickCommand(
            float throttle,
            float yaw,
            float pitch,
            float roll,
            float maxV,
            String reason) {
        try {
            int seq = NativeUdp.sendMoveRcJoystick(FRAME_NED, throttle, yaw, pitch, roll, maxV);
            m_tvStatus.setText(String.format(Locale.US,
                    "%s seq=%d OFFBOARD thr=%.2f yaw=%.2f pitch=%.2f roll=%.2f maxV=%.2f",
                    reason, seq, throttle, yaw, pitch, roll, maxV));
        } catch (Throwable t) {
            m_tvStatus.setText(reason + " error: " + t.getMessage());
        }
    }

    private int sendRuntimeConfig(
            int exposureUs,
            float gain,
            int sensorMode,
            boolean sendImage,
            boolean sendFeature,
            boolean sendMap) {
        try {
            int seq = NativeUdp.sendRuntimeConfig(exposureUs, gain, sensorMode, sendImage, sendFeature, sendMap);
            m_tvStatus.setText(String.format(Locale.US,
                    "CFG seq=%d exp=%d gain=%.1f sensor=%s img=%s feat=%s map=%s",
                    seq, exposureUs, gain, sensorModeToText(sensorMode),
                    sendImage ? "on" : "off",
                    sendFeature ? "on" : "off",
                    sendMap ? "on" : "off"));
            return seq;
        } catch (Throwable t) {
            m_tvStatus.setText("CFG error: " + t.getMessage());
            return -1;
        }
    }

    private int sendRuntimeConfig() {
        return sendRuntimeConfig(
                m_cfgExposureUs,
                (float) m_cfgGain,
                m_sensorMode,
                m_sendImage,
                m_sendFeature,
                m_sendMap);
    }

    private void sendRuntimeConfigAwaitAck(
            int exposureUs,
            float gain,
            int sensorMode,
            boolean sendImage,
            boolean sendFeature,
            boolean sendMap,
            String label,
            String pendingKey,
            AckSuccess onSuccess) {
        if (!ensureVehicleConnection()) {
            return;
        }
        if (isPending(pendingKey)) {
            return;
        }
        int seq = sendRuntimeConfig(exposureUs, gain, sensorMode, sendImage, sendFeature, sendMap);
        if (seq < 0) {
            return;
        }
        registerPendingAck(seq, CMD_RUNTIME_CONFIG, label, pendingKey, onSuccess);
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

    private void setButtonState(Button button, boolean active, boolean pending, String color) {
        if (button == null) {
            return;
        }
        button.setBackgroundColor(Color.parseColor(color));
        button.setAlpha(pending ? 0.2f : (active ? 1.0f : 0.35f));
        button.setTextColor(Color.WHITE);
        button.setEnabled(!pending);
    }

    private void updateRuntimeButtons() {
        boolean sensorPending = isPending(PENDING_SENSOR) || isPending(PENDING_RUNTIME);
        boolean runtimePending = isPending(PENDING_RUNTIME);
        m_updatingToggleUi = true;
        if (m_btnSensorMode != null) {
            m_btnSensorMode.setChecked(m_sensorMode == SENSOR_STEREO_IMU);
            m_btnSensorMode.setEnabled(!sensorPending);
            m_btnSensorMode.setAlpha(sensorPending ? 0.35f : 1.0f);
        }
        if (m_btnToggleSlam != null) {
            m_btnToggleSlam.setChecked(m_runtimeMode == MODE_SLAM);
            m_btnToggleSlam.setEnabled(!runtimePending);
            m_btnToggleSlam.setAlpha(runtimePending ? 0.35f : 1.0f);
        }
        if (m_btnToggleCalib != null) {
            m_btnToggleCalib.setChecked(m_runtimeMode == MODE_CALIB);
            m_btnToggleCalib.setEnabled(!runtimePending);
            m_btnToggleCalib.setAlpha(runtimePending ? 0.35f : 1.0f);
        }
        m_updatingToggleUi = false;
        if (m_btnCleanCalib != null) {
            setButtonState(m_btnCleanCalib, true, isPending(PENDING_CLEAN_CALIB), "#546E7A");
        }
    }

    private void updateFlightButtons() {
        if (m_btnArmToggle != null) {
            m_btnArmToggle.setText(m_armLatched ? "DISARM" : "ARM");
            setButtonState(m_btnArmToggle, true, isPending(PENDING_ARM), "#C62828");
        }
        if (m_btnEmergencyStop != null) {
            setButtonState(m_btnEmergencyStop, "EMERGENCY_STOP".equals(m_lastFlightCommand), isPending(PENDING_EMERGENCY_STOP), "#B71C1C");
        }
        if (m_btnOffboard != null) {
            setButtonState(m_btnOffboard, "OFFBOARD".equals(m_lastFlightCommand), isPending(PENDING_OFFBOARD), "#6A1B9A");
        }
        if (m_btnHold != null) {
            setButtonState(m_btnHold, "HOLD".equals(m_lastFlightCommand), isPending(PENDING_HOLD), "#00897B");
        }
        if (m_btnLand != null) {
            setButtonState(m_btnLand, "LAND".equals(m_lastFlightCommand), isPending(PENDING_LAND), "#EF6C00");
        }
    }

    private void sendRuntimeMode(int mode, String label) {
        if (!ensureVehicleConnection()) {
            return;
        }
        if (isPending(PENDING_RUNTIME)) {
            return;
        }
        final int exposureUs = m_cfgExposureUs;
        final float gain = (float) m_cfgGain;
        final int sensorMode = m_sensorMode;
        sendRuntimeConfigAwaitAck(
                exposureUs,
                gain,
                sensorMode,
                m_sendImage,
                m_sendFeature,
                m_sendMap,
                label + " CFG",
                PENDING_RUNTIME,
                () -> {
            try {
                int seq = NativeUdp.sendRuntimeMode(mode);
                if (seq < 0) {
                    clearPendingKey(PENDING_RUNTIME);
                    m_tvStatus.setText(label + " send failed");
                    return;
                }
                registerPendingAck(seq, CMD_RUNTIME_MODE, label, PENDING_RUNTIME, () -> {
                    m_runtimeMode = mode;
                    updateRuntimeButtons();
                });
                m_tvStatus.setText(label + " sent seq=" + seq);
            } catch (Throwable t) {
                clearPendingKey(PENDING_RUNTIME);
                m_tvStatus.setText(label + " error: " + t.getMessage());
            }
        });
    }

    private void stopRuntime(String label) {
        if (!ensureVehicleConnection()) {
            return;
        }
        if (isPending(PENDING_RUNTIME)) {
            return;
        }
        try {
            int seq = NativeUdp.sendRuntimeMode(MODE_IDLE);
            if (seq < 0) {
                m_tvStatus.setText(label + " send failed");
                return;
            }
            registerPendingAck(seq, CMD_RUNTIME_MODE, label, PENDING_RUNTIME, () -> {
                m_runtimeMode = MODE_IDLE;
                updateRuntimeButtons();
            });
            m_tvStatus.setText(label + " sent seq=" + seq);
        } catch (Throwable t) {
            m_tvStatus.setText(label + " error: " + t.getMessage());
        }
    }

    private void registerPendingAck(long seq, int command, String label, String pendingKey, AckSuccess onSuccess) {
        if (pendingKey != null && !pendingKey.isEmpty()) {
            m_pendingUiKeys.add(pendingKey);
            updateRuntimeButtons();
            updateFlightButtons();
        }
        Runnable timeoutRunnable = () -> {
            PendingAckAction removed = m_pendingAckActions.remove(seq);
            if (removed == null) {
                return;
            }
            clearPendingKey(removed.pendingKey);
            m_tvStatus.setText(removed.label + " timeout");
        };
        m_pendingAckActions.put(seq, new PendingAckAction(command, label, pendingKey, onSuccess, timeoutRunnable));
        m_handler.postDelayed(timeoutRunnable, ACK_PENDING_TIMEOUT_MS);
    }

    private boolean isPending(String pendingKey) {
        return pendingKey != null && m_pendingUiKeys.contains(pendingKey);
    }

    private void clearPendingKey(String pendingKey) {
        if (pendingKey == null || pendingKey.isEmpty()) {
            return;
        }
        m_pendingUiKeys.remove(pendingKey);
        updateRuntimeButtons();
        updateFlightButtons();
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
        AckFrame ack = parseAckFrame(rx);
        if (ack.valid) {
            return String.format(Locale.US,
                    "ACK reqSeq=%d ackCmd=0x%02X ackSeq=%d status=%s",
                    ack.reqSeq, ack.ackCmd, ack.ackSeq, ackStatusToText(ack.status));
        }
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
        return String.format(Locale.US,
                "RX TLV ver=%d cmd=0x%02X flags=%d len=%d seq=%d tMs=%d",
                ver, cmd, flags, len, seq, tMs);
    }

    private AckFrame parseAckFrame(byte[] rx) {
        AckFrame out = new AckFrame();
        if (rx == null) {
            return out;
        }
        if (rx.length < 17) {
            return out;
        }
        if ((rx[0] & 0xFF) != 0xAA || (rx[1] & 0xFF) != 0x55) {
            return out;
        }
        int cmd = rx[3] & 0xFF;
        int len = readU16Le(rx, 5);
        int total = 2 + (1 + 1 + 1 + 2 + 4 + 4) + len + 2;
        if (rx.length < total) {
            return out;
        }
        if (cmd != CMD_ACK || len < 9) {
            return out;
        }
        int payloadOffset = 15;
        out.valid = true;
        out.reqSeq = readU32Le(rx, 7);
        out.ackCmd = rx[payloadOffset] & 0xFF;
        out.ackSeq = readU32Le(rx, payloadOffset + 1);
        out.status = readI16Le(rx, payloadOffset + 5);
        return out;
    }

    private boolean tryHandleAckPacket(byte[] rx) {
        AckFrame ack = parseAckFrame(rx);
        if (!ack.valid) {
            return false;
        }
        PendingAckAction pending = m_pendingAckActions.remove(ack.ackSeq);
        if (pending != null) {
            m_handler.removeCallbacks(pending.onTimeout);
            clearPendingKey(pending.pendingKey);
        }
        if (pending != null && pending.command == ack.ackCmd && ack.status == 0) {
            pending.onSuccess.run();
        }
        if (pending != null) {
            m_tvStatus.setText(String.format(Locale.US,
                    "%s ack=%s seq=%d",
                    pending.label, ackStatusToText(ack.status), ack.ackSeq));
        } else {
            m_tvStatus.setText(decodeTlvAck(rx));
        }
        return true;
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

    private String sensorModeToText(int sensorMode) {
        return sensorMode == SENSOR_STEREO_IMU ? "Stereo-IMU" : "Stereo";
    }

    private interface AckSuccess {
        void run();
    }

    private static final class PendingAckAction {
        final int command;
        final String label;
        final String pendingKey;
        final AckSuccess onSuccess;
        final Runnable onTimeout;

        PendingAckAction(int command, String label, String pendingKey, AckSuccess onSuccess, Runnable onTimeout) {
            this.command = command;
            this.label = label;
            this.pendingKey = pendingKey;
            this.onSuccess = onSuccess;
            this.onTimeout = onTimeout;
        }
    }

    private static final class AckFrame {
        boolean valid;
        long reqSeq;
        int ackCmd;
        long ackSeq;
        int status;
    }

    private String trackingStateToText(int trackingState) {
        switch (trackingState) {
            case 0:
                return "NO_IMAGES_YET";
            case 1:
                return "NOT_INITIALIZED";
            case 2:
                return "OK";
            case 3:
                return "RECENTLY_LOST";
            case 4:
                return "LOST";
            case 5:
                return "OK_KLT";
            case 0xFF:
                return "INVALID";
            default:
                return "STATE(" + trackingState + ")";
        }
    }

    private boolean tryHandleStatePacket(byte[] rx) {
        if (!isTlvPacket(rx) || rx.length < 15) {
            return false;
        }
        int cmd = rx[3] & 0xFF;
        int len = readU16Le(rx, 5);
        if (cmd != CMD_STATE || (len != 32 && len != 34) || rx.length < 15 + len + 2) {
            return false;
        }
        int payloadOffset = 15;
        int runtimeMode = rx[payloadOffset] & 0xFF;
        int trackingState = rx[payloadOffset + 1] & 0xFF;
        int resetCounter = len >= 34 ? readU16Le(rx, payloadOffset + 2) : 0;
        int resetMapCount = len >= 34 ? readU16Le(rx, payloadOffset + 4) : 0;
        int poseOffset = len >= 34 ? payloadOffset + 6 : payloadOffset + 4;
        float x = readF32Le(rx, poseOffset);
        float y = readF32Le(rx, poseOffset + 4);
        float z = readF32Le(rx, poseOffset + 8);
        float qw = readF32Le(rx, poseOffset + 12);
        float qx = readF32Le(rx, poseOffset + 16);
        float qy = readF32Le(rx, poseOffset + 20);
        float qz = readF32Le(rx, poseOffset + 24);
        if (m_tvPose != null) {
            if (runtimeMode == MODE_SLAM) {
                boolean hasValidPose = trackingState == 2 || trackingState == 5;
                if (!hasValidPose) {
                    x = Float.NaN;
                    y = Float.NaN;
                    z = Float.NaN;
                    qw = Float.NaN;
                    qx = Float.NaN;
                    qy = Float.NaN;
                    qz = Float.NaN;
                }
                m_tvPose.setText(String.format(Locale.US,
                        "Pose %s trk=%s rst=%d map_rst=%d\np[%.2f %.2f %.2f]\nq[%.2f %.2f %.2f %.2f]",
                        runtimeModeToText(runtimeMode), trackingStateToText(trackingState),
                        resetCounter, resetMapCount, x, y, z, qw, qx, qy, qz));
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
            if (!m_sendFeature) {
                return true;
            }
            return tryHandleFeaturePacket(rx, camIndex);
        }
        if (!m_sendImage) {
            return true;
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
        if (camIndex == 0) {
            m_featurePktCount0++;
        } else {
            m_featurePktCount1++;
        }
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
        if (m_sendFeature && m_showFeaturePoints
                && featureFrame.xs != null
                && Math.abs(featureFrame.frameTimeSec - displayFrame.frameTimeSec) <= FRAME_MATCH_TOLERANCE_SEC) {
            output = overlayFeaturePoints(displayFrame.bitmap, featureFrame);
            if (displayFrame.overlayFrameId != featureFrame.frameId) {
                displayFrame.overlayFrameId = featureFrame.frameId;
                m_featureMatchCount++;
                if (camIndex == 0) {
                    m_featureMatchCount0++;
                } else {
                    m_featureMatchCount1++;
                }
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
            if (tryHandlePointCloudPacket(rx)) {
                continue;
            }
            tryHandleStatePoseForMap(rx);
            if (tryHandleStatePacket(rx)) {
                continue;
            }
            if (tryHandleAckPacket(rx)) {
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
                "Video pkt=%d feat=%d(L%d/R%d) fuse=%d(L%d/R%d) ok=%d fail=%d bad=%d L=%d R=%d last=%s",
                m_videoPktCount, m_featurePktCount, m_featurePktCount0, m_featurePktCount1,
                m_featureMatchCount, m_featureMatchCount0, m_featureMatchCount1,
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
        boolean active = m_leftActive || m_rightActive
                || leftX != 0f || leftY != 0f || rightX != 0f || rightY != 0f;

        float horizontalVx = rightY * 5.0f;
        float horizontalVy = rightX * 5.0f;
        float verticalVz = -leftY * 3.0f;

        m_tvJoystickState.setText(String.format(Locale.US,
                "OFFBOARD L[yaw=%.2f vz=%.2f] R[vy=%.2f vx=%.2f] magL=%.2f magR=%.2f %s",
                leftX, verticalVz, horizontalVy, horizontalVx, leftMag, rightMag, active ? "ACTIVE" : "CENTER"));

        if (!active) {
            if (m_lastJoystickActive) {
                m_lastJoystickActive = false;
                sendHoldBurst(3, "HOLD(center)");
            }
            return;
        }
        m_lastJoystickActive = true;

        float throttle = leftY;
        float yaw = leftX;
        float pitch = rightY;
        float roll = rightX;
        sendMoveRcJoystickCommand(throttle, yaw, pitch, roll, 5.0f, "JOY RC");
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

    private void setSettingsVisible(boolean visible) {
        if (m_pageCommand == null) {
            return;
        }
        m_settingsVisible = visible;
        m_pageCommand.setVisibility(visible ? View.VISIBLE : View.GONE);
        updateDebugPanelVisibility();
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
        m_map3dView = findViewById(R.id.map3dView);
        m_tvStatus = findViewById(R.id.tvStatus);
        m_tvPose = findViewById(R.id.tvPose);
        m_tvVideoStats = findViewById(R.id.tvVideoStats);
        m_tvJoystickState = findViewById(R.id.tvJoystickState);
        m_debugPanel = findViewById(R.id.debugPanel);
        m_remoteControlsBar = findViewById(R.id.remoteControlsBar);
        m_pageCommand = findViewById(R.id.pageCommand);
        m_mapPanel = findViewById(R.id.mapPanel);
        m_btnModeToggle = findViewById(R.id.btnModeToggle);
        m_btnArmToggle = findViewById(R.id.btnArm);
        m_btnEmergencyStop = findViewById(R.id.btnEmergencyStop);
        m_btnOffboard = findViewById(R.id.btnOffboard);
        m_btnHold = findViewById(R.id.btnHold);
        m_btnLand = findViewById(R.id.btnLand);
        m_btnToggleSlam = findViewById(R.id.btnToggleSlam);
        m_btnToggleCalib = findViewById(R.id.btnToggleCalib);
        m_btnSensorMode = findViewById(R.id.btnSensorMode);
        m_btnRemoteToggle = findViewById(R.id.btnRemoteToggle);
        m_btnDebugToggle = findViewById(R.id.btnDebugToggle);
        m_btnMapClear = findViewById(R.id.btnMapClear);
        m_btnMapZoom = findViewById(R.id.btnMapZoom);
        m_btnImageToggle = findViewById(R.id.btnImageToggle);
        m_btnMapToggle = findViewById(R.id.btnMapToggle);
        m_btnFeatureToggle = findViewById(R.id.btnFeatureToggle);
        m_featurePaint.setColor(Color.GREEN);
        m_featurePaint.setStyle(Paint.Style.STROKE);
        m_featurePaint.setStrokeWidth(2.0f);

        m_btnCleanCalib = findViewById(R.id.btnCleanCalib);

        m_etVehicleIp = findViewById(R.id.etVehicleIp);
        if (m_etVehicleIp != null) {
            ArrayAdapter<String> vehicleIpAdapter = new ArrayAdapter<>(
                    this,
                    R.layout.dropdown_vehicle_ip_item,
                    DEFAULT_VEHICLE_IPS);
            m_etVehicleIp.setAdapter(vehicleIpAdapter);
            m_etVehicleIp.setOnClickListener(v -> m_etVehicleIp.showDropDown());
            m_etVehicleIp.setOnFocusChangeListener((v, hasFocus) -> {
                if (hasFocus) {
                    m_etVehicleIp.showDropDown();
                }
            });
        }
        m_tvCfgExposureValue = findViewById(R.id.tvCfgExposureValue);
        m_tvCfgGainValue = findViewById(R.id.tvCfgGainValue);
        m_sbCfgExposure = findViewById(R.id.sbCfgExposure);
        m_sbCfgGain = findViewById(R.id.sbCfgGain);

        m_joystickLeft = findViewById(R.id.joystickLeft);
        m_joystickRight = findViewById(R.id.joystickRight);

        final String cm5Ip = "10.42.0.1";
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

        if (m_sbCfgExposure != null) {
            m_sbCfgExposure.setMax((EXPOSURE_MAX_US - EXPOSURE_MIN_US) / EXPOSURE_STEP_US);
            m_sbCfgExposure.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    m_cfgExposureUs = EXPOSURE_MIN_US + progress * EXPOSURE_STEP_US;
                    updateConfigViews();
                }

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {
                }

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {
                    sendCurrentRuntimeConfig("Exposure", PENDING_CONFIG, () -> { });
                }
            });
        }
        if (m_sbCfgGain != null) {
            m_sbCfgGain.setMax(GAIN_MAX - GAIN_MIN);
            m_sbCfgGain.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    m_cfgGain = GAIN_MIN + progress;
                    updateConfigViews();
                }

                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {
                }

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {
                    sendCurrentRuntimeConfig("Gain", PENDING_CONFIG, () -> { });
                }
            });
        }
        m_cfgExposureUs = quantizeExposureUs(m_cfgExposureUs);
        m_cfgGain = quantizeGain(m_cfgGain);
        updateConfigViews();
        updatePoseMapFromText();
        updateMapButtons();
        updateStreamToggleButtons();
        updateFeatureToggleButton();
        updateDebugPanelVisibility();
        updateRemoteControlsVisibility();

        if (m_btnMapClear != null) {
            m_btnMapClear.setOnClickListener(v -> {
                if (m_map3dView != null) {
                    m_map3dView.clearPointCloud();
                }
            });
        }
        if (m_btnMapZoom != null) {
            m_btnMapZoom.setOnClickListener(v -> {
                if (m_map3dView != null) {
                    m_map3dView.toggleZoom();
                    updateMapButtons();
                }
            });
        }
        if (m_btnImageToggle != null) {
            m_btnImageToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final boolean nextValue = isChecked;
                sendRuntimeConfigAwaitAck(
                        m_cfgExposureUs,
                        (float) m_cfgGain,
                        m_sensorMode,
                        nextValue,
                        m_sendFeature,
                        m_sendMap,
                        "Image stream",
                        PENDING_CONFIG,
                        () -> {
                            m_sendImage = nextValue;
                            if (!m_sendImage) {
                                if (m_ivVideoLeft != null) m_ivVideoLeft.setImageDrawable(null);
                                if (m_ivVideoRight != null) m_ivVideoRight.setImageDrawable(null);
                            }
                            updateStreamToggleButtons();
                        });
            });
        }
        if (m_btnMapToggle != null) {
            m_btnMapToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final boolean nextValue = isChecked;
                sendRuntimeConfigAwaitAck(
                        m_cfgExposureUs,
                        (float) m_cfgGain,
                        m_sensorMode,
                        m_sendImage,
                        m_sendFeature,
                        nextValue,
                        "Map stream",
                        PENDING_CONFIG,
                        () -> {
                            m_sendMap = nextValue;
                            updateStreamToggleButtons();
                        });
            });
        }
        if (m_btnFeatureToggle != null) {
            m_btnFeatureToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final boolean nextValue = isChecked;
                sendRuntimeConfigAwaitAck(
                        m_cfgExposureUs,
                        (float) m_cfgGain,
                        m_sensorMode,
                        m_sendImage,
                        nextValue,
                        m_sendMap,
                        "Feature stream",
                        PENDING_CONFIG,
                        () -> {
                            m_sendFeature = nextValue;
                            m_showFeaturePoints = nextValue;
                            refreshVideoFrames();
                            updateStreamToggleButtons();
                        });
            });
        }
        if (m_btnDebugToggle != null) {
            m_btnDebugToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                m_debugVisible = isChecked;
                updateDebugPanelVisibility();
            });
        }
        if (m_btnRemoteToggle != null) {
            m_btnRemoteToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                m_remoteVisible = isChecked;
                if (!m_remoteVisible) {
                    resetRemoteInputs();
                    sendHoldBurst(3, "HOLD(remote off)");
                }
                updateRemoteControlsVisibility();
            });
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

        m_btnModeToggle.setOnClickListener(v -> setSettingsVisible(!m_settingsVisible));
        if (savedInstanceState != null) {
            m_settingsVisible = savedInstanceState.getBoolean(KEY_SETTINGS_VISIBLE, false);
            m_debugVisible = savedInstanceState.getBoolean(KEY_DEBUG_VISIBLE, false);
            m_remoteVisible = savedInstanceState.getBoolean(KEY_REMOTE_VISIBLE, false);
        }
        setSettingsVisible(m_settingsVisible);
        updateDebugPanelVisibility();
        updateRemoteControlsVisibility();

        if (m_btnArmToggle != null) {
            m_btnArmToggle.setOnClickListener(v -> {
                final boolean nextArm = !m_armLatched;
                sendSimpleCmdAwaitAck(nextArm ? "ARM" : "DISARM", nextArm ? CMD_ARM : CMD_DISARM, PENDING_ARM, () -> {
                    m_armLatched = nextArm;
                    m_lastFlightCommand = nextArm ? "ARM" : "DISARM";
                    updateFlightButtons();
                });
            });
        }
        if (m_btnEmergencyStop != null) {
            m_btnEmergencyStop.setOnClickListener(v ->
                    sendSimpleCmdAwaitAck("EMERGENCY_STOP", CMD_EMERGENCY_STOP, PENDING_EMERGENCY_STOP, () -> {
                        m_armLatched = false;
                        m_lastFlightCommand = "EMERGENCY_STOP";
                        updateFlightButtons();
                    }));
        }
        if (m_btnOffboard != null) {
            m_btnOffboard.setOnClickListener(v -> {
                sendSimpleCmdAwaitAck("OFFBOARD", CMD_OFFBOARD, PENDING_OFFBOARD, () -> {
                    m_lastFlightCommand = "OFFBOARD";
                    updateFlightButtons();
                });
            });
        }
        if (m_btnHold != null) {
            m_btnHold.setOnClickListener(v -> {
                sendSimpleCmdAwaitAck("HOLD", CMD_HOLD, PENDING_HOLD, () -> {
                    m_lastFlightCommand = "HOLD";
                    updateFlightButtons();
                });
            });
        }
        if (m_btnLand != null) {
            m_btnLand.setOnClickListener(v -> {
                sendSimpleCmdAwaitAck("LAND", CMD_LAND, PENDING_LAND, () -> {
                    m_lastFlightCommand = "LAND";
                    updateFlightButtons();
                });
            });
        }
        if (m_btnCleanCalib != null) {
            m_btnCleanCalib.setOnClickListener(v ->
                    sendSimpleCmdAwaitAck("CLEAN_CALIB", CMD_CALIB_CLEAN, PENDING_CLEAN_CALIB, () -> { }));
        }
        if (m_btnSensorMode != null) {
            m_btnSensorMode.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final int nextSensorMode = isChecked ? SENSOR_STEREO_IMU : SENSOR_STEREO;
                final int exposureUs = m_cfgExposureUs;
                final float gain = (float) m_cfgGain;
                sendRuntimeConfigAwaitAck(
                        exposureUs,
                        gain,
                        nextSensorMode,
                        m_sendImage,
                        m_sendFeature,
                        m_sendMap,
                        "Sensor mode",
                        PENDING_SENSOR,
                        () -> {
                            m_sensorMode = nextSensorMode;
                            updateRuntimeButtons();
                        });
            });
        }

        m_btnToggleSlam.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (m_updatingToggleUi) {
                return;
            }
            if (isChecked) {
                sendRuntimeMode(MODE_SLAM, "Start VIO");
            } else if (m_runtimeMode == MODE_SLAM) {
                stopRuntime("Stop VIO");
            } else {
                updateRuntimeButtons();
            }
        });
        m_btnToggleCalib.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (m_updatingToggleUi) {
                return;
            }
            if (isChecked) {
                sendRuntimeMode(MODE_CALIB, "Start Calib");
            } else if (m_runtimeMode == MODE_CALIB) {
                stopRuntime("Stop Calib");
            } else {
                updateRuntimeButtons();
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
        outState.putBoolean(KEY_SETTINGS_VISIBLE, m_settingsVisible);
        outState.putBoolean(KEY_DEBUG_VISIBLE, m_debugVisible);
        outState.putBoolean(KEY_REMOTE_VISIBLE, m_remoteVisible);
        super.onSaveInstanceState(outState);
    }
}
