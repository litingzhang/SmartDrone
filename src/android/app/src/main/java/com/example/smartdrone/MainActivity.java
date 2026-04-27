package com.example.smartdrone;

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
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.AutoCompleteTextView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.Switch;
import android.widget.TextView;
import com.example.smartdrone.R;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
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
    private static final int CMD_POSITION = 0x16;
    private static final int CMD_RUNTIME_MODE = 0x30;
    private static final int CMD_RUNTIME_CONFIG = 0x31;
    private static final int CMD_CALIB_CLEAN = 0x32;
    private static final int CMD_GET_CAPABILITIES = 0x33;
    private static final int CMD_GET_CONFIG = 0x34;
    private static final int CMD_FORCE_RESTART = 0x35;
    private static final int CMD_ACK = 0xF0;
    private static final int CMD_STATE = 0xF1;
    private static final int CMD_POINT_CLOUD = 0xF2;
    private static final int CMD_CAPABILITIES = 0xF3;
    private static final int CMD_CONFIG = 0xF4;
    private static final int CMD_HEARTBEAT = 0xF5;

    private static final int MODE_IDLE = 0;
    private static final int MODE_SLAM = 1;
    private static final int MODE_CALIB = 2;
    private static final int SENSOR_STEREO = 0;
    private static final int SENSOR_STEREO_IMU = 1;
    private static final int SENSOR_MONO = 2;
    private static final int SENSOR_MONO_IMU = 3;
    private static final int FEATURE_FRONTEND_ORB = 0;
    private static final int FEATURE_FRONTEND_XFEAT = 1;
    private static final int FEATURE_FRONTEND_DROID_LIGHT = 2;
    private static final int FEATURE_FRONTEND_LK = 3;
    private static final int FEATURE_FRONTEND_LK_GFTT_PER_FRAME = 4;
    private static final int FEATURE_FRONTEND_LK_XFEAT = 5;
    private static final int FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI = 6;
    private static final int LK_PER_FRAME_ACCEL_CPU = 0;
    private static final int LK_PER_FRAME_ACCEL_VPI_CUDA = 1;
    private static final int SLAM_MODE_MAPPING = 0;
    private static final int SLAM_MODE_LOCALIZATION = 1;
    private static final int SLAM_MODE_AUTO = 4;
    private static final int PX4_MAIN_MODE_MANUAL = 1;
    private static final int PX4_MAIN_MODE_ALTCTL = 2;
    private static final int PX4_MAIN_MODE_POSCTL = 3;
    private static final int PX4_MAIN_MODE_AUTO = 4;
    private static final int PX4_MAIN_MODE_ACRO = 5;
    private static final int PX4_MAIN_MODE_OFFBOARD = 6;
    private static final int PX4_MAIN_MODE_STABILIZED = 7;
    private static final int PX4_AUTO_SUB_MODE_READY = 1;
    private static final int PX4_AUTO_SUB_MODE_TAKEOFF = 2;
    private static final int PX4_AUTO_SUB_MODE_LOITER = 3;
    private static final int PX4_AUTO_SUB_MODE_MISSION = 4;
    private static final int PX4_AUTO_SUB_MODE_RTL = 5;
    private static final int PX4_AUTO_SUB_MODE_LAND = 6;
    private static final int PX4_AUTO_SUB_MODE_FOLLOW_TARGET = 8;
    private static final long ACK_PENDING_TIMEOUT_MS = 3000L;
    private static final String PENDING_ARM = "arm";
    private static final String PENDING_EMERGENCY_STOP = "emergency_stop";
    private static final String PENDING_OFFBOARD = "offboard";
    private static final String PENDING_LAND = "land";
    private static final String PENDING_POSITION = "position";
    private static final String PENDING_RUNTIME = "runtime";
    private static final String PENDING_SENSOR = "sensor";
    private static final String PENDING_CONFIG = "config";
    private static final String PENDING_CLEAN_CALIB = "clean_calib";
    private static final int EXPOSURE_MIN_US = 100;
    private static final int EXPOSURE_MAX_US = 30000;
    private static final int EXPOSURE_STEP_US = 100;
    private static final int GAIN_MIN = 0;
    private static final int GAIN_MAX = 255;
    private static final int PAIR_MS_MIN = 1;
    private static final int PAIR_MS_MAX = 20;
    private static final int SLAM_FPS_MIN = 1;
    private static final int SLAM_FPS_MAX = 120;
    private static final int SLAM_FPS_DEFAULT = 30;
    private static final int TBC_TRANSLATION_MIN_MM = -300;
    private static final int TBC_TRANSLATION_MAX_MM = 300;
    private static final int TBC_ROLL_MIN_TENTH_DEG = -100;
    private static final int TBC_ROLL_MAX_TENTH_DEG = 100;
    private static final int TBC_PITCH_MIN_TENTH_DEG = -100;
    private static final int TBC_PITCH_MAX_TENTH_DEG = 1000;
    private static final int TBC_YAW_MIN_TENTH_DEG = -100;
    private static final int TBC_YAW_MAX_TENTH_DEG = 100;
    private static final int ORB_NFEATURES_MIN = 500;
    private static final int ORB_NFEATURES_MAX = 3000;
    private static final int ORB_NFEATURES_STEP = 50;
    private static final int ORB_SCALE_MIN_CENTI = 105;
    private static final int ORB_SCALE_MAX_CENTI = 200;
    private static final int ORB_NLEVELS_MIN = 4;
    private static final int ORB_NLEVELS_MAX = 12;
    private static final int ORB_FAST_TH_MIN = 2;
    private static final int ORB_FAST_TH_MAX = 40;
    private static final int XFEAT_TOP_K_MIN = 1;
    private static final int XFEAT_TOP_K_MAX = 4096;
    private static final int XFEAT_MAX_POINTS_MIN = 1;
    private static final int XFEAT_MAX_POINTS_MAX = 4096;
    private static final int XFEAT_INPUT_MAX_MIN = 0;
    private static final int XFEAT_INPUT_MAX_MAX = 4096;
    private static final int XFEAT_INPUT_MAX_STEP = 16;

    private static final int FRAME_NED = 2;
    private static final long JOYSTICK_PERIOD_MS = 50L;
    private static final long RX_POLL_PERIOD_MS = 5L;
    private static final long HEARTBEAT_PERIOD_MS = 500L;
    private static final long HEARTBEAT_TIMEOUT_MS = 3000L;
    private static final float BUTTON_AXIS_MAGNITUDE = 0.6f;
    private static final float BUTTON_MAX_SPEED_MPS = 3.0f;
    private static final int VIDEO_MAGIC = 0x5643494D;
    private static final int VIDEO_HEADER_LEN = 36;
    private static final int MAX_RX_PACKETS_PER_TICK = 96;
    private static final int MAX_VIDEO_JPEG_BYTES = 2 * 1024 * 1024;
    private static final int VIDEO_FLAG_FEATURE_POINTS = 0x01;
    private static final double FRAME_MATCH_TOLERANCE_SEC = 0.002;
    private static final String KEY_SETTINGS_VISIBLE = "settingsVisible";
    private static final String KEY_DEBUG_VISIBLE = "debugVisible";
    private static final String KEY_REMOTE_VISIBLE = "remoteVisible";
    private static final int DEFAULT_CMD_PORT = 14550;
    private static final int DEFAULT_PHONE_VIDEO_PORT = 5000;
    private static final int DISCOVERY_PORT = 15000;
    private static final int DISCOVERY_SOCKET_TIMEOUT_MS = 1000;
    private static final String DISCOVERY_MAGIC = "smartdrone_discovery";
    private static final String[] DEFAULT_VEHICLE_IPS = new String[] {"10.42.0.1", "192.168.0.105"};
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
    private Button m_btnLand;
    private Switch m_btnToggleSlam;
    private Switch m_btnToggleCalib;
    private Switch m_btnAutoExposureToggle;
    private Switch m_btnTbcOverrideToggle;
    private Spinner m_spinnerSensorMode;
    private Spinner m_spinnerFeatureFrontend;
    private Button m_btnQuickSlamAuto;
    private Button m_btnQuickSlamManual;
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
    private TextView m_tvCfgPairMsValue;
    private TextView m_tvCfgSlamFpsValue;
    private TextView m_tvCfgTbcXValue;
    private TextView m_tvCfgTbcYValue;
    private TextView m_tvCfgTbcZValue;
    private TextView m_tvCfgTbcRollValue;
    private TextView m_tvCfgTbcPitchValue;
    private TextView m_tvCfgTbcYawValue;
    private TextView m_tvCfgOrbNFeaturesValue;
    private TextView m_tvCfgOrbScaleFactorValue;
    private TextView m_tvCfgOrbNLevelsValue;
    private TextView m_tvCfgOrbIniThFastValue;
    private TextView m_tvCfgOrbMinThFastValue;
    private TextView m_tvCfgXFeatTopKValue;
    private TextView m_tvCfgXFeatMaxPointsValue;
    private TextView m_tvCfgXFeatInputMaxWidthValue;
    private TextView m_tvCfgXFeatInputMaxHeightValue;
    private SeekBar m_sbCfgExposure;
    private SeekBar m_sbCfgGain;
    private SeekBar m_sbCfgPairMs;
    private SeekBar m_sbCfgSlamFps;
    private SeekBar m_sbCfgTbcX;
    private SeekBar m_sbCfgTbcY;
    private SeekBar m_sbCfgTbcZ;
    private SeekBar m_sbCfgTbcRoll;
    private SeekBar m_sbCfgTbcPitch;
    private SeekBar m_sbCfgTbcYaw;
    private SeekBar m_sbCfgOrbNFeatures;
    private SeekBar m_sbCfgOrbScaleFactor;
    private SeekBar m_sbCfgOrbNLevels;
    private SeekBar m_sbCfgOrbIniThFast;
    private SeekBar m_sbCfgOrbMinThFast;
    private SeekBar m_sbCfgXFeatTopK;
    private SeekBar m_sbCfgXFeatMaxPoints;
    private SeekBar m_sbCfgXFeatInputMaxWidth;
    private SeekBar m_sbCfgXFeatInputMaxHeight;

    private View m_joystickLeft;
    private View m_joystickRight;
    private ImageButton m_btnLeftUp;
    private ImageButton m_btnLeftDown;
    private ImageButton m_btnLeftYawLeft;
    private ImageButton m_btnLeftYawRight;
    private ImageButton m_btnRightForward;
    private ImageButton m_btnRightBack;
    private ImageButton m_btnRightLeft;
    private ImageButton m_btnRightRight;

    private final Handler m_handler = new Handler(Looper.getMainLooper());

    private volatile boolean m_leftUpPressed;
    private volatile boolean m_leftDownPressed;
    private volatile boolean m_leftYawLeftPressed;
    private volatile boolean m_leftYawRightPressed;
    private volatile boolean m_rightForwardPressed;
    private volatile boolean m_rightBackPressed;
    private volatile boolean m_rightLeftPressed;
    private volatile boolean m_rightRightPressed;

    private long m_lastJoystickTickMs;
    private boolean m_joystickLoopRunning;
    private boolean m_lastJoystickActive;
    private boolean m_settingsVisible = false;
    private boolean m_debugVisible = true;
    private boolean m_remoteVisible = true;
    private boolean m_updatingToggleUi = false;
    private boolean m_updatingConfigUi = false;
    private boolean m_rxLoopRunning;
    private boolean m_heartbeatLoopRunning;
    private volatile boolean m_discoveryLoopRunning;
    private Thread m_discoveryThread;
    private boolean m_udpReady = false;
    private int m_runtimeMode = MODE_IDLE;
    private String m_vehicleIp = "10.42.0.1";
    private String m_lastDiscoveredVehicleIp = "";
    private int m_vehicleCmdPort = DEFAULT_CMD_PORT;
    private int m_phoneVideoPort = DEFAULT_PHONE_VIDEO_PORT;
    private boolean m_armLatched = false;
    private String m_lastFlightCommand = "";
    private int m_px4MainMode = 0;
    private int m_px4SubMode = 0;
    private int m_sensorMode = SENSOR_STEREO;
    private int m_cfgFeatureFrontend = FEATURE_FRONTEND_ORB;
    private boolean m_cfgLkXFeatSeeding = false;
    private int m_cfgLkPerFrameAcceleration = LK_PER_FRAME_ACCEL_CPU;
    private int m_cfgExposureUs = 3000;
    private int m_cfgGain = 2;
    private boolean m_cfgAutoExposure = true;
    private int m_cfgPairMs = 5;
    private int m_cfgSlamFps = SLAM_FPS_DEFAULT;
    private int m_cfgSlamMode = SLAM_MODE_MAPPING;
    private boolean m_cfgUseCustomTbc = false;
    private float m_cfgTbcTx = 0.0f;
    private float m_cfgTbcTy = 0.0f;
    private float m_cfgTbcTz = 0.0f;
    private float m_cfgTbcRollDeg = 0.0f;
    private float m_cfgTbcPitchDeg = 0.0f;
    private float m_cfgTbcYawDeg = 0.0f;
    private int m_cfgOrbNFeatures = 1200;
    private float m_cfgOrbScaleFactor = 1.2f;
    private int m_cfgOrbNLevels = 8;
    private int m_cfgOrbIniThFast = 16;
    private int m_cfgOrbMinThFast = 6;
    private int m_cfgXFeatTopK = 512;
    private int m_cfgXFeatMaxPoints = 320;
    private int m_cfgXFeatInputMaxWidth = 640;
    private int m_cfgXFeatInputMaxHeight = 400;
    private int m_effectiveSlamMode = SLAM_MODE_MAPPING;
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
    private boolean m_sendMap = false;
    private boolean m_showFeaturePoints = true;
    private long m_lastVehicleHeartbeatMs = 0L;
    private boolean m_vehicleHeartbeatTimeoutHandled = false;
    private boolean m_featureDefaultEnsured = false;
    private boolean m_supportsCalib = true;
    private boolean m_supportsStereoImu = true;
    private boolean m_supportsMono = true;
    private boolean m_supportsMonoImu = true;
    private boolean m_supportsXFeat = true;
    private boolean m_supportsDroidLight = false;
    private boolean m_supportsLK = true;
    private boolean m_isPackedStereoUvc = false;
    private boolean m_pairWindowRequired = true;
    private boolean m_cfgUvcPackedStereo = false;
    private String m_lastCapabilitiesText = "";
    private String m_lastConfigText = "";
    private int[] m_availableSensorModes = new int[] {SENSOR_STEREO, SENSOR_STEREO_IMU, SENSOR_MONO, SENSOR_MONO_IMU};
    private final Paint m_featurePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_lkGridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
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

        void reset()
        {
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

        void reset()
        {
            frameId = -1;
            frameTimeSec = Double.NaN;
            width = 0;
            height = 0;
            xs = null;
            ys = null;
            count = 0;
        }
    }

    private final VideoAssembly[] m_videoAssemblies = new VideoAssembly[] {new VideoAssembly(), new VideoAssembly()};
    private final DisplayFrame[] m_displayFrames = new DisplayFrame[] {new DisplayFrame(), new DisplayFrame()};
    private final FeatureFrame[] m_featureFrames = new FeatureFrame[] {new FeatureFrame(), new FeatureFrame()};

    private static final class DiscoveryAnnouncement {
        String vehicleIp = "";
        int cmdPort = DEFAULT_CMD_PORT;
        int videoPort = DEFAULT_PHONE_VIDEO_PORT;
    }

    private final Runnable m_joystickLoop = new Runnable() {
        @Override public void run()
        {
            if (!m_joystickLoopRunning) {
                return;
            }
            tickJoystickControl();
            m_handler.postDelayed(this, JOYSTICK_PERIOD_MS);
        }
    };

    private final Runnable m_rxLoop = new Runnable() {
        @Override public void run()
        {
            if (!m_rxLoopRunning) {
                return;
            }
            tickRxLoop();
            m_handler.postDelayed(this, RX_POLL_PERIOD_MS);
        }
    };

    private final Runnable m_heartbeatLoop = new Runnable() {
        @Override public void run()
        {
            if (!m_heartbeatLoopRunning) {
                return;
            }
            tickHeartbeatLoop();
            m_handler.postDelayed(this, HEARTBEAT_PERIOD_MS);
        }
    };

    private static float parseF(EditText et, float defVal)
    {
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

    private static int parseI(EditText et, int defVal)
    {
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

    private static int clampInt(int value, int min, int max) { return Math.max(min, Math.min(max, value)); }

    private static String px4ModeToText(int mainMode, int subMode)
    {
        switch (mainMode) {
        case PX4_MAIN_MODE_MANUAL:
            return "MANUAL";
        case PX4_MAIN_MODE_ALTCTL:
            return "ALTCTL";
        case PX4_MAIN_MODE_POSCTL:
            return "POSCTL";
        case PX4_MAIN_MODE_AUTO:
            switch (subMode) {
            case PX4_AUTO_SUB_MODE_READY:
                return "AUTO.READY";
            case PX4_AUTO_SUB_MODE_TAKEOFF:
                return "AUTO.TAKEOFF";
            case PX4_AUTO_SUB_MODE_LOITER:
                return "AUTO.LOITER";
            case PX4_AUTO_SUB_MODE_MISSION:
                return "AUTO.MISSION";
            case PX4_AUTO_SUB_MODE_RTL:
                return "AUTO.RTL";
            case PX4_AUTO_SUB_MODE_LAND:
                return "AUTO.LAND";
            case PX4_AUTO_SUB_MODE_FOLLOW_TARGET:
                return "AUTO.FOLLOW";
            default:
                return "AUTO(" + subMode + ")";
            }
        case PX4_MAIN_MODE_ACRO:
            return "ACRO";
        case PX4_MAIN_MODE_OFFBOARD:
            return "OFFBOARD";
        case PX4_MAIN_MODE_STABILIZED:
            return "STABILIZED";
        default:
            return mainMode > 0 ? ("MODE(" + mainMode + ":" + subMode + ")") : "UNKNOWN";
        }
    }

    private boolean isPx4AutoMode()
    {
        return m_px4MainMode == PX4_MAIN_MODE_AUTO;
    }

    private boolean isPx4PositionMode()
    {
        return m_px4MainMode == PX4_MAIN_MODE_POSCTL;
    }

    private static int parseIntOrDefault(String text, int defaultValue)
    {
        try {
            return Integer.parseInt(text);
        } catch (Throwable ignored) {
            return defaultValue;
        }
    }

    private static float parseFloatOrDefault(String text, float defaultValue)
    {
        try {
            return Float.parseFloat(text);
        } catch (Throwable ignored) {
            return defaultValue;
        }
    }

    private static DiscoveryAnnouncement parseDiscoveryAnnouncement(byte[] data, int len, InetAddress sourceAddress)
    {
        if (data == null || len <= 0 || sourceAddress == null) {
            return null;
        }
        final String text = new String(data, 0, len, StandardCharsets.UTF_8).trim();
        if (text.isEmpty() || !text.contains(DISCOVERY_MAGIC)) {
            return null;
        }
        DiscoveryAnnouncement out = new DiscoveryAnnouncement();
        out.vehicleIp = sourceAddress.getHostAddress();
        String[] fields = text.split(";");
        for (String rawField : fields) {
            String field = rawField != null ? rawField.trim() : "";
            if (field.isEmpty()) {
                continue;
            }
            int eq = field.indexOf('=');
            if (eq <= 0 || eq >= field.length() - 1) {
                continue;
            }
            String key = field.substring(0, eq).trim().toLowerCase(Locale.US);
            String value = field.substring(eq + 1).trim();
            if ("ip".equals(key) && !value.isEmpty()) {
                out.vehicleIp = value;
            } else if ("cmd".equals(key)) {
                out.cmdPort = parseIntOrDefault(value, DEFAULT_CMD_PORT);
            } else if ("video".equals(key)) {
                out.videoPort = parseIntOrDefault(value, DEFAULT_PHONE_VIDEO_PORT);
            }
        }
        if (out.vehicleIp.isEmpty()) {
            return null;
        }
        return out;
    }

    private static int quantizeExposureUs(int exposureUs)
    {
        int clamped = clampInt(exposureUs, EXPOSURE_MIN_US, EXPOSURE_MAX_US);
        int steps = Math.round((clamped - EXPOSURE_MIN_US) / (float)EXPOSURE_STEP_US);
        return EXPOSURE_MIN_US + steps * EXPOSURE_STEP_US;
    }

    private static int quantizeGain(int gain) { return clampInt(gain, GAIN_MIN, GAIN_MAX); }

    private static int quantizePairMs(int pairMs) { return clampInt(pairMs, PAIR_MS_MIN, PAIR_MS_MAX); }

    private static int quantizeSlamFps(int slamFps) { return clampInt(slamFps, SLAM_FPS_MIN, SLAM_FPS_MAX); }

    private static boolean containsBehaviorNote(String notes, String keyPrefix)
    {
        return behaviorNoteValue(notes, keyPrefix) != null;
    }

    private static String behaviorNoteValue(String notes, String keyPrefix)
    {
        if (notes == null || notes.trim().isEmpty()) {
            return null;
        }
        String[] items = notes.split(";");
        for (String item : items) {
            if (item == null) {
                continue;
            }
            String trimmed = item.trim();
            if (trimmed.startsWith(keyPrefix)) {
                return trimmed.substring(keyPrefix.length()).trim();
            }
        }
        return null;
    }

    private static float quantizeTbcTranslationM(float meters)
    {
        int mm = Math.round(meters * 1000.0f);
        mm = clampInt(mm, TBC_TRANSLATION_MIN_MM, TBC_TRANSLATION_MAX_MM);
        return mm / 1000.0f;
    }

    private static float quantizeTbcAngleDeg(float degrees, int minTenthDeg, int maxTenthDeg)
    {
        int tenthDeg = Math.round(degrees * 10.0f);
        tenthDeg = clampInt(tenthDeg, minTenthDeg, maxTenthDeg);
        return tenthDeg / 10.0f;
    }

    private static int quantizeOrbNFeatures(int value)
    {
        int clamped = clampInt(value, ORB_NFEATURES_MIN, ORB_NFEATURES_MAX);
        int steps = Math.round((clamped - ORB_NFEATURES_MIN) / (float)ORB_NFEATURES_STEP);
        return ORB_NFEATURES_MIN + steps * ORB_NFEATURES_STEP;
    }

    private static float quantizeOrbScaleFactor(float value)
    {
        int centi = Math.round(value * 100.0f);
        centi = clampInt(centi, ORB_SCALE_MIN_CENTI, ORB_SCALE_MAX_CENTI);
        return centi / 100.0f;
    }

    private static int quantizeOrbNLevels(int value) { return clampInt(value, ORB_NLEVELS_MIN, ORB_NLEVELS_MAX); }

    private static int quantizeOrbFastThreshold(int value) { return clampInt(value, ORB_FAST_TH_MIN, ORB_FAST_TH_MAX); }

    private static int quantizeXFeatTopK(int value) { return clampInt(value, XFEAT_TOP_K_MIN, XFEAT_TOP_K_MAX); }

    private static int quantizeXFeatMaxPoints(int value, int topK)
    {
        return clampInt(value, XFEAT_MAX_POINTS_MIN, Math.max(XFEAT_MAX_POINTS_MIN, quantizeXFeatTopK(topK)));
    }

    private static int quantizeXFeatInputMax(int value)
    {
        final int clamped = clampInt(value, XFEAT_INPUT_MAX_MIN, XFEAT_INPUT_MAX_MAX);
        if (clamped == 0) {
            return 0;
        }
        return clampInt(Math.round((float)clamped / (float)XFEAT_INPUT_MAX_STEP) * XFEAT_INPUT_MAX_STEP,
                        XFEAT_INPUT_MAX_STEP, XFEAT_INPUT_MAX_MAX);
    }

    private static Float findPoseField(String text, String... keys)
    {
        if (text == null) {
            return null;
        }
        for (String key : keys) {
            Pattern pattern = Pattern.compile("(?i)\\b" + Pattern.quote(key) + "\\s*[:=]\\s*(-?\\d+(?:\\.\\d+)?)");
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

    private static int readLeU16(byte[] data, int offset)
    {
        return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8);
    }

    private static int readFramePayloadLen(byte[] data)
    {
        if (data == null || data.length < 17) {
            return -1;
        }
        return readLeU16(data, 5);
    }

    private static float readLeF32(byte[] data, int offset)
    {
        int bits = (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8) | ((data[offset + 2] & 0xFF) << 16) |
                   ((data[offset + 3] & 0xFF) << 24);
        return Float.intBitsToFloat(bits);
    }

    private static float clampUnit(float value) { return Math.max(-1.0f, Math.min(1.0f, value)); }

    private static float quatYawDeg(float qw, float qx, float qy, float qz)
    {
        double siny = 2.0 * (qw * qz + qx * qy);
        double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
        return (float)Math.toDegrees(Math.atan2(siny, cosy));
    }

    private static float quatPitchDeg(float qw, float qx, float qy, float qz)
    {
        double sinp = 2.0 * (qw * qy - qz * qx);
        return (float)Math.toDegrees(Math.asin(clampUnit((float)sinp)));
    }

    private static float quatRollDeg(float qw, float qx, float qy, float qz)
    {
        double sinr = 2.0 * (qw * qx + qy * qz);
        double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
        return (float)Math.toDegrees(Math.atan2(sinr, cosr));
    }

    private boolean tryHandleStatePoseForMap(byte[] rx)
    {
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
        if ((payloadLen != 34 && payloadLen != 35 && payloadLen != 36 && payloadLen != 38) || rx.length < 17 + payloadLen) {
            return false;
        }
        final int payloadOffset = 15;
        final int trackingOffset = payloadLen >= 35 ? payloadOffset + 2 : payloadOffset + 1;
        final int armedOffset = payloadLen >= 36 ? trackingOffset + 1 : -1;
        final int resetBaseOffset = armedOffset >= 0 ? (armedOffset + 1) : (trackingOffset + 1);
        final int poseOffset = payloadLen >= 34 ? resetBaseOffset + 4 : payloadOffset + 4;
        final float x = readLeF32(rx, poseOffset);
        final float y = readLeF32(rx, poseOffset + 4);
        final float z = readLeF32(rx, poseOffset + 8);
        final float qw = readLeF32(rx, poseOffset + 12);
        final float qx = readLeF32(rx, poseOffset + 16);
        final float qy = readLeF32(rx, poseOffset + 20);
        final float qz = readLeF32(rx, poseOffset + 24);
        float mapX = x;
        float mapY = y;
        float mapZ = z;
        float rollDeg = quatRollDeg(qw, qx, qy, qz);
        float pitchDeg = quatPitchDeg(qw, qx, qy, qz);
        float yawDeg = quatYawDeg(qw, qx, qy, qz);
        if (m_map3dView != null) {
            m_map3dView.setPose(mapX, mapY, mapZ, rollDeg, pitchDeg, yawDeg, true);
        }
        return true;
    }

    private boolean tryHandlePointCloudPacket(byte[] rx)
    {
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

    private boolean tryHandleHeartbeatPacket(byte[] rx)
    {
        if (rx == null || rx.length < 17) {
            return false;
        }
        if ((rx[0] & 0xFF) != 0xAA || (rx[1] & 0xFF) != 0x55 || (rx[3] & 0xFF) != CMD_HEARTBEAT) {
            return false;
        }
        m_lastVehicleHeartbeatMs = System.currentTimeMillis();
        m_vehicleHeartbeatTimeoutHandled = false;
        return true;
    }

    private void updatePoseMapFromText()
    {
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
        m_map3dView.setPose(x, y, z, roll != null ? roll : 0f, pitch != null ? pitch : 0f, yaw != null ? yaw : 0f,
                            true);
    }

    private void updateConfigViews()
    {
        if (m_updatingConfigUi) {
            return;
        }
        m_updatingConfigUi = true;
        try {
            final boolean runtimeActive = isRuntimeActive();
            final boolean packedStereoUvc = m_isPackedStereoUvc || m_cfgUvcPackedStereo;
            final boolean manualExposureEditable = !runtimeActive && !m_cfgAutoExposure;
            final boolean tbcEditable = m_cfgUseCustomTbc && m_sensorMode == SENSOR_STEREO;
            final boolean orbEditable = !runtimeActive;
            final boolean xfeatEditable = !runtimeActive && m_cfgFeatureFrontend == FEATURE_FRONTEND_LK &&
                                          m_cfgLkXFeatSeeding;
            if (m_tvCfgExposureValue != null) {
                m_tvCfgExposureValue.setText(m_cfgAutoExposure ? "Exposure: Auto (UVC AE)"
                                                               : String.format(Locale.US, "Exposure: %d us (UVC)",
                                                                               m_cfgExposureUs));
                m_tvCfgExposureValue.setAlpha(manualExposureEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgGainValue != null) {
                m_tvCfgGainValue.setText(String.format(Locale.US, "Gain: %d (UVC)", m_cfgGain));
                m_tvCfgGainValue.setAlpha(manualExposureEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgPairMsValue != null) {
                m_tvCfgPairMsValue.setText(packedStereoUvc ? "Pair Window: n/a (packed UVC stereo)"
                                                           : String.format(Locale.US, "Pair Window: %d ms", m_cfgPairMs));
                m_tvCfgPairMsValue.setAlpha((runtimeActive || packedStereoUvc) ? 0.45f : 1.0f);
            }
            if (m_tvCfgSlamFpsValue != null) {
                m_tvCfgSlamFpsValue.setText(String.format(Locale.US, "SLAM FPS: %d", m_cfgSlamFps));
            }
            if (m_tvCfgTbcXValue != null) {
                m_tvCfgTbcXValue.setText(String.format(Locale.US, "T_b_c1 Tx: %.3f m", m_cfgTbcTx));
                m_tvCfgTbcXValue.setAlpha(tbcEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgTbcYValue != null) {
                m_tvCfgTbcYValue.setText(String.format(Locale.US, "T_b_c1 Ty: %.3f m", m_cfgTbcTy));
                m_tvCfgTbcYValue.setAlpha(tbcEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgTbcZValue != null) {
                m_tvCfgTbcZValue.setText(String.format(Locale.US, "T_b_c1 Tz: %.3f m", m_cfgTbcTz));
                m_tvCfgTbcZValue.setAlpha(tbcEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgTbcRollValue != null) {
                m_tvCfgTbcRollValue.setText(String.format(Locale.US, "Camera Roll: %.1f deg", m_cfgTbcRollDeg));
                m_tvCfgTbcRollValue.setAlpha(tbcEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgTbcPitchValue != null) {
                m_tvCfgTbcPitchValue.setText(String.format(Locale.US, "Camera Pitch: %.1f deg", m_cfgTbcPitchDeg));
                m_tvCfgTbcPitchValue.setAlpha(tbcEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgTbcYawValue != null) {
                m_tvCfgTbcYawValue.setText(String.format(Locale.US, "Camera Yaw: %.1f deg", m_cfgTbcYawDeg));
                m_tvCfgTbcYawValue.setAlpha(tbcEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgOrbNFeaturesValue != null) {
                m_tvCfgOrbNFeaturesValue.setText(String.format(Locale.US, "ORB nFeatures: %d", m_cfgOrbNFeatures));
                m_tvCfgOrbNFeaturesValue.setAlpha(orbEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgOrbScaleFactorValue != null) {
                m_tvCfgOrbScaleFactorValue.setText(
                    String.format(Locale.US, "ORB scaleFactor: %.2f", m_cfgOrbScaleFactor));
                m_tvCfgOrbScaleFactorValue.setAlpha(orbEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgOrbNLevelsValue != null) {
                m_tvCfgOrbNLevelsValue.setText(String.format(Locale.US, "ORB nLevels: %d", m_cfgOrbNLevels));
                m_tvCfgOrbNLevelsValue.setAlpha(orbEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgOrbIniThFastValue != null) {
                m_tvCfgOrbIniThFastValue.setText(String.format(Locale.US, "ORB iniThFAST: %d", m_cfgOrbIniThFast));
                m_tvCfgOrbIniThFastValue.setAlpha(orbEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgOrbMinThFastValue != null) {
                m_tvCfgOrbMinThFastValue.setText(String.format(Locale.US, "ORB minThFAST: %d", m_cfgOrbMinThFast));
                m_tvCfgOrbMinThFastValue.setAlpha(orbEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgXFeatTopKValue != null) {
                m_tvCfgXFeatTopKValue.setText(String.format(Locale.US, "XFeat topK: %d", m_cfgXFeatTopK));
                m_tvCfgXFeatTopKValue.setAlpha(xfeatEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgXFeatMaxPointsValue != null) {
                m_tvCfgXFeatMaxPointsValue.setText(String.format(Locale.US, "XFeat maxPoints: %d", m_cfgXFeatMaxPoints));
                m_tvCfgXFeatMaxPointsValue.setAlpha(xfeatEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgXFeatInputMaxWidthValue != null) {
                final String widthText = m_cfgXFeatInputMaxWidth > 0 ? Integer.toString(m_cfgXFeatInputMaxWidth) : "off";
                m_tvCfgXFeatInputMaxWidthValue.setText("XFeat input max width: " + widthText);
                m_tvCfgXFeatInputMaxWidthValue.setAlpha(xfeatEditable ? 1.0f : 0.45f);
            }
            if (m_tvCfgXFeatInputMaxHeightValue != null) {
                final String heightText = m_cfgXFeatInputMaxHeight > 0 ? Integer.toString(m_cfgXFeatInputMaxHeight) : "off";
                m_tvCfgXFeatInputMaxHeightValue.setText("XFeat input max height: " + heightText);
                m_tvCfgXFeatInputMaxHeightValue.setAlpha(xfeatEditable ? 1.0f : 0.45f);
            }
            if (m_sbCfgExposure != null) {
                int progress = (m_cfgExposureUs - EXPOSURE_MIN_US) / EXPOSURE_STEP_US;
                if (m_sbCfgExposure.getProgress() != progress) {
                    m_sbCfgExposure.setProgress(progress);
                }
                m_sbCfgExposure.setEnabled(manualExposureEditable);
                m_sbCfgExposure.setAlpha(manualExposureEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgGain != null) {
                int progress = m_cfgGain - GAIN_MIN;
                if (m_sbCfgGain.getProgress() != progress) {
                    m_sbCfgGain.setProgress(progress);
                }
                m_sbCfgGain.setEnabled(manualExposureEditable);
                m_sbCfgGain.setAlpha(manualExposureEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgPairMs != null) {
                int progress = m_cfgPairMs - PAIR_MS_MIN;
                if (m_sbCfgPairMs.getProgress() != progress) {
                    m_sbCfgPairMs.setProgress(progress);
                }
                final boolean pairWindowEditable = !runtimeActive && !packedStereoUvc;
                m_sbCfgPairMs.setEnabled(pairWindowEditable);
                m_sbCfgPairMs.setAlpha(pairWindowEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgSlamFps != null) {
                int progress = m_cfgSlamFps - SLAM_FPS_MIN;
                if (m_sbCfgSlamFps.getProgress() != progress) {
                    m_sbCfgSlamFps.setProgress(progress);
                }
            }
            if (m_sbCfgTbcX != null) {
                final int progress = Math.round(m_cfgTbcTx * 1000.0f) - TBC_TRANSLATION_MIN_MM;
                if (m_sbCfgTbcX.getProgress() != progress) {
                    m_sbCfgTbcX.setProgress(progress);
                }
                m_sbCfgTbcX.setEnabled(tbcEditable);
                m_sbCfgTbcX.setAlpha(tbcEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgTbcY != null) {
                final int progress = Math.round(m_cfgTbcTy * 1000.0f) - TBC_TRANSLATION_MIN_MM;
                if (m_sbCfgTbcY.getProgress() != progress) {
                    m_sbCfgTbcY.setProgress(progress);
                }
                m_sbCfgTbcY.setEnabled(tbcEditable);
                m_sbCfgTbcY.setAlpha(tbcEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgTbcZ != null) {
                final int progress = Math.round(m_cfgTbcTz * 1000.0f) - TBC_TRANSLATION_MIN_MM;
                if (m_sbCfgTbcZ.getProgress() != progress) {
                    m_sbCfgTbcZ.setProgress(progress);
                }
                m_sbCfgTbcZ.setEnabled(tbcEditable);
                m_sbCfgTbcZ.setAlpha(tbcEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgTbcRoll != null) {
                final int progress = Math.round(m_cfgTbcRollDeg * 10.0f) - TBC_ROLL_MIN_TENTH_DEG;
                if (m_sbCfgTbcRoll.getProgress() != progress) {
                    m_sbCfgTbcRoll.setProgress(progress);
                }
                m_sbCfgTbcRoll.setEnabled(tbcEditable);
                m_sbCfgTbcRoll.setAlpha(tbcEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgTbcPitch != null) {
                final int progress = Math.round(m_cfgTbcPitchDeg * 10.0f) - TBC_PITCH_MIN_TENTH_DEG;
                if (m_sbCfgTbcPitch.getProgress() != progress) {
                    m_sbCfgTbcPitch.setProgress(progress);
                }
                m_sbCfgTbcPitch.setEnabled(tbcEditable);
                m_sbCfgTbcPitch.setAlpha(tbcEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgTbcYaw != null) {
                final int progress = Math.round(m_cfgTbcYawDeg * 10.0f) - TBC_YAW_MIN_TENTH_DEG;
                if (m_sbCfgTbcYaw.getProgress() != progress) {
                    m_sbCfgTbcYaw.setProgress(progress);
                }
                m_sbCfgTbcYaw.setEnabled(tbcEditable);
                m_sbCfgTbcYaw.setAlpha(tbcEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgOrbNFeatures != null) {
                final int progress = (m_cfgOrbNFeatures - ORB_NFEATURES_MIN) / ORB_NFEATURES_STEP;
                if (m_sbCfgOrbNFeatures.getProgress() != progress) {
                    m_sbCfgOrbNFeatures.setProgress(progress);
                }
                m_sbCfgOrbNFeatures.setEnabled(orbEditable);
                m_sbCfgOrbNFeatures.setAlpha(orbEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgOrbScaleFactor != null) {
                final int progress = Math.round(m_cfgOrbScaleFactor * 100.0f) - ORB_SCALE_MIN_CENTI;
                if (m_sbCfgOrbScaleFactor.getProgress() != progress) {
                    m_sbCfgOrbScaleFactor.setProgress(progress);
                }
                m_sbCfgOrbScaleFactor.setEnabled(orbEditable);
                m_sbCfgOrbScaleFactor.setAlpha(orbEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgOrbNLevels != null) {
                final int progress = m_cfgOrbNLevels - ORB_NLEVELS_MIN;
                if (m_sbCfgOrbNLevels.getProgress() != progress) {
                    m_sbCfgOrbNLevels.setProgress(progress);
                }
                m_sbCfgOrbNLevels.setEnabled(orbEditable);
                m_sbCfgOrbNLevels.setAlpha(orbEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgOrbIniThFast != null) {
                final int progress = m_cfgOrbIniThFast - ORB_FAST_TH_MIN;
                if (m_sbCfgOrbIniThFast.getProgress() != progress) {
                    m_sbCfgOrbIniThFast.setProgress(progress);
                }
                m_sbCfgOrbIniThFast.setEnabled(orbEditable);
                m_sbCfgOrbIniThFast.setAlpha(orbEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgOrbMinThFast != null) {
                final int progress = m_cfgOrbMinThFast - ORB_FAST_TH_MIN;
                if (m_sbCfgOrbMinThFast.getProgress() != progress) {
                    m_sbCfgOrbMinThFast.setProgress(progress);
                }
                m_sbCfgOrbMinThFast.setEnabled(orbEditable);
                m_sbCfgOrbMinThFast.setAlpha(orbEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgXFeatTopK != null) {
                final int progress = m_cfgXFeatTopK - XFEAT_TOP_K_MIN;
                if (m_sbCfgXFeatTopK.getProgress() != progress) {
                    m_sbCfgXFeatTopK.setProgress(progress);
                }
                m_sbCfgXFeatTopK.setEnabled(xfeatEditable);
                m_sbCfgXFeatTopK.setAlpha(xfeatEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgXFeatMaxPoints != null) {
                final int progress = m_cfgXFeatMaxPoints - XFEAT_MAX_POINTS_MIN;
                if (m_sbCfgXFeatMaxPoints.getProgress() != progress) {
                    m_sbCfgXFeatMaxPoints.setProgress(progress);
                }
                m_sbCfgXFeatMaxPoints.setEnabled(xfeatEditable);
                m_sbCfgXFeatMaxPoints.setAlpha(xfeatEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgXFeatInputMaxWidth != null) {
                final int progress = m_cfgXFeatInputMaxWidth / XFEAT_INPUT_MAX_STEP;
                if (m_sbCfgXFeatInputMaxWidth.getProgress() != progress) {
                    m_sbCfgXFeatInputMaxWidth.setProgress(progress);
                }
                m_sbCfgXFeatInputMaxWidth.setEnabled(xfeatEditable);
                m_sbCfgXFeatInputMaxWidth.setAlpha(xfeatEditable ? 1.0f : 0.35f);
            }
            if (m_sbCfgXFeatInputMaxHeight != null) {
                final int progress = m_cfgXFeatInputMaxHeight / XFEAT_INPUT_MAX_STEP;
                if (m_sbCfgXFeatInputMaxHeight.getProgress() != progress) {
                    m_sbCfgXFeatInputMaxHeight.setProgress(progress);
                }
                m_sbCfgXFeatInputMaxHeight.setEnabled(xfeatEditable);
                m_sbCfgXFeatInputMaxHeight.setAlpha(xfeatEditable ? 1.0f : 0.35f);
            }
            if (m_btnAutoExposureToggle != null) {
                m_btnAutoExposureToggle.setChecked(m_cfgAutoExposure);
                m_btnAutoExposureToggle.setEnabled(!runtimeActive);
                m_btnAutoExposureToggle.setAlpha(runtimeActive ? 0.35f : 1.0f);
                m_btnAutoExposureToggle.setText("Auto Exposure (UVC)");
            }
            if (m_btnTbcOverrideToggle != null) {
                m_btnTbcOverrideToggle.setChecked(m_cfgUseCustomTbc);
                m_btnTbcOverrideToggle.setAlpha(m_cfgUseCustomTbc ? 1.0f : 0.65f);
            }
            updateQuickSlamModeButtons();
        } finally {
            m_updatingConfigUi = false;
        }
    }

    private void updateQuickSlamModeButtons()
    {
        final boolean pending = isPending(PENDING_CONFIG) || isPending(PENDING_RUNTIME);
        final boolean autoEnabled = m_cfgSlamMode == SLAM_MODE_AUTO;
        final int manualDisplayMode = autoEnabled ? m_effectiveSlamMode : m_cfgSlamMode;

        if (m_btnQuickSlamAuto != null) {
            m_btnQuickSlamAuto.setText(autoEnabled ? "Auto On" : "Auto Off");
            setButtonState(m_btnQuickSlamAuto, autoEnabled, pending, "#1565C0");
        }

        if (m_btnQuickSlamManual != null) {
            final boolean localization = manualDisplayMode == SLAM_MODE_LOCALIZATION;
            m_btnQuickSlamManual.setText(localization ? "Localization" : "Mapping");
            setButtonState(m_btnQuickSlamManual, localization, pending, "#1565C0");
            if (autoEnabled && !pending) {
                m_btnQuickSlamManual.setEnabled(false);
                m_btnQuickSlamManual.setAlpha(0.2f);
            }
        }
    }

    private void updateMapButtons()
    {
        if (m_btnMapZoom != null && m_map3dView != null) {
            m_btnMapZoom.setImageResource(m_map3dView.isZoomedIn() ? android.R.drawable.ic_menu_zoom
                                                                   : android.R.drawable.ic_menu_search);
        }
    }

    private void updateStreamToggleButtons()
    {
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

    private void updateFeatureToggleButton()
    {
        m_showFeaturePoints = m_sendFeature;
        updateStreamToggleButtons();
    }

    private void updateDebugPanelVisibility()
    {
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

    private void updateRemoteControlsVisibility()
    {
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

    private void resetRemoteInputs()
    {
        m_leftUpPressed = false;
        m_leftDownPressed = false;
        m_leftYawLeftPressed = false;
        m_leftYawRightPressed = false;
        m_rightForwardPressed = false;
        m_rightBackPressed = false;
        m_rightLeftPressed = false;
        m_rightRightPressed = false;
        releaseDirectionalButton(m_btnLeftUp);
        releaseDirectionalButton(m_btnLeftDown);
        releaseDirectionalButton(m_btnLeftYawLeft);
        releaseDirectionalButton(m_btnLeftYawRight);
        releaseDirectionalButton(m_btnRightForward);
        releaseDirectionalButton(m_btnRightBack);
        releaseDirectionalButton(m_btnRightLeft);
        releaseDirectionalButton(m_btnRightRight);
        m_lastJoystickActive = false;
    }

    private interface DirectionButtonHandler {
        void onStateChanged(boolean pressed);
    }

    private void bindDirectionalButton(View button, DirectionButtonHandler handler)
    {
        if (button == null) {
            return;
        }
        button.setAlpha(0.76f);
        button.setOnTouchListener((v, event) -> {
            switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_POINTER_DOWN:
            case MotionEvent.ACTION_MOVE:
                v.setPressed(true);
                v.setAlpha(1.0f);
                v.setScaleX(1.24f);
                v.setScaleY(1.24f);
                if (v instanceof ImageButton) {
                    ((ImageButton)v).setColorFilter(Color.parseColor("#FFF1A8"));
                }
                handler.onStateChanged(true);
                return true;
            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_POINTER_UP:
                v.setPressed(false);
                v.setAlpha(0.76f);
                v.setScaleX(1.0f);
                v.setScaleY(1.0f);
                if (v instanceof ImageButton) {
                    ((ImageButton)v).clearColorFilter();
                }
                handler.onStateChanged(false);
                v.performClick();
                return true;
            case MotionEvent.ACTION_CANCEL:
                v.setPressed(false);
                v.setAlpha(0.76f);
                v.setScaleX(1.0f);
                v.setScaleY(1.0f);
                if (v instanceof ImageButton) {
                    ((ImageButton)v).clearColorFilter();
                }
                handler.onStateChanged(false);
                return true;
            default:
                return false;
            }
        });
    }

    private void releaseDirectionalButton(View button)
    {
        if (button != null) {
            button.setPressed(false);
            button.setAlpha(0.76f);
            button.setScaleX(1.0f);
            button.setScaleY(1.0f);
            if (button instanceof ImageButton) {
                ((ImageButton)button).clearColorFilter();
            }
        }
    }

    private static float axisFromButtons(boolean positivePressed, boolean negativePressed)
    {
        if (positivePressed == negativePressed) {
            return 0f;
        }
        return positivePressed ? BUTTON_AXIS_MAGNITUDE : -BUTTON_AXIS_MAGNITUDE;
    }

    private void refreshVideoFrames()
    {
        renderVideoFrame(0);
        renderVideoFrame(1);
    }

    private boolean isRuntimeActive() { return m_runtimeMode == MODE_SLAM || m_runtimeMode == MODE_CALIB; }

    private String effectiveConfigLabel(String label, boolean appliesAfterRestart)
    {
        if (appliesAfterRestart && isRuntimeActive()) {
            return label + " (applies after stop/start)";
        }
        return label;
    }

    private void sendCurrentRuntimeConfig(String label, boolean appliesAfterRestart, String pendingKey,
                                          AckSuccess onSuccess)
    {
        sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                  m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure,
                                  effectiveConfigLabel(label, appliesAfterRestart), pendingKey, onSuccess);
    }

    private static float clamp01(float v) { return Math.max(0f, Math.min(1f, v)); }

    private void sendSimpleCmd(String name, int cmd)
    {
        try {
            int seq = NativeUdp.sendCmd(cmd);
            m_tvStatus.setText(name + " sent seq=" + seq + " cmd=0x" + Integer.toHexString(cmd));
        } catch (Throwable t) {
            m_tvStatus.setText(name + " error: " + t.getMessage());
        }
    }

    private void sendSimpleCmdAwaitAck(String name, int cmd, String pendingKey, AckSuccess onSuccess)
    {
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

    private void sendHoldBurst(int count, String reason)
    {
        for (int i = 0; i < count; ++i) {
            sendSimpleCmd(reason + "[" + (i + 1) + "/" + count + "]", CMD_HOLD);
        }
    }

    private void requestRemoteControlModeIfNeeded()
    {
        if ("REMOTE".equals(m_lastFlightCommand) || isPending(PENDING_POSITION)) {
            return;
        }
        // Button-based RC control maps to MANUAL_CONTROL stream. POSCTL is the
        // expected companion mode for that path; OFFBOARD uses setpoint stream.
        sendSimpleCmdAwaitAck("REMOTE", CMD_POSITION, PENDING_POSITION, () -> {
            m_lastFlightCommand = "REMOTE";
            updateFlightButtons();
        });
    }

    private void sendMoveRcJoystickCommand(float throttle, float yaw, float pitch, float roll, float maxV,
                                           String reason)
    {
        try {
            int seq = NativeUdp.sendMoveRcJoystick(FRAME_NED, throttle, yaw, pitch, roll, maxV);
            m_tvStatus.setText(String.format(Locale.US,
                                             "%s seq=%d REMOTE thr=%.2f yaw=%.2f pitch=%.2f roll=%.2f maxV=%.2f",
                                             reason, seq, throttle, yaw, pitch, roll, maxV));
        } catch (Throwable t) {
            m_tvStatus.setText(reason + " error: " + t.getMessage());
        }
    }

    private int sendRuntimeConfig(int exposureUs, float gain, int pairMs, int slamFps, int slamMode, int sensorMode,
                                  int featureFrontend, boolean sendImage, boolean sendFeature, boolean sendMap,
                                  boolean autoExposure, boolean useCustomTbc, float tbcTx, float tbcTy, float tbcTz,
                                  float tbcRollDeg, float tbcPitchDeg, float tbcYawDeg, int orbNFeatures,
                                  float orbScaleFactor, int orbNLevels, int orbIniThFast, int orbMinThFast,
                                  int xfeatTopK, int xfeatMaxPoints, int xfeatInputMaxWidth, int xfeatInputMaxHeight,
                                  boolean lkXFeatSeeding, int lkPerFrameAcceleration)
    {
        try {
            int seq = NativeUdp.sendRuntimeConfig(exposureUs, gain, pairMs, slamFps, slamMode, sensorMode, sendImage,
                                                  sendFeature, sendMap, autoExposure, useCustomTbc, tbcTx, tbcTy,
                                                  tbcTz, tbcRollDeg, tbcPitchDeg, tbcYawDeg, orbNFeatures,
                                                  orbScaleFactor, orbNLevels, orbIniThFast, orbMinThFast,
                                                  featureFrontend, xfeatTopK, xfeatMaxPoints, xfeatInputMaxWidth,
                                                  xfeatInputMaxHeight, lkXFeatSeeding, lkPerFrameAcceleration);
            m_tvStatus.setText(String.format(
                Locale.US,
                "CFG seq=%d exp=%d gain=%.1f pair=%dms slam=%dfps mode=%s sensor=%s frontend=%s img=%s feat=%s map=%s ae=%s tbc=%s orb=%d/%.2f/%d/%d/%d xfeat=%d/%d/%d/%d lkSeed=%s lkAccel=%s",
                seq, exposureUs, gain, pairMs, slamFps, slamModeToText(slamMode), sensorModeToText(sensorMode),
                runtimeFeatureFrontendToText(featureFrontend, lkXFeatSeeding, lkPerFrameAcceleration),
                sendImage ? "on" : "off", sendFeature ? "on" : "off",
                sendMap ? "on" : "off", autoExposure ? "on" : "off",
                useCustomTbc
                    ? String.format(Locale.US, "on(%.3f,%.3f,%.3f,r%.1f,p%.1f,y%.1f)", tbcTx, tbcTy, tbcTz,
                                    tbcRollDeg, tbcPitchDeg, tbcYawDeg)
                    : "off",
                orbNFeatures, orbScaleFactor, orbNLevels, orbIniThFast, orbMinThFast, xfeatTopK, xfeatMaxPoints,
                xfeatInputMaxWidth, xfeatInputMaxHeight, lkXFeatSeeding ? "XFeat" : "GFTT",
                lkPerFrameAccelerationToText(lkPerFrameAcceleration)));
            return seq;
        } catch (Throwable t) {
            m_tvStatus.setText("CFG error: " + t.getMessage());
            return -1;
        }
    }

    private int sendRuntimeConfig()
    {
        return sendRuntimeConfig(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                 m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap,
                                 m_cfgAutoExposure,
                                 m_cfgUseCustomTbc, m_cfgTbcTx, m_cfgTbcTy, m_cfgTbcTz, m_cfgTbcRollDeg,
                                 m_cfgTbcPitchDeg, m_cfgTbcYawDeg, m_cfgOrbNFeatures, m_cfgOrbScaleFactor,
                                 m_cfgOrbNLevels, m_cfgOrbIniThFast, m_cfgOrbMinThFast, m_cfgXFeatTopK,
                                 m_cfgXFeatMaxPoints, m_cfgXFeatInputMaxWidth, m_cfgXFeatInputMaxHeight,
                                 m_cfgLkXFeatSeeding, m_cfgLkPerFrameAcceleration);
    }

    private void sendRuntimeConfigAwaitAck(int exposureUs, float gain, int pairMs, int slamFps, int slamMode,
                                           int sensorMode, int featureFrontend, boolean sendImage, boolean sendFeature, boolean sendMap,
                                           boolean autoExposure,
                                           String label, String pendingKey, AckSuccess onSuccess)
    {
        sendRuntimeConfigAwaitAck(exposureUs, gain, pairMs, slamFps, slamMode, sensorMode, featureFrontend, sendImage, sendFeature,
                                  sendMap, autoExposure, m_cfgUseCustomTbc, m_cfgTbcTx, m_cfgTbcTy, m_cfgTbcTz,
                                  m_cfgTbcRollDeg, m_cfgTbcPitchDeg, m_cfgTbcYawDeg, label, pendingKey, onSuccess);
    }

    private void sendRuntimeConfigAwaitAck(int exposureUs, float gain, int pairMs, int slamFps, int slamMode,
                                           int sensorMode, int featureFrontend, boolean sendImage, boolean sendFeature, boolean sendMap,
                                           boolean autoExposure, boolean useCustomTbc, float tbcTx, float tbcTy,
                                           float tbcTz, float tbcRollDeg, float tbcPitchDeg, float tbcYawDeg,
                                           String label, String pendingKey, AckSuccess onSuccess)
    {
        if (!ensureVehicleConnection()) {
            return;
        }
        if (isPending(pendingKey)) {
            return;
        }
        int seq = sendRuntimeConfig(exposureUs, gain, pairMs, slamFps, slamMode, sensorMode, featureFrontend, sendImage, sendFeature,
                                    sendMap, autoExposure, useCustomTbc, tbcTx, tbcTy, tbcTz, tbcRollDeg, tbcPitchDeg,
                                    tbcYawDeg, m_cfgOrbNFeatures, m_cfgOrbScaleFactor, m_cfgOrbNLevels,
                                    m_cfgOrbIniThFast, m_cfgOrbMinThFast, m_cfgXFeatTopK, m_cfgXFeatMaxPoints,
                                    m_cfgXFeatInputMaxWidth, m_cfgXFeatInputMaxHeight, m_cfgLkXFeatSeeding,
                                    m_cfgLkPerFrameAcceleration);
        if (seq < 0) {
            return;
        }
        registerPendingAck(seq, CMD_RUNTIME_CONFIG, label, pendingKey, onSuccess);
    }

    private void setPendingKey(String pendingKey)
    {
        if (pendingKey == null || pendingKey.isEmpty()) {
            return;
        }
        m_pendingUiKeys.add(pendingKey);
        updateRuntimeButtons();
        updateFlightButtons();
    }

    private boolean ensureVehicleConnection()
    {
        String vehicleIp = m_etVehicleIp.getText().toString().trim();
        if (vehicleIp.isEmpty()) {
            m_tvStatus.setText("Vehicle IP is empty");
            return false;
        }
        if (vehicleIp.equals(m_vehicleIp) && m_udpReady) {
            return true;
        }
        return reconnectVehicle(vehicleIp, m_vehicleCmdPort, m_phoneVideoPort, true);
    }

    private boolean reconnectVehicle(String vehicleIp, int cmdPort, int videoPort, boolean updateStatus)
    {
        try {
            NativeUdp.close();
            m_udpReady = false;
            boolean ok = NativeUdp.init(vehicleIp, cmdPort, videoPort);
            if (!ok) {
                if (updateStatus) {
                    m_tvStatus.setText("Reconnect failed: " + vehicleIp);
                }
                return false;
            }
            m_vehicleIp = vehicleIp;
            m_vehicleCmdPort = cmdPort;
            m_phoneVideoPort = videoPort;
            m_udpReady = true;
            m_lastVehicleHeartbeatMs = 0L;
            m_vehicleHeartbeatTimeoutHandled = false;
            try {
                NativeUdp.sendGetCapabilities();
                NativeUdp.sendGetConfig();
            } catch (Throwable ignored) {
            }
            if (updateStatus) {
                m_tvStatus.setText("UDP ready cmd-> " + vehicleIp + ":" + cmdPort + " video<-" + videoPort);
            }
            return true;
        } catch (Throwable t) {
            if (updateStatus) {
                m_tvStatus.setText("Reconnect error: " + t.getMessage());
            }
            return false;
        }
    }

    private void applyDiscoveredVehicle(DiscoveryAnnouncement announcement)
    {
        if (announcement == null || announcement.vehicleIp == null) {
            return;
        }
        final String discoveredIp = announcement.vehicleIp.trim();
        if (discoveredIp.isEmpty()) {
            return;
        }
        final boolean changed = !discoveredIp.equals(m_vehicleIp) || !m_udpReady || announcement.cmdPort != m_vehicleCmdPort ||
                                announcement.videoPort != m_phoneVideoPort;
        m_lastDiscoveredVehicleIp = discoveredIp;
        if (m_etVehicleIp != null && !discoveredIp.equals(m_etVehicleIp.getText().toString().trim())) {
            m_etVehicleIp.setText(discoveredIp);
        }
        if (!changed) {
            return;
        }
        if (reconnectVehicle(discoveredIp, announcement.cmdPort, announcement.videoPort, false)) {
            m_tvStatus.setText(
                "CM5 discovered " + discoveredIp + ", UDP ready cmd-> " + discoveredIp + ":" + announcement.cmdPort +
                " video<-" + announcement.videoPort);
            requestRuntimeMetadata();
        } else {
            m_tvStatus.setText("CM5 discovered " + discoveredIp + ", reconnect failed");
        }
    }

    private void startDiscoveryLoop()
    {
        if (m_discoveryLoopRunning) {
            return;
        }
        m_discoveryLoopRunning = true;
        m_discoveryThread = new Thread(() -> {
            DatagramSocket socket = null;
            try {
                socket = new DatagramSocket(null);
                socket.setReuseAddress(true);
                socket.setBroadcast(true);
                socket.bind(new InetSocketAddress(DISCOVERY_PORT));
                socket.setSoTimeout(DISCOVERY_SOCKET_TIMEOUT_MS);
                byte[] buf = new byte[512];
                while (m_discoveryLoopRunning) {
                    DatagramPacket packet = new DatagramPacket(buf, buf.length);
                    try {
                        socket.receive(packet);
                    } catch (SocketTimeoutException timeout) {
                        continue;
                    }
                    DiscoveryAnnouncement announcement =
                        parseDiscoveryAnnouncement(packet.getData(), packet.getLength(), packet.getAddress());
                    if (announcement == null) {
                        continue;
                    }
                    m_handler.post(() -> applyDiscoveredVehicle(announcement));
                }
            } catch (Throwable t) {
                final String message = t.getMessage() != null ? t.getMessage() : t.getClass().getSimpleName();
                m_handler.post(() -> {
                    if (m_tvStatus != null) {
                        m_tvStatus.setText("Discovery error: " + message);
                    }
                });
            } finally {
                if (socket != null) {
                    socket.close();
                }
            }
        }, "cm5-discovery");
        m_discoveryThread.start();
    }

    private void stopDiscoveryLoop()
    {
        m_discoveryLoopRunning = false;
        if (m_discoveryThread != null) {
            try {
                m_discoveryThread.join(1500L);
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            }
            m_discoveryThread = null;
        }
    }

    private void setButtonState(Button button, boolean active, boolean pending, String color)
    {
        if (button == null) {
            return;
        }
        button.setBackgroundColor(Color.parseColor(color));
        button.setAlpha(pending ? 0.2f : (active ? 1.0f : 0.35f));
        button.setTextColor(Color.WHITE);
        button.setEnabled(!pending);
    }

    private void updateRuntimeButtons()
    {
        boolean sensorPending = isPending(PENDING_SENSOR) || isPending(PENDING_RUNTIME);
        boolean runtimePending = isPending(PENDING_RUNTIME);
        boolean runtimeActive = isRuntimeActive();
        m_updatingToggleUi = true;
        updateSensorModeSpinner();
        updateFeatureFrontendSpinner();
        if (m_spinnerSensorMode != null) {
            boolean enabled = !runtimeActive && !sensorPending && m_availableSensorModes.length > 0;
            m_spinnerSensorMode.setEnabled(enabled);
            m_spinnerSensorMode.setAlpha(enabled ? 1.0f : 0.35f);
        }
        if (m_spinnerFeatureFrontend != null) {
            boolean enabled = !runtimeActive && !sensorPending && getSupportedFeatureFrontends().length > 1;
            m_spinnerFeatureFrontend.setEnabled(enabled);
            m_spinnerFeatureFrontend.setAlpha(enabled ? 1.0f : 0.35f);
        }
        if (m_btnToggleSlam != null) {
            m_btnToggleSlam.setChecked(m_runtimeMode == MODE_SLAM);
            m_btnToggleSlam.setEnabled(!runtimePending);
            m_btnToggleSlam.setAlpha(runtimePending ? 0.35f : 1.0f);
        }
        if (m_btnToggleCalib != null) {
            m_btnToggleCalib.setChecked(m_runtimeMode == MODE_CALIB);
            boolean enabled = !runtimePending && m_supportsCalib;
            m_btnToggleCalib.setEnabled(enabled);
            m_btnToggleCalib.setAlpha(enabled ? 1.0f : 0.35f);
        }
        m_updatingToggleUi = false;
        if (m_btnCleanCalib != null) {
            setButtonState(m_btnCleanCalib, true, isPending(PENDING_CLEAN_CALIB), "#546E7A");
        }
        updateConfigViews();
    }

    private void updateFeatureFrontendSpinner()
    {
        if (m_spinnerFeatureFrontend == null) {
            return;
        }
        final int[] supportedFrontends = getSupportedFeatureFrontends();
        if (m_spinnerFeatureFrontend.getAdapter() == null || m_spinnerFeatureFrontend.getCount() != supportedFrontends.length) {
            String[] labels = new String[supportedFrontends.length];
            for (int i = 0; i < supportedFrontends.length; ++i) {
                labels[i] = featureFrontendOptionToText(supportedFrontends[i]);
            }
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, labels);
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            m_spinnerFeatureFrontend.setAdapter(adapter);
        }
        int targetIndex = 0;
        for (int i = 0; i < supportedFrontends.length; ++i) {
            if (supportedFrontends[i] == currentFeatureFrontendOption()) {
                targetIndex = i;
                break;
            }
        }
        if (m_spinnerFeatureFrontend.getSelectedItemPosition() != targetIndex) {
            m_spinnerFeatureFrontend.setSelection(targetIndex, false);
        }
    }

    private int[] getSupportedFeatureFrontends()
    {
        if (m_supportsLK) {
            if (m_supportsXFeat) {
                return new int[] {FEATURE_FRONTEND_ORB, FEATURE_FRONTEND_LK, FEATURE_FRONTEND_LK_GFTT_PER_FRAME,
                                  FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI, FEATURE_FRONTEND_LK_XFEAT};
            }
            return new int[] {FEATURE_FRONTEND_ORB, FEATURE_FRONTEND_LK, FEATURE_FRONTEND_LK_GFTT_PER_FRAME,
                              FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI};
        }
        return new int[] {FEATURE_FRONTEND_ORB};
    }

    private int currentFeatureFrontendOption()
    {
        if (m_cfgFeatureFrontend == FEATURE_FRONTEND_LK && m_cfgLkXFeatSeeding) {
            return FEATURE_FRONTEND_LK_XFEAT;
        }
        if (m_cfgFeatureFrontend == FEATURE_FRONTEND_LK_GFTT_PER_FRAME &&
            m_cfgLkPerFrameAcceleration == LK_PER_FRAME_ACCEL_VPI_CUDA) {
            return FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI;
        }
        return m_cfgFeatureFrontend;
    }

    private void updateSensorModeSpinner()
    {
        if (m_spinnerSensorMode == null) {
            return;
        }

        int[] supportedModes = getSupportedSensorModes();
        boolean changed = m_availableSensorModes.length != supportedModes.length;
        if (!changed) {
            for (int i = 0; i < supportedModes.length; ++i) {
                if (m_availableSensorModes[i] != supportedModes[i]) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed || m_spinnerSensorMode.getAdapter() == null) {
            m_availableSensorModes = supportedModes;
            String[] labels = new String[m_availableSensorModes.length];
            for (int i = 0; i < m_availableSensorModes.length; ++i) {
                labels[i] = sensorModeToText(m_availableSensorModes[i]);
            }
            ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, labels);
            adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            m_spinnerSensorMode.setAdapter(adapter);
        } else {
            m_availableSensorModes = supportedModes;
        }

        int targetIndex = findSensorModeIndex(m_sensorMode);
        if (targetIndex < 0) {
            targetIndex = 0;
        }
        if (m_spinnerSensorMode.getSelectedItemPosition() != targetIndex) {
            m_spinnerSensorMode.setSelection(targetIndex, false);
        }
    }

    private int[] getSupportedSensorModes()
    {
        int[] scratch = new int[4];
        int count = 0;
        scratch[count++] = SENSOR_STEREO;
        if (m_supportsStereoImu) {
            scratch[count++] = SENSOR_STEREO_IMU;
        }
        if (m_supportsMono) {
            scratch[count++] = SENSOR_MONO;
        }
        if (m_supportsMonoImu) {
            scratch[count++] = SENSOR_MONO_IMU;
        }
        return Arrays.copyOf(scratch, count);
    }

    private int findSensorModeIndex(int sensorMode)
    {
        for (int i = 0; i < m_availableSensorModes.length; ++i) {
            if (m_availableSensorModes[i] == sensorMode) {
                return i;
            }
        }
        return -1;
    }

    private void updateFlightButtons()
    {
        if (m_btnArmToggle != null) {
            m_btnArmToggle.setText(m_armLatched ? "DISARM" : "ARM");
            setButtonState(m_btnArmToggle, true, isPending(PENDING_ARM), "#C62828");
        }
        if (m_btnEmergencyStop != null) {
            setButtonState(m_btnEmergencyStop, "EMERGENCY_STOP".equals(m_lastFlightCommand),
                           isPending(PENDING_EMERGENCY_STOP), "#B71C1C");
        }
        if (m_btnLand != null) {
            final boolean autoMode = isPx4AutoMode();
            final boolean positionMode = isPx4PositionMode();
            m_btnLand.setText(autoMode ? "POSITION" : "LAND");
            setButtonState(m_btnLand, autoMode || positionMode, isPending(PENDING_LAND) || isPending(PENDING_POSITION),
                           "#EF6C00");
        }
    }

    private void sendRuntimeMode(int mode, String label)
    {
        if (!ensureVehicleConnection()) {
            return;
        }
        if (isPending(PENDING_RUNTIME)) {
            return;
        }
        final int exposureUs = m_cfgExposureUs;
        final float gain = (float)m_cfgGain;
        final int pairMs = m_cfgPairMs;
        final int slamFps = m_cfgSlamFps;
        final int slamMode = m_cfgSlamMode;
        final int sensorMode = m_sensorMode;
        sendRuntimeConfigAwaitAck(exposureUs, gain, pairMs, slamFps, slamMode, sensorMode, m_cfgFeatureFrontend,
                                  m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure, label + " CFG",
                                  PENDING_RUNTIME, () -> {
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

    private void stopRuntime(String label)
    {
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

    private void requestRuntimeMetadata()
    {
        if (!ensureVehicleConnection()) {
            return;
        }
        try {
            NativeUdp.sendGetCapabilities();
            NativeUdp.sendGetConfig();
        } catch (Throwable t) {
            m_tvStatus.setText("Query error: " + t.getMessage());
        }
    }

    private void registerPendingAck(long seq, int command, String label, String pendingKey, AckSuccess onSuccess)
    {
        setPendingKey(pendingKey);
        Runnable timeoutRunnable = () ->
        {
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

    private boolean isPending(String pendingKey) { return pendingKey != null && m_pendingUiKeys.contains(pendingKey); }

    private void clearPendingKey(String pendingKey)
    {
        if (pendingKey == null || pendingKey.isEmpty()) {
            return;
        }
        m_pendingUiKeys.remove(pendingKey);
        updateRuntimeButtons();
        updateFlightButtons();
    }

    private static int readU16Le(byte[] data, int offset)
    {
        return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8);
    }

    private static long readU32Le(byte[] data, int offset)
    {
        return ((long)data[offset] & 0xFFL) | (((long)data[offset + 1] & 0xFFL) << 8) |
            (((long)data[offset + 2] & 0xFFL) << 16) | (((long)data[offset + 3] & 0xFFL) << 24);
    }

    private static int readI16Le(byte[] data, int offset)
    {
        int v = readU16Le(data, offset);
        return (v >= 0x8000) ? (v - 0x10000) : v;
    }

    private static int readI32Le(byte[] data, int offset)
    {
        return (data[offset] & 0xFF) | ((data[offset + 1] & 0xFF) << 8) | ((data[offset + 2] & 0xFF) << 16) |
            ((data[offset + 3] & 0xFF) << 24);
    }

    private static long readI64Le(byte[] data, int offset)
    {
        return ((long)data[offset] & 0xFFL) | (((long)data[offset + 1] & 0xFFL) << 8) |
            (((long)data[offset + 2] & 0xFFL) << 16) | (((long)data[offset + 3] & 0xFFL) << 24) |
            (((long)data[offset + 4] & 0xFFL) << 32) | (((long)data[offset + 5] & 0xFFL) << 40) |
            (((long)data[offset + 6] & 0xFFL) << 48) | (((long)data[offset + 7] & 0xFFL) << 56);
    }

    private static float readF32Le(byte[] data, int offset) { return Float.intBitsToFloat(readI32Le(data, offset)); }

    private static double readF64Le(byte[] data, int offset)
    {
        return Double.longBitsToDouble(readI64Le(data, offset));
    }

    private static String ackStatusToText(int status)
    {
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

    private static Map<String, String> parseKeyValueText(String text)
    {
        Map<String, String> out = new HashMap<>();
        if (text == null || text.isEmpty()) {
            return out;
        }
        String[] lines = text.split("\\r?\\n");
        for (String line : lines) {
            if (line == null) {
                continue;
            }
            int pos = line.indexOf('=');
            if (pos <= 0 || pos >= line.length() - 1) {
                continue;
            }
            String key = line.substring(0, pos).trim();
            String value = line.substring(pos + 1).trim();
            if (!key.isEmpty()) {
                out.put(key, value);
            }
        }
        return out;
    }

    private static String readTlvTextPayload(byte[] rx, int expectedCmd)
    {
        if (!isTlvPacket(rx) || rx.length < 17) {
            return null;
        }
        int cmd = rx[3] & 0xFF;
        if (cmd != expectedCmd) {
            return null;
        }
        int len = readU16Le(rx, 5);
        int total = 2 + (1 + 1 + 1 + 2 + 4 + 4) + len + 2;
        if (rx.length < total) {
            return null;
        }
        if (len <= 0) {
            return "";
        }
        return new String(rx, 15, len);
    }

    private static boolean parseBooleanText(String value, boolean defaultValue)
    {
        if (value == null) {
            return defaultValue;
        }
        if ("true".equalsIgnoreCase(value) || "on".equalsIgnoreCase(value) || "1".equals(value)) {
            return true;
        }
        if ("false".equalsIgnoreCase(value) || "off".equalsIgnoreCase(value) || "0".equals(value)) {
            return false;
        }
        return defaultValue;
    }

    private static int parseRuntimeModeText(String value, int defaultValue)
    {
        if (value == null) {
            return defaultValue;
        }
        switch (value.trim().toLowerCase(Locale.US)) {
        case "slam":
            return MODE_SLAM;
        case "calib":
            return MODE_CALIB;
        case "idle":
        default:
            return MODE_IDLE;
        }
    }

    private static int parseSensorModeText(String value, int defaultValue)
    {
        if (value == null) {
            return defaultValue;
        }
        String normalized = value.trim().toLowerCase(Locale.US);
        if ("mono-imu".equals(normalized)) {
            return SENSOR_MONO_IMU;
        }
        if ("mono".equals(normalized)) {
            return SENSOR_MONO;
        }
        if ("stereo-imu".equals(normalized)) {
            return SENSOR_STEREO_IMU;
        }
        if ("stereo".equals(normalized)) {
            return SENSOR_STEREO;
        }
        return defaultValue;
    }

    private static int parseFeatureFrontendText(String value, int defaultValue)
    {
        if (value == null) {
            return defaultValue;
        }
        String normalized = value.trim().toLowerCase(Locale.US);
        if ("droid".equals(normalized) || "droid-light".equals(normalized) || "droid_light".equals(normalized) ||
            "droidlight".equals(normalized)) {
            return FEATURE_FRONTEND_DROID_LIGHT;
        }
        if ("lk-gftt-per-frame".equals(normalized) || "lk_gftt_per_frame".equals(normalized) ||
            "lk-gftt-every-frame".equals(normalized) || "lk_gftt_every_frame".equals(normalized) ||
            "per-frame-gftt".equals(normalized) || "per_frame_gftt".equals(normalized)) {
            return FEATURE_FRONTEND_LK_GFTT_PER_FRAME;
        }
        if ("lk".equals(normalized) || "klt".equals(normalized) || "stereo-lk".equals(normalized) ||
            "stereo_lk".equals(normalized) || "optical-flow".equals(normalized) || "optical_flow".equals(normalized)) {
            return FEATURE_FRONTEND_LK;
        }
        if ("orb".equals(normalized)) {
            return FEATURE_FRONTEND_ORB;
        }
        return defaultValue;
    }

    private static int parseLkPerFrameAccelerationText(String value, int defaultValue)
    {
        if (value == null) {
            return defaultValue;
        }
        String normalized = value.trim().toLowerCase(Locale.US);
        if ("vpi-cuda".equals(normalized) || "vpi_cuda".equals(normalized) || "vpi".equals(normalized) ||
            "gpu".equals(normalized)) {
            return LK_PER_FRAME_ACCEL_VPI_CUDA;
        }
        if ("cpu".equals(normalized) || "off".equals(normalized)) {
            return LK_PER_FRAME_ACCEL_CPU;
        }
        return defaultValue;
    }

    private static int parseSlamModeText(String value, int defaultValue)
    {
        if (value == null) {
            return defaultValue;
        }
        String normalized = value.trim().toLowerCase(Locale.US);
        if ("localization".equals(normalized) || "localisation".equals(normalized)) {
            return SLAM_MODE_LOCALIZATION;
        }
        if ("auto".equals(normalized)) {
            return SLAM_MODE_AUTO;
        }
        if ("mapping".equals(normalized)) {
            return SLAM_MODE_MAPPING;
        }
        return defaultValue;
    }

    private static boolean containsTokenList(String csv, String token)
    {
        if (csv == null || token == null) {
            return false;
        }
        String[] parts = csv.split(",");
        for (String part : parts) {
            if (token.equalsIgnoreCase(part.trim())) {
                return true;
            }
        }
        return false;
    }

    private String decodeTlvAck(byte[] rx)
    {
        AckFrame ack = parseAckFrame(rx);
        if (ack.valid) {
            return String.format(Locale.US, "ACK reqSeq=%d ackCmd=0x%02X ackSeq=%d status=%s", ack.reqSeq, ack.ackCmd,
                                 ack.ackSeq, ackStatusToText(ack.status));
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
            return String.format(Locale.US, "RX partial ver=%d cmd=0x%02X len=%d bytes=%d need=%d", ver, cmd, len,
                                 rx.length, total);
        }
        return String.format(Locale.US, "RX TLV ver=%d cmd=0x%02X flags=%d len=%d seq=%d tMs=%d", ver, cmd, flags, len,
                             seq, tMs);
    }

    private AckFrame parseAckFrame(byte[] rx)
    {
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

    private boolean tryHandleAckPacket(byte[] rx)
    {
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
            m_tvStatus.setText(
                String.format(Locale.US, "%s ack=%s seq=%d", pending.label, ackStatusToText(ack.status), ack.ackSeq));
        } else {
            m_tvStatus.setText(decodeTlvAck(rx));
        }
        return true;
    }

    private boolean tryHandleCapabilitiesPacket(byte[] rx)
    {
        String payloadText = readTlvTextPayload(rx, CMD_CAPABILITIES);
        if (payloadText == null) {
            return false;
        }
        m_lastCapabilitiesText = payloadText;
        Map<String, String> values = parseKeyValueText(payloadText);
        final String cameraProviders = values.get("camera_providers");
        final String behaviorNotes = values.get("behavior_notes");
        m_supportsCalib = containsTokenList(values.get("runtime_modes"), "calib");
        m_supportsStereoImu = containsTokenList(values.get("perception_modes"), "stereo-imu");
        m_supportsMono = containsTokenList(values.get("perception_modes"), "mono");
        m_supportsMonoImu = containsTokenList(values.get("perception_modes"), "mono-imu");
        final String xfeatSeedCapability = behaviorNoteValue(behaviorNotes, "slam.lk_seed.xfeat=");
        m_supportsXFeat = xfeatSeedCapability != null && !"disabled_at_build_time".equalsIgnoreCase(xfeatSeedCapability);
        final String droidLightCapability = behaviorNoteValue(behaviorNotes, "slam.feature_frontend.droid_light=");
        m_supportsDroidLight = droidLightCapability != null && !"disabled_at_build_time".equalsIgnoreCase(droidLightCapability);
        final String lkCapability = behaviorNoteValue(behaviorNotes, "slam.feature_frontend.lk=");
        m_supportsLK = lkCapability != null && !"disabled_at_build_time".equalsIgnoreCase(lkCapability);
        m_isPackedStereoUvc = containsTokenList(cameraProviders, "uvc_stereo_opencv") &&
                              containsBehaviorNote(behaviorNotes, "camera.uvc_pairing=not_required_single_capture_provides_both_eyes");
        m_pairWindowRequired = !m_isPackedStereoUvc;
        updateRuntimeButtons();
        m_tvStatus.setText(String.format(Locale.US,
                                         "Capabilities synced calib=%s stereo_imu=%s mono=%s mono_imu=%s uvc=%s lk=%s xfeat_seed=%s",
                                         m_supportsCalib ? "yes" : "no", m_supportsStereoImu ? "yes" : "no",
                                         m_supportsMono ? "yes" : "no", m_supportsMonoImu ? "yes" : "no",
                                         m_isPackedStereoUvc ? "packed" : "no", m_supportsLK ? "yes" : "no",
                                         m_supportsXFeat ? "yes" : "no"));
        return true;
    }

    private boolean tryHandleConfigPacket(byte[] rx)
    {
        String payloadText = readTlvTextPayload(rx, CMD_CONFIG);
        if (payloadText == null) {
            return false;
        }
        m_lastConfigText = payloadText;
        Map<String, String> values = parseKeyValueText(payloadText);

        m_runtimeMode = parseRuntimeModeText(values.get("runtime.mode"), m_runtimeMode);
        m_cfgExposureUs = quantizeExposureUs(parseI(values.get("camera.exposure_us"), m_cfgExposureUs));
        m_cfgGain = quantizeGain(parseI(values.get("camera.gain"), m_cfgGain));
        m_cfgPairMs = quantizePairMs(parseI(values.get("camera.pair_window_ms"), m_cfgPairMs));
        m_cfgUvcPackedStereo = parseBooleanText(values.get("camera.uvc_packed_stereo"), m_cfgUvcPackedStereo);
        m_pairWindowRequired = !m_cfgUvcPackedStereo;
        m_cfgAutoExposure = parseBooleanText(values.get("camera.auto_exposure"), m_cfgAutoExposure);
        int parsedSlamFps = parseI(values.get("slam.input_fps"), m_cfgSlamFps);
        if (parsedSlamFps <= 0) {
            parsedSlamFps = SLAM_FPS_DEFAULT;
        }
        m_cfgSlamFps = quantizeSlamFps(parsedSlamFps);
        m_cfgSlamMode = parseSlamModeText(values.get("slam.operation_mode"), m_cfgSlamMode);
        m_cfgFeatureFrontend = parseFeatureFrontendText(values.get("slam.feature_frontend"), m_cfgFeatureFrontend);
        if (m_cfgFeatureFrontend == FEATURE_FRONTEND_XFEAT) {
            m_cfgFeatureFrontend = FEATURE_FRONTEND_ORB;
        }
        if (!m_supportsDroidLight && m_cfgFeatureFrontend == FEATURE_FRONTEND_DROID_LIGHT) {
            m_cfgFeatureFrontend = FEATURE_FRONTEND_ORB;
        }
        if (!m_supportsLK &&
            (m_cfgFeatureFrontend == FEATURE_FRONTEND_LK ||
             m_cfgFeatureFrontend == FEATURE_FRONTEND_LK_GFTT_PER_FRAME)) {
            m_cfgFeatureFrontend = FEATURE_FRONTEND_ORB;
        }
        m_cfgLkXFeatSeeding = parseBooleanText(values.get("slam.lk_xfeat_seeding"), m_cfgLkXFeatSeeding);
        m_cfgLkPerFrameAcceleration =
            parseLkPerFrameAccelerationText(values.get("slam.lk_per_frame_accel"), m_cfgLkPerFrameAcceleration);
        if (m_cfgFeatureFrontend != FEATURE_FRONTEND_LK_GFTT_PER_FRAME) {
            m_cfgLkPerFrameAcceleration = LK_PER_FRAME_ACCEL_CPU;
        }
        m_cfgUseCustomTbc = parseBooleanText(values.get("slam.tbc_override_enabled"), m_cfgUseCustomTbc);
        m_cfgTbcTx = quantizeTbcTranslationM(parseFloatOrDefault(values.get("slam.tbc_tx_m"), m_cfgTbcTx));
        m_cfgTbcTy = quantizeTbcTranslationM(parseFloatOrDefault(values.get("slam.tbc_ty_m"), m_cfgTbcTy));
        m_cfgTbcTz = quantizeTbcTranslationM(parseFloatOrDefault(values.get("slam.tbc_tz_m"), m_cfgTbcTz));
        m_cfgTbcRollDeg = quantizeTbcAngleDeg(parseFloatOrDefault(values.get("slam.tbc_roll_deg"), m_cfgTbcRollDeg),
                                              TBC_ROLL_MIN_TENTH_DEG, TBC_ROLL_MAX_TENTH_DEG);
        m_cfgTbcPitchDeg = quantizeTbcAngleDeg(parseFloatOrDefault(values.get("slam.tbc_pitch_deg"), m_cfgTbcPitchDeg),
                                               TBC_PITCH_MIN_TENTH_DEG, TBC_PITCH_MAX_TENTH_DEG);
        m_cfgTbcYawDeg = quantizeTbcAngleDeg(parseFloatOrDefault(values.get("slam.tbc_yaw_deg"), m_cfgTbcYawDeg),
                                             TBC_YAW_MIN_TENTH_DEG, TBC_YAW_MAX_TENTH_DEG);
        m_cfgOrbNFeatures = quantizeOrbNFeatures(parseI(values.get("slam.orb_nfeatures"), m_cfgOrbNFeatures));
        m_cfgOrbScaleFactor =
            quantizeOrbScaleFactor(parseFloatOrDefault(values.get("slam.orb_scale_factor"), m_cfgOrbScaleFactor));
        m_cfgOrbNLevels = quantizeOrbNLevels(parseI(values.get("slam.orb_nlevels"), m_cfgOrbNLevels));
        m_cfgOrbIniThFast = quantizeOrbFastThreshold(parseI(values.get("slam.orb_ini_th_fast"), m_cfgOrbIniThFast));
        m_cfgOrbMinThFast = quantizeOrbFastThreshold(parseI(values.get("slam.orb_min_th_fast"), m_cfgOrbMinThFast));
        if (m_cfgOrbMinThFast > m_cfgOrbIniThFast) {
            m_cfgOrbMinThFast = m_cfgOrbIniThFast;
        }
        m_cfgXFeatTopK = quantizeXFeatTopK(parseI(values.get("slam.xfeat_top_k"), m_cfgXFeatTopK));
        m_cfgXFeatMaxPoints =
            quantizeXFeatMaxPoints(parseI(values.get("slam.xfeat_max_points"), m_cfgXFeatMaxPoints), m_cfgXFeatTopK);
        m_cfgXFeatInputMaxWidth =
            quantizeXFeatInputMax(parseI(values.get("slam.xfeat_input_max_width"), m_cfgXFeatInputMaxWidth));
        m_cfgXFeatInputMaxHeight =
            quantizeXFeatInputMax(parseI(values.get("slam.xfeat_input_max_height"), m_cfgXFeatInputMaxHeight));
        m_sensorMode = parseSensorModeText(values.get("slam.perception_mode"), m_sensorMode);
        m_sendImage = parseBooleanText(values.get("stream.send_image"), m_sendImage);
        final boolean remoteSendFeature = parseBooleanText(values.get("stream.send_feature"), m_sendFeature);
        m_sendFeature = remoteSendFeature;
        m_sendMap = parseBooleanText(values.get("stream.send_map"), m_sendMap);

        if (!m_featureDefaultEnsured) {
            m_featureDefaultEnsured = true;
            if (!remoteSendFeature) {
                m_sendFeature = true;
                m_showFeaturePoints = true;
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, true, m_sendMap, m_cfgAutoExposure,
                                          "Feature stream default",
                                          PENDING_CONFIG, () -> {
                                              m_sendFeature = true;
                                              m_showFeaturePoints = true;
                                              updateStreamToggleButtons();
                                          });
            }
        }

        updateConfigViews();
        updateRuntimeButtons();
        updateStreamToggleButtons();
        updateFeatureToggleButton();
        m_tvStatus.setText(String.format(Locale.US,
                                         "Config synced mode=%s sensor=%s frontend=%s slam_mode=%s slam=%dfps ae=%s tbc=%s orb=%d/%.2f/%d xfeat=%d/%d/%d/%d",
                                         runtimeModeToText(m_runtimeMode), sensorModeToText(m_sensorMode),
                                         featureFrontendToText(m_cfgFeatureFrontend),
                                         slamModeToText(m_cfgSlamMode), m_cfgSlamFps, m_cfgAutoExposure ? "on" : "off",
                                         m_cfgUseCustomTbc ? "override" : "yaml", m_cfgOrbNFeatures,
                                         m_cfgOrbScaleFactor, m_cfgOrbNLevels, m_cfgXFeatTopK, m_cfgXFeatMaxPoints,
                                         m_cfgXFeatInputMaxWidth, m_cfgXFeatInputMaxHeight));
        return true;
    }

    private static int parseI(String value, int defaultValue)
    {
        try {
            if (value == null || value.trim().isEmpty()) {
                return defaultValue;
            }
            String normalized = value.trim();
            if (normalized.contains(".")) {
                return Math.round(Float.parseFloat(normalized));
            }
            return Integer.parseInt(normalized);
        } catch (Throwable t) {
            return defaultValue;
        }
    }

    private String runtimeModeToText(int mode)
    {
        switch (mode) {
        case MODE_SLAM:
            return "SLAM";
        case MODE_CALIB:
            return "CALIB";
        default:
            return "IDLE";
        }
    }

    private String sensorModeToText(int sensorMode)
    {
        switch (sensorMode) {
        case SENSOR_STEREO_IMU:
            return "Stereo-IMU";
        case SENSOR_MONO:
            return "Mono";
        case SENSOR_MONO_IMU:
            return "Mono-IMU";
        case SENSOR_STEREO:
        default:
            return "Stereo";
        }
    }

    private String featureFrontendToText(int featureFrontend)
    {
        switch (featureFrontend) {
        case FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI:
            return "LK GFTT VPI CUDA";
        case FEATURE_FRONTEND_LK_GFTT_PER_FRAME:
            return m_cfgLkPerFrameAcceleration == LK_PER_FRAME_ACCEL_VPI_CUDA ? "LK GFTT VPI CUDA"
                                                                              : "LK GFTT Per-Frame";
        case FEATURE_FRONTEND_LK:
            return m_cfgLkXFeatSeeding ? "LK + XFeat seed" : "LK GFTT";
        case FEATURE_FRONTEND_DROID_LIGHT:
            return "DROID-Light";
        case FEATURE_FRONTEND_ORB:
        default:
            return "ORB";
        }
    }

    private String featureFrontendOptionToText(int featureFrontend)
    {
        switch (featureFrontend) {
        case FEATURE_FRONTEND_LK_XFEAT:
            return "LK + XFeat seed";
        case FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI:
            return "LK GFTT VPI CUDA";
        case FEATURE_FRONTEND_LK_GFTT_PER_FRAME:
            return "LK GFTT Per-Frame";
        case FEATURE_FRONTEND_LK:
            return "LK GFTT";
        case FEATURE_FRONTEND_DROID_LIGHT:
            return "DROID-Light";
        case FEATURE_FRONTEND_ORB:
        default:
            return "ORB";
        }
    }

    private String runtimeFeatureFrontendToText(int featureFrontend, boolean lkXFeatSeeding,
                                                int lkPerFrameAcceleration)
    {
        if (featureFrontend == FEATURE_FRONTEND_LK) {
            return lkXFeatSeeding ? "LK + XFeat seed" : "LK GFTT";
        }
        if (featureFrontend == FEATURE_FRONTEND_LK_GFTT_PER_FRAME &&
            lkPerFrameAcceleration == LK_PER_FRAME_ACCEL_VPI_CUDA) {
            return "LK GFTT VPI CUDA";
        }
        return featureFrontendOptionToText(featureFrontend);
    }

    private String lkPerFrameAccelerationToText(int acceleration)
    {
        return acceleration == LK_PER_FRAME_ACCEL_VPI_CUDA ? "VPI CUDA" : "CPU";
    }

    private String slamModeToText(int slamMode)
    {
        switch (slamMode) {
        case SLAM_MODE_LOCALIZATION:
            return "Localization";
        case SLAM_MODE_AUTO:
            return "Auto";
        case SLAM_MODE_MAPPING:
        default:
            return "Mapping";
        }
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

        PendingAckAction(int command, String label, String pendingKey, AckSuccess onSuccess, Runnable onTimeout)
        {
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

    private String trackingStateToText(int trackingState)
    {
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

    private boolean tryHandleStatePacket(byte[] rx)
    {
        if (!isTlvPacket(rx) || rx.length < 15) {
            return false;
        }
        int cmd = rx[3] & 0xFF;
        int len = readU16Le(rx, 5);
        if (cmd != CMD_STATE || (len != 32 && len != 34 && len != 35 && len != 36 && len != 38) || rx.length < 15 + len + 2) {
            return false;
        }
        int payloadOffset = 15;
        int runtimeMode = rx[payloadOffset] & 0xFF;
        int slamMode = m_effectiveSlamMode;
        int trackingOffset = payloadOffset + 1;
        int armedOffset = -1;
        if (len >= 35) {
            slamMode = rx[payloadOffset + 1] & 0xFF;
            trackingOffset = payloadOffset + 2;
        }
        if (len >= 36) {
            armedOffset = trackingOffset + 1;
        }
        int trackingState = rx[trackingOffset] & 0xFF;
        if (armedOffset >= 0) {
            m_armLatched = (rx[armedOffset] & 0xFF) != 0;
            updateFlightButtons();
        }
        int resetBaseOffset = (armedOffset >= 0) ? (armedOffset + 1) : (trackingOffset + 1);
        int resetCounter = len >= 34 ? readU16Le(rx, resetBaseOffset) : 0;
        int resetMapCount = len >= 34 ? readU16Le(rx, resetBaseOffset + 2) : 0;
        int poseOffset = len >= 34 ? resetBaseOffset + 4 : payloadOffset + 4;
        float x = readF32Le(rx, poseOffset);
        float y = readF32Le(rx, poseOffset + 4);
        float z = readF32Le(rx, poseOffset + 8);
        float qw = readF32Le(rx, poseOffset + 12);
        float qx = readF32Le(rx, poseOffset + 16);
        float qy = readF32Le(rx, poseOffset + 20);
        float qz = readF32Le(rx, poseOffset + 24);
        if (len >= 38) {
            m_px4MainMode = rx[poseOffset + 28] & 0xFF;
            m_px4SubMode = rx[poseOffset + 29] & 0xFF;
            updateFlightButtons();
        }
        if (slamMode == SLAM_MODE_MAPPING || slamMode == SLAM_MODE_LOCALIZATION) {
            m_effectiveSlamMode = slamMode;
            updateQuickSlamModeButtons();
        }
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
                final float pitchDeg = quatPitchDeg(qw, qx, qy, qz);
                final float rollDeg = quatRollDeg(qw, qx, qy, qz);
                final float yawDeg = quatYawDeg(qw, qx, qy, qz);
                m_tvPose.setText(String.format(
                    Locale.US, "P %.2f %.2f %.2f\nR %.0f  P %.0f  Y %.0f", x, y, z, rollDeg, pitchDeg, yawDeg));
            } else if (runtimeMode == MODE_CALIB) {
                m_tvPose.setText("Waiting for vehicle pose...");
            } else {
                m_tvPose.setText("Waiting for vehicle pose...");
            }
        }
        return true;
    }

    private boolean tryHandleVideoPacket(byte[] rx)
    {
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
        boolean needReset = frameId != assembly.frameId || assembly.chunks == null || assembly.chunkCount != chunkCnt ||
                            assembly.totalSize != totalSize;
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

    private boolean tryHandleFeaturePacket(byte[] rx, int camIndex)
    {
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

    private void renderVideoFrame(int camIndex)
    {
        DisplayFrame displayFrame = m_displayFrames[camIndex];
        ImageView target = (camIndex == 0) ? m_ivVideoLeft : m_ivVideoRight;
        if (displayFrame.bitmap == null || target == null) {
            return;
        }
        Bitmap output = displayFrame.bitmap;
        FeatureFrame featureFrame = m_featureFrames[camIndex];
        final boolean frameIdMatches = featureFrame.frameId >= 0 && displayFrame.frameId >= 0 &&
                                       featureFrame.frameId == displayFrame.frameId;
        final boolean legacyTimeMatches = !frameIdMatches &&
                                          featureFrame.frameId < 0 && displayFrame.frameId < 0 &&
                                          Math.abs(featureFrame.frameTimeSec - displayFrame.frameTimeSec) <=
                                              FRAME_MATCH_TOLERANCE_SEC;
        if (m_sendFeature && m_showFeaturePoints && featureFrame.xs != null && (frameIdMatches || legacyTimeMatches)) {
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

    private Bitmap overlayFeaturePoints(Bitmap source, FeatureFrame featureFrame)
    {
        Bitmap mutable = source.copy(Bitmap.Config.ARGB_8888, true);
        if (mutable == null) {
            return source;
        }
        Canvas canvas = new Canvas(mutable);
        float scaleX = (featureFrame.width > 0) ? ((float)mutable.getWidth() / (float)featureFrame.width) : 1.0f;
        float scaleY = (featureFrame.height > 0) ? ((float)mutable.getHeight() / (float)featureFrame.height) : 1.0f;
        if (m_cfgFeatureFrontend == FEATURE_FRONTEND_LK) {
            drawLkGrid(canvas, mutable.getWidth(), mutable.getHeight());
        }
        for (int i = 0; i < featureFrame.count; ++i) {
            float x = featureFrame.xs[i] * scaleX;
            float y = featureFrame.ys[i] * scaleY;
            canvas.drawCircle(x, y, 5.0f, m_featurePaint);
        }
        return mutable;
    }

    private void drawLkGrid(Canvas canvas, int width, int height)
    {
        if (canvas == null || width <= 0 || height <= 0) {
            return;
        }
        final int cols = 8;
        final int rows = 6;
        for (int col = 1; col < cols; ++col) {
            float x = width * (col / (float)cols);
            canvas.drawLine(x, 0.0f, x, height, m_lkGridPaint);
        }
        for (int row = 1; row < rows; ++row) {
            float y = height * (row / (float)rows);
            canvas.drawLine(0.0f, y, width, y, m_lkGridPaint);
        }
    }

    private static boolean isTlvPacket(byte[] rx)
    {
        return rx != null && rx.length >= 4 && (rx[0] & 0xFF) == 0xAA && (rx[1] & 0xFF) == 0x55;
    }

    private void tickRxLoop()
    {
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
            if (tryHandleHeartbeatPacket(rx)) {
                continue;
            }
            tryHandleStatePoseForMap(rx);
            if (tryHandleStatePacket(rx)) {
                continue;
            }
            if (tryHandleCapabilitiesPacket(rx)) {
                continue;
            }
            if (tryHandleConfigPacket(rx)) {
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

    private void tickHeartbeatLoop()
    {
        String vehicleIp = (m_etVehicleIp != null && m_etVehicleIp.getText() != null) ? m_etVehicleIp.getText().toString().trim()
                                                                                       : "";
        if (vehicleIp.isEmpty()) {
            return;
        }
        if (!ensureVehicleConnection()) {
            return;
        }
        try {
            NativeUdp.sendHeartbeat();
        } catch (Throwable t) {
            m_tvStatus.setText("heartbeat send error: " + t.getMessage());
        }

        long nowMs = System.currentTimeMillis();
        if (m_lastVehicleHeartbeatMs == 0L || (nowMs - m_lastVehicleHeartbeatMs) <= HEARTBEAT_TIMEOUT_MS) {
            return;
        }
        if (!m_armLatched) {
            m_vehicleHeartbeatTimeoutHandled = false;
            return;
        }
        if (m_vehicleHeartbeatTimeoutHandled) {
            return;
        }
        m_vehicleHeartbeatTimeoutHandled = true;
        sendSimpleCmd("LAND(heartbeat timeout)", CMD_LAND);
        m_armLatched = false;
        m_lastFlightCommand = "LAND";
        updateFlightButtons();
        m_tvStatus.setText("Heartbeat timeout >3s, LAND triggered");
    }

    private void updateVideoStatsView()
    {
        if (m_tvVideoStats == null) {
            return;
        }
        long nowMs = System.currentTimeMillis();
        if (nowMs - m_lastVideoStatsMs < 250L) {
            return;
        }
        m_lastVideoStatsMs = nowMs;
        final String lastSeen =
            (m_lastVideoPacketMs == 0L) ? "never" : String.format(Locale.US, "%dms", (nowMs - m_lastVideoPacketMs));
        m_tvVideoStats.setText(String.format(
            Locale.US, "Video pkt=%d feat=%d(L%d/R%d) fuse=%d(L%d/R%d) ok=%d fail=%d bad=%d L=%d R=%d last=%s",
            m_videoPktCount, m_featurePktCount, m_featurePktCount0, m_featurePktCount1, m_featureMatchCount,
            m_featureMatchCount0, m_featureMatchCount1, m_videoFrameOk, m_videoDecodeFail, m_videoInvalidPkt,
            m_videoCamFrameOk0, m_videoCamFrameOk1, lastSeen));
    }

    private void tickJoystickControl()
    {
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

        float leftY = axisFromButtons(m_leftUpPressed, m_leftDownPressed);
        float leftX = (leftY == 0f) ? axisFromButtons(m_leftYawRightPressed, m_leftYawLeftPressed) : 0f;
        float rightX = axisFromButtons(m_rightRightPressed, m_rightLeftPressed);
        float rightY = axisFromButtons(m_rightForwardPressed, m_rightBackPressed);

        float leftMag = clamp01((float)Math.hypot(leftX, leftY));
        float rightMag = clamp01((float)Math.hypot(rightX, rightY));
        boolean active = leftX != 0f || leftY != 0f || rightX != 0f || rightY != 0f;

        if (m_tvJoystickState != null) {
            m_tvJoystickState.setText(String.format(
                Locale.US, "REMOTE L[up=%.2f yaw=%.2f] R[fwd=%.2f right=%.2f] magL=%.2f magR=%.2f %s", leftY, leftX,
                rightY, rightX, leftMag, rightMag, active ? "ACTIVE" : "CENTER"));
        }

        if (!active) {
            if (m_lastJoystickActive) {
                m_lastJoystickActive = false;
                sendHoldBurst(1, "HOLD(center)");
            }
            return;
        }
        m_lastJoystickActive = true;
        requestRemoteControlModeIfNeeded();

        float throttle = leftY;
        float yaw = leftX;
        float pitch = rightY;
        float roll = rightX;
        sendMoveRcJoystickCommand(throttle, yaw, pitch, roll, BUTTON_MAX_SPEED_MPS, "JOY RC");
    }

    private void startJoystickLoop()
    {
        if (m_joystickLoopRunning) {
            return;
        }
        m_joystickLoopRunning = true;
        m_lastJoystickTickMs = 0L;
        m_handler.post(m_joystickLoop);
    }

    private void stopJoystickLoop()
    {
        m_joystickLoopRunning = false;
        m_handler.removeCallbacks(m_joystickLoop);
    }

    private void startRxLoop()
    {
        if (m_rxLoopRunning) {
            return;
        }
        m_rxLoopRunning = true;
        m_handler.post(m_rxLoop);
    }

    private void stopRxLoop()
    {
        m_rxLoopRunning = false;
        m_handler.removeCallbacks(m_rxLoop);
    }

    private void startHeartbeatLoop()
    {
        if (m_heartbeatLoopRunning) {
            return;
        }
        m_heartbeatLoopRunning = true;
        m_handler.post(m_heartbeatLoop);
    }

    private void stopHeartbeatLoop()
    {
        m_heartbeatLoopRunning = false;
        m_handler.removeCallbacks(m_heartbeatLoop);
    }

    private void setSettingsVisible(boolean visible)
    {
        if (m_pageCommand == null) {
            return;
        }
        m_settingsVisible = visible;
        m_pageCommand.setVisibility(visible ? View.VISIBLE : View.GONE);
        updateDebugPanelVisibility();
    }

    @Override protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
                                        View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);

        m_ivVideoLeft = findViewById(R.id.ivVideoLeft);
        m_ivVideoRight = findViewById(R.id.ivVideoRight);
        m_map3dView = findViewById(R.id.map3dView);
        m_tvStatus = findViewById(R.id.tvStatus);
        m_tvPose = findViewById(R.id.tvPose);
        if (m_tvPose != null) {
            m_tvPose.setText("Waiting for vehicle pose...");
        }
        m_debugPanel = findViewById(R.id.debugPanel);
        m_remoteControlsBar = findViewById(R.id.remoteControlsBar);
        m_pageCommand = findViewById(R.id.pageCommand);
        m_mapPanel = findViewById(R.id.mapPanel);
        m_btnModeToggle = findViewById(R.id.btnModeToggle);
        m_btnArmToggle = findViewById(R.id.btnArm);
        m_btnEmergencyStop = findViewById(R.id.btnEmergencyStop);
        m_btnLand = findViewById(R.id.btnLand);
        m_btnToggleSlam = findViewById(R.id.btnToggleSlam);
        m_btnToggleCalib = findViewById(R.id.btnToggleCalib);
        m_btnAutoExposureToggle = findViewById(R.id.btnAutoExposureToggle);
        m_btnTbcOverrideToggle = findViewById(R.id.btnTbcOverrideToggle);
        m_spinnerSensorMode = findViewById(R.id.spinnerSensorMode);
        m_spinnerFeatureFrontend = findViewById(R.id.spinnerFeatureFrontend);
        m_btnQuickSlamAuto = findViewById(R.id.btnQuickSlamAuto);
        m_btnQuickSlamManual = findViewById(R.id.btnQuickSlamManual);
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
        m_lkGridPaint.setColor(Color.argb(150, 255, 255, 0));
        m_lkGridPaint.setStyle(Paint.Style.STROKE);
        m_lkGridPaint.setStrokeWidth(1.5f);

        m_btnCleanCalib = findViewById(R.id.btnCleanCalib);

        m_etVehicleIp = findViewById(R.id.etVehicleIp);
        if (m_etVehicleIp != null) {
            ArrayAdapter<String> vehicleIpAdapter =
                new ArrayAdapter<>(this, R.layout.dropdown_vehicle_ip_item, DEFAULT_VEHICLE_IPS);
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
        m_tvCfgPairMsValue = findViewById(R.id.tvCfgPairMsValue);
        m_tvCfgSlamFpsValue = findViewById(R.id.tvCfgSlamFpsValue);
        m_tvCfgTbcXValue = findViewById(R.id.tvCfgTbcXValue);
        m_tvCfgTbcYValue = findViewById(R.id.tvCfgTbcYValue);
        m_tvCfgTbcZValue = findViewById(R.id.tvCfgTbcZValue);
        m_tvCfgTbcRollValue = findViewById(R.id.tvCfgTbcRollValue);
        m_tvCfgTbcPitchValue = findViewById(R.id.tvCfgTbcPitchValue);
        m_tvCfgTbcYawValue = findViewById(R.id.tvCfgTbcYawValue);
        m_tvCfgOrbNFeaturesValue = findViewById(R.id.tvCfgOrbNFeaturesValue);
        m_tvCfgOrbScaleFactorValue = findViewById(R.id.tvCfgOrbScaleFactorValue);
        m_tvCfgOrbNLevelsValue = findViewById(R.id.tvCfgOrbNLevelsValue);
        m_tvCfgOrbIniThFastValue = findViewById(R.id.tvCfgOrbIniThFastValue);
        m_tvCfgOrbMinThFastValue = findViewById(R.id.tvCfgOrbMinThFastValue);
        m_tvCfgXFeatTopKValue = findViewById(R.id.tvCfgXFeatTopKValue);
        m_tvCfgXFeatMaxPointsValue = findViewById(R.id.tvCfgXFeatMaxPointsValue);
        m_tvCfgXFeatInputMaxWidthValue = findViewById(R.id.tvCfgXFeatInputMaxWidthValue);
        m_tvCfgXFeatInputMaxHeightValue = findViewById(R.id.tvCfgXFeatInputMaxHeightValue);
        m_sbCfgExposure = findViewById(R.id.sbCfgExposure);
        m_sbCfgGain = findViewById(R.id.sbCfgGain);
        m_sbCfgPairMs = findViewById(R.id.sbCfgPairMs);
        m_sbCfgSlamFps = findViewById(R.id.sbCfgSlamFps);
        m_sbCfgTbcX = findViewById(R.id.sbCfgTbcX);
        m_sbCfgTbcY = findViewById(R.id.sbCfgTbcY);
        m_sbCfgTbcZ = findViewById(R.id.sbCfgTbcZ);
        m_sbCfgTbcRoll = findViewById(R.id.sbCfgTbcRoll);
        m_sbCfgTbcPitch = findViewById(R.id.sbCfgTbcPitch);
        m_sbCfgTbcYaw = findViewById(R.id.sbCfgTbcYaw);
        m_sbCfgOrbNFeatures = findViewById(R.id.sbCfgOrbNFeatures);
        m_sbCfgOrbScaleFactor = findViewById(R.id.sbCfgOrbScaleFactor);
        m_sbCfgOrbNLevels = findViewById(R.id.sbCfgOrbNLevels);
        m_sbCfgOrbIniThFast = findViewById(R.id.sbCfgOrbIniThFast);
        m_sbCfgOrbMinThFast = findViewById(R.id.sbCfgOrbMinThFast);
        m_sbCfgXFeatTopK = findViewById(R.id.sbCfgXFeatTopK);
        m_sbCfgXFeatMaxPoints = findViewById(R.id.sbCfgXFeatMaxPoints);
        m_sbCfgXFeatInputMaxWidth = findViewById(R.id.sbCfgXFeatInputMaxWidth);
        m_sbCfgXFeatInputMaxHeight = findViewById(R.id.sbCfgXFeatInputMaxHeight);

        m_joystickLeft = findViewById(R.id.joystickLeft);
        m_joystickRight = findViewById(R.id.joystickRight);
        m_btnLeftUp = findViewById(R.id.btnLeftUp);
        m_btnLeftDown = findViewById(R.id.btnLeftDown);
        m_btnLeftYawLeft = findViewById(R.id.btnLeftYawLeft);
        m_btnLeftYawRight = findViewById(R.id.btnLeftYawRight);
        m_btnRightForward = findViewById(R.id.btnRightForward);
        m_btnRightBack = findViewById(R.id.btnRightBack);
        m_btnRightLeft = findViewById(R.id.btnRightLeft);
        m_btnRightRight = findViewById(R.id.btnRightRight);

        final String cm5Ip = "10.42.0.1";
        final int cm5CmdPort = DEFAULT_CMD_PORT;
        final int phoneVideoPort = DEFAULT_PHONE_VIDEO_PORT;
        m_vehicleIp = cm5Ip;
        m_vehicleCmdPort = cm5CmdPort;
        m_phoneVideoPort = phoneVideoPort;
        if (m_etVehicleIp != null) {
            m_etVehicleIp.setText(cm5Ip);
        }
        boolean ok;
        try {
            ok = NativeUdp.init(cm5Ip, cm5CmdPort, phoneVideoPort);
        } catch (Throwable t) {
            ok = false;
            m_udpReady = false;
            m_tvStatus.setText("Native init error: " + t.getMessage());
        }
        if (ok) {
            m_udpReady = true;
            m_tvStatus.setText("UDP ready cmd-> " + cm5Ip + ":" + cm5CmdPort + " video<-" + phoneVideoPort);
            m_lastVehicleHeartbeatMs = 0L;
            m_vehicleHeartbeatTimeoutHandled = false;
        } else {
            m_udpReady = false;
        }
        if (ok) {
            requestRuntimeMetadata();
        }
        startDiscoveryLoop();

        if (m_sbCfgExposure != null) {
            m_sbCfgExposure.setMax((EXPOSURE_MAX_US - EXPOSURE_MIN_US) / EXPOSURE_STEP_US);
            m_sbCfgExposure.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgExposureUs = EXPOSURE_MIN_US + progress * EXPOSURE_STEP_US;
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("Exposure", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgGain != null) {
            m_sbCfgGain.setMax(GAIN_MAX - GAIN_MIN);
            m_sbCfgGain.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgGain = GAIN_MIN + progress;
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("Gain", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgPairMs != null) {
            m_sbCfgPairMs.setMax(PAIR_MS_MAX - PAIR_MS_MIN);
            m_sbCfgPairMs.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgPairMs = PAIR_MS_MIN + progress;
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("Pair window", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgSlamFps != null) {
            m_sbCfgSlamFps.setMax(SLAM_FPS_MAX - SLAM_FPS_MIN);
            m_sbCfgSlamFps.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgSlamFps = SLAM_FPS_MIN + progress;
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("SLAM fps", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgTbcX != null) {
            m_sbCfgTbcX.setMax(TBC_TRANSLATION_MAX_MM - TBC_TRANSLATION_MIN_MM);
            m_sbCfgTbcX.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgTbcTx = quantizeTbcTranslationM((TBC_TRANSLATION_MIN_MM + progress) / 1000.0f);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("T_b_c1 tx", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgTbcY != null) {
            m_sbCfgTbcY.setMax(TBC_TRANSLATION_MAX_MM - TBC_TRANSLATION_MIN_MM);
            m_sbCfgTbcY.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgTbcTy = quantizeTbcTranslationM((TBC_TRANSLATION_MIN_MM + progress) / 1000.0f);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("T_b_c1 ty", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgTbcZ != null) {
            m_sbCfgTbcZ.setMax(TBC_TRANSLATION_MAX_MM - TBC_TRANSLATION_MIN_MM);
            m_sbCfgTbcZ.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgTbcTz = quantizeTbcTranslationM((TBC_TRANSLATION_MIN_MM + progress) / 1000.0f);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("T_b_c1 tz", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgTbcRoll != null) {
            m_sbCfgTbcRoll.setMax(TBC_ROLL_MAX_TENTH_DEG - TBC_ROLL_MIN_TENTH_DEG);
            m_sbCfgTbcRoll.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgTbcRollDeg =
                        quantizeTbcAngleDeg((TBC_ROLL_MIN_TENTH_DEG + progress) / 10.0f, TBC_ROLL_MIN_TENTH_DEG,
                                            TBC_ROLL_MAX_TENTH_DEG);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("T_b_c1 roll", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgTbcPitch != null) {
            m_sbCfgTbcPitch.setMax(TBC_PITCH_MAX_TENTH_DEG - TBC_PITCH_MIN_TENTH_DEG);
            m_sbCfgTbcPitch.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgTbcPitchDeg =
                        quantizeTbcAngleDeg((TBC_PITCH_MIN_TENTH_DEG + progress) / 10.0f, TBC_PITCH_MIN_TENTH_DEG,
                                            TBC_PITCH_MAX_TENTH_DEG);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("T_b_c1 pitch", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgTbcYaw != null) {
            m_sbCfgTbcYaw.setMax(TBC_YAW_MAX_TENTH_DEG - TBC_YAW_MIN_TENTH_DEG);
            m_sbCfgTbcYaw.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgTbcYawDeg =
                        quantizeTbcAngleDeg((TBC_YAW_MIN_TENTH_DEG + progress) / 10.0f, TBC_YAW_MIN_TENTH_DEG,
                                            TBC_YAW_MAX_TENTH_DEG);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("T_b_c1 yaw", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgOrbNFeatures != null) {
            m_sbCfgOrbNFeatures.setMax((ORB_NFEATURES_MAX - ORB_NFEATURES_MIN) / ORB_NFEATURES_STEP);
            m_sbCfgOrbNFeatures.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgOrbNFeatures = quantizeOrbNFeatures(ORB_NFEATURES_MIN + progress * ORB_NFEATURES_STEP);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("ORB nFeatures", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgOrbScaleFactor != null) {
            m_sbCfgOrbScaleFactor.setMax(ORB_SCALE_MAX_CENTI - ORB_SCALE_MIN_CENTI);
            m_sbCfgOrbScaleFactor.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgOrbScaleFactor = quantizeOrbScaleFactor((ORB_SCALE_MIN_CENTI + progress) / 100.0f);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("ORB scaleFactor", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgOrbNLevels != null) {
            m_sbCfgOrbNLevels.setMax(ORB_NLEVELS_MAX - ORB_NLEVELS_MIN);
            m_sbCfgOrbNLevels.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgOrbNLevels = quantizeOrbNLevels(ORB_NLEVELS_MIN + progress);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("ORB nLevels", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgOrbIniThFast != null) {
            m_sbCfgOrbIniThFast.setMax(ORB_FAST_TH_MAX - ORB_FAST_TH_MIN);
            m_sbCfgOrbIniThFast.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgOrbIniThFast = quantizeOrbFastThreshold(ORB_FAST_TH_MIN + progress);
                    if (m_cfgOrbMinThFast > m_cfgOrbIniThFast) {
                        m_cfgOrbMinThFast = m_cfgOrbIniThFast;
                    }
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("ORB iniThFAST", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgOrbMinThFast != null) {
            m_sbCfgOrbMinThFast.setMax(ORB_FAST_TH_MAX - ORB_FAST_TH_MIN);
            m_sbCfgOrbMinThFast.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgOrbMinThFast = quantizeOrbFastThreshold(ORB_FAST_TH_MIN + progress);
                    if (m_cfgOrbMinThFast > m_cfgOrbIniThFast) {
                        m_cfgOrbMinThFast = m_cfgOrbIniThFast;
                    }
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("ORB minThFAST", false, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgXFeatTopK != null) {
            m_sbCfgXFeatTopK.setMax(XFEAT_TOP_K_MAX - XFEAT_TOP_K_MIN);
            m_sbCfgXFeatTopK.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgXFeatTopK = quantizeXFeatTopK(XFEAT_TOP_K_MIN + progress);
                    m_cfgXFeatMaxPoints = quantizeXFeatMaxPoints(m_cfgXFeatMaxPoints, m_cfgXFeatTopK);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("XFeat TopK", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgXFeatMaxPoints != null) {
            m_sbCfgXFeatMaxPoints.setMax(XFEAT_MAX_POINTS_MAX - XFEAT_MAX_POINTS_MIN);
            m_sbCfgXFeatMaxPoints.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgXFeatMaxPoints = quantizeXFeatMaxPoints(XFEAT_MAX_POINTS_MIN + progress, m_cfgXFeatTopK);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("XFeat MaxPoints", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgXFeatInputMaxWidth != null) {
            m_sbCfgXFeatInputMaxWidth.setMax(XFEAT_INPUT_MAX_MAX / XFEAT_INPUT_MAX_STEP);
            m_sbCfgXFeatInputMaxWidth.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgXFeatInputMaxWidth = quantizeXFeatInputMax(progress * XFEAT_INPUT_MAX_STEP);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("XFeat Input Max Width", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        if (m_sbCfgXFeatInputMaxHeight != null) {
            m_sbCfgXFeatInputMaxHeight.setMax(XFEAT_INPUT_MAX_MAX / XFEAT_INPUT_MAX_STEP);
            m_sbCfgXFeatInputMaxHeight.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
                {
                    if (m_updatingConfigUi || !fromUser) {
                        return;
                    }
                    m_cfgXFeatInputMaxHeight = quantizeXFeatInputMax(progress * XFEAT_INPUT_MAX_STEP);
                    updateConfigViews();
                }

                @Override public void onStartTrackingTouch(SeekBar seekBar) {}

                @Override public void onStopTrackingTouch(SeekBar seekBar)
                {
                    sendCurrentRuntimeConfig("XFeat Input Max Height", true, PENDING_CONFIG, () -> {});
                }
            });
        }
        m_cfgExposureUs = quantizeExposureUs(m_cfgExposureUs);
        m_cfgGain = quantizeGain(m_cfgGain);
        m_cfgPairMs = quantizePairMs(m_cfgPairMs);
        m_cfgSlamFps = quantizeSlamFps(m_cfgSlamFps);
        m_cfgTbcTx = quantizeTbcTranslationM(m_cfgTbcTx);
        m_cfgTbcTy = quantizeTbcTranslationM(m_cfgTbcTy);
        m_cfgTbcTz = quantizeTbcTranslationM(m_cfgTbcTz);
        m_cfgTbcRollDeg = quantizeTbcAngleDeg(m_cfgTbcRollDeg, TBC_ROLL_MIN_TENTH_DEG, TBC_ROLL_MAX_TENTH_DEG);
        m_cfgTbcPitchDeg = quantizeTbcAngleDeg(m_cfgTbcPitchDeg, TBC_PITCH_MIN_TENTH_DEG, TBC_PITCH_MAX_TENTH_DEG);
        m_cfgTbcYawDeg = quantizeTbcAngleDeg(m_cfgTbcYawDeg, TBC_YAW_MIN_TENTH_DEG, TBC_YAW_MAX_TENTH_DEG);
        m_cfgOrbNFeatures = quantizeOrbNFeatures(m_cfgOrbNFeatures);
        m_cfgOrbScaleFactor = quantizeOrbScaleFactor(m_cfgOrbScaleFactor);
        m_cfgOrbNLevels = quantizeOrbNLevels(m_cfgOrbNLevels);
        m_cfgOrbIniThFast = quantizeOrbFastThreshold(m_cfgOrbIniThFast);
        m_cfgOrbMinThFast = quantizeOrbFastThreshold(m_cfgOrbMinThFast);
        if (m_cfgOrbMinThFast > m_cfgOrbIniThFast) {
            m_cfgOrbMinThFast = m_cfgOrbIniThFast;
        }
        m_cfgXFeatTopK = quantizeXFeatTopK(m_cfgXFeatTopK);
        m_cfgXFeatMaxPoints = quantizeXFeatMaxPoints(m_cfgXFeatMaxPoints, m_cfgXFeatTopK);
        m_cfgXFeatInputMaxWidth = quantizeXFeatInputMax(m_cfgXFeatInputMaxWidth);
        m_cfgXFeatInputMaxHeight = quantizeXFeatInputMax(m_cfgXFeatInputMaxHeight);
        m_cfgSlamMode = SLAM_MODE_MAPPING;
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
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, nextValue, m_sendFeature, m_sendMap, m_cfgAutoExposure, "Image stream",
                                          PENDING_CONFIG, () -> {
                                              m_sendImage = nextValue;
                                              if (!m_sendImage) {
                                                  if (m_ivVideoLeft != null)
                                                      m_ivVideoLeft.setImageDrawable(null);
                                                  if (m_ivVideoRight != null)
                                                      m_ivVideoRight.setImageDrawable(null);
                                              }
                                              updateStreamToggleButtons();
                                          });
            });
        }
        if (m_btnAutoExposureToggle != null) {
            m_btnAutoExposureToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final boolean nextValue = isChecked;
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap, nextValue,
                                          effectiveConfigLabel("Auto exposure", true), PENDING_CONFIG, () -> {
                                              m_cfgAutoExposure = nextValue;
                                              updateConfigViews();
                                          });
            });
        }
        if (m_btnTbcOverrideToggle != null) {
            m_btnTbcOverrideToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final boolean nextValue = isChecked;
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure,
                                          nextValue, m_cfgTbcTx, m_cfgTbcTy, m_cfgTbcTz, m_cfgTbcRollDeg,
                                          m_cfgTbcPitchDeg, m_cfgTbcYawDeg,
                                          "T_b_c1 override", PENDING_CONFIG, () -> {
                    m_cfgUseCustomTbc = nextValue;
                    updateConfigViews();
                });
            });
        }
        if (m_btnMapToggle != null) {
            m_btnMapToggle.setOnCheckedChangeListener((buttonView, isChecked) -> {
                if (m_updatingToggleUi) {
                    return;
                }
                final boolean nextValue = isChecked;
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, nextValue, m_cfgAutoExposure, "Map stream",
                                          PENDING_CONFIG, () -> {
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
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, nextValue, m_sendMap, m_cfgAutoExposure, "Feature stream",
                                          PENDING_CONFIG, () -> {
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
                    sendHoldBurst(1, "HOLD(remote off)");
                } else {
                    requestRemoteControlModeIfNeeded();
                }
                updateRemoteControlsVisibility();
            });
        }

        bindDirectionalButton(m_btnLeftUp, pressed -> m_leftUpPressed = pressed);
        bindDirectionalButton(m_btnLeftDown, pressed -> m_leftDownPressed = pressed);
        bindDirectionalButton(m_btnLeftYawLeft, pressed -> m_leftYawLeftPressed = pressed);
        bindDirectionalButton(m_btnLeftYawRight, pressed -> m_leftYawRightPressed = pressed);
        bindDirectionalButton(m_btnRightForward, pressed -> m_rightForwardPressed = pressed);
        bindDirectionalButton(m_btnRightBack, pressed -> m_rightBackPressed = pressed);
        bindDirectionalButton(m_btnRightLeft, pressed -> m_rightLeftPressed = pressed);
        bindDirectionalButton(m_btnRightRight, pressed -> m_rightRightPressed = pressed);

        m_btnModeToggle.setOnClickListener(v -> setSettingsVisible(!m_settingsVisible));
        if (savedInstanceState != null) {
            m_settingsVisible = savedInstanceState.getBoolean(KEY_SETTINGS_VISIBLE, false);
            m_debugVisible = savedInstanceState.getBoolean(KEY_DEBUG_VISIBLE, true);
            m_remoteVisible = savedInstanceState.getBoolean(KEY_REMOTE_VISIBLE, true);
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
                    if (nextArm) {
                        // Prime REMOTE/OFFBOARD right after arming so the first UP tap
                        // is not consumed by mode switching.
                        requestRemoteControlModeIfNeeded();
                    }
                });
            });
        }
        if (m_btnEmergencyStop != null) {
            m_btnEmergencyStop.setOnClickListener(
                v -> sendSimpleCmdAwaitAck("EMERGENCY_STOP", CMD_EMERGENCY_STOP, PENDING_EMERGENCY_STOP, () -> {
                    m_armLatched = false;
                    m_lastFlightCommand = "EMERGENCY_STOP";
                    updateFlightButtons();
                }));
        }
        if (m_btnLand != null) {
            m_btnLand.setOnClickListener(v -> {
                final boolean sendPosition = isPx4AutoMode();
                final String nextLabel = sendPosition ? "POSITION" : "LAND";
                final int nextCmd = sendPosition ? CMD_POSITION : CMD_LAND;
                final String pendingKey = sendPosition ? PENDING_POSITION : PENDING_LAND;
                sendSimpleCmdAwaitAck(nextLabel, nextCmd, pendingKey, () -> {
                    if ("LAND".equals(nextLabel)) {
                        m_armLatched = false;
                    }
                    m_lastFlightCommand = nextLabel;
                    updateFlightButtons();
                });
            });
        }
        if (m_btnCleanCalib != null) {
            m_btnCleanCalib.setOnClickListener(
                v -> sendSimpleCmdAwaitAck("CLEAN_CALIB", CMD_CALIB_CLEAN, PENDING_CLEAN_CALIB, () -> {}));
        }
        if (m_spinnerSensorMode != null) {
            m_spinnerSensorMode.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                @Override public void onItemSelected(AdapterView<?> parent, View view, int position, long id)
                {
                    if (m_updatingToggleUi || position < 0 || position >= m_availableSensorModes.length) {
                        return;
                    }
                    final int nextSensorMode = m_availableSensorModes[position];
                    if (nextSensorMode == m_sensorMode) {
                        return;
                    }
                    final int exposureUs = m_cfgExposureUs;
                    final float gain = (float)m_cfgGain;
                    final int pairMs = m_cfgPairMs;
                    final int slamFps = m_cfgSlamFps;
                    sendRuntimeConfigAwaitAck(exposureUs, gain, pairMs, slamFps, m_cfgSlamMode, nextSensorMode,
                                              m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure,
                                              effectiveConfigLabel("Sensor mode", true), PENDING_SENSOR, () -> {
                                                  m_sensorMode = nextSensorMode;
                                                  updateRuntimeButtons();
                                              });
                }

                @Override public void onNothingSelected(AdapterView<?> parent) {}
            });
        }
        if (m_spinnerFeatureFrontend != null) {
            m_spinnerFeatureFrontend.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
                @Override public void onItemSelected(AdapterView<?> parent, View view, int position, long id)
                {
                    if (m_updatingToggleUi || position < 0) {
                        return;
                    }
                    final int[] supportedFrontends = getSupportedFeatureFrontends();
                    final int nextOption =
                        position < supportedFrontends.length ? supportedFrontends[position] : FEATURE_FRONTEND_ORB;
                    final int nextFrontend = nextOption == FEATURE_FRONTEND_LK_XFEAT
                                                 ? FEATURE_FRONTEND_LK
                                                 : (nextOption == FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI
                                                        ? FEATURE_FRONTEND_LK_GFTT_PER_FRAME
                                                        : nextOption);
                    final boolean nextLkXFeatSeeding = nextOption == FEATURE_FRONTEND_LK_XFEAT;
                    final int nextLkPerFrameAcceleration =
                        nextOption == FEATURE_FRONTEND_LK_GFTT_PER_FRAME_VPI ? LK_PER_FRAME_ACCEL_VPI_CUDA
                                                                              : LK_PER_FRAME_ACCEL_CPU;
                    if (nextFrontend == m_cfgFeatureFrontend && nextLkXFeatSeeding == m_cfgLkXFeatSeeding &&
                        nextLkPerFrameAcceleration == m_cfgLkPerFrameAcceleration) {
                        return;
                    }
                    final boolean previousLkXFeatSeeding = m_cfgLkXFeatSeeding;
                    final int previousLkPerFrameAcceleration = m_cfgLkPerFrameAcceleration;
                    m_cfgLkXFeatSeeding = nextLkXFeatSeeding;
                    m_cfgLkPerFrameAcceleration = nextLkPerFrameAcceleration;
                    sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, m_cfgSlamMode,
                                              m_sensorMode, nextFrontend, m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure,
                                              effectiveConfigLabel("Feature frontend", true), PENDING_CONFIG, () -> {
                                                  m_cfgFeatureFrontend = nextFrontend;
                                                  m_cfgLkXFeatSeeding = nextLkXFeatSeeding;
                                                  m_cfgLkPerFrameAcceleration = nextLkPerFrameAcceleration;
                                                  updateRuntimeButtons();
                                              });
                    if (!isPending(PENDING_CONFIG)) {
                        m_cfgLkXFeatSeeding = previousLkXFeatSeeding;
                        m_cfgLkPerFrameAcceleration = previousLkPerFrameAcceleration;
                    }
                }

                @Override public void onNothingSelected(AdapterView<?> parent) {}
            });
        }
        if (m_btnQuickSlamAuto != null) {
            m_btnQuickSlamAuto.setOnClickListener(v -> {
                final int nextSlamMode = (m_cfgSlamMode == SLAM_MODE_AUTO) ? SLAM_MODE_MAPPING : SLAM_MODE_AUTO;
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, nextSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure,
                                          nextSlamMode == SLAM_MODE_AUTO ? "Enable Auto" : "Disable Auto",
                                          PENDING_CONFIG, () -> {
                                              m_cfgSlamMode = nextSlamMode;
                                              updateConfigViews();
                                              updateRuntimeButtons();
                                          });
            });
        }
        if (m_btnQuickSlamManual != null) {
            m_btnQuickSlamManual.setOnClickListener(v -> {
                if (m_cfgSlamMode == SLAM_MODE_AUTO) {
                    return;
                }
                final int nextSlamMode =
                    (m_cfgSlamMode == SLAM_MODE_LOCALIZATION) ? SLAM_MODE_MAPPING : SLAM_MODE_LOCALIZATION;
                sendRuntimeConfigAwaitAck(m_cfgExposureUs, (float)m_cfgGain, m_cfgPairMs, m_cfgSlamFps, nextSlamMode,
                                          m_sensorMode, m_cfgFeatureFrontend, m_sendImage, m_sendFeature, m_sendMap, m_cfgAutoExposure,
                                          "Switch to " + slamModeToText(nextSlamMode), PENDING_CONFIG, () -> {
                                              m_cfgSlamMode = nextSlamMode;
                                              updateConfigViews();
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
        startHeartbeatLoop();
    }

    @Override protected void onResume()
    {
        super.onResume();
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
                                        View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    @Override protected void onDestroy()
    {
        stopDiscoveryLoop();
        stopRxLoop();
        stopJoystickLoop();
        stopHeartbeatLoop();
        super.onDestroy();
        try {
            NativeUdp.close();
            m_udpReady = false;
        } catch (Throwable ignored) {
        }
    }

    @Override protected void onSaveInstanceState(Bundle outState)
    {
        outState.putBoolean(KEY_SETTINGS_VISIBLE, m_settingsVisible);
        outState.putBoolean(KEY_DEBUG_VISIBLE, m_debugVisible);
        outState.putBoolean(KEY_REMOTE_VISIBLE, m_remoteVisible);
        super.onSaveInstanceState(outState);
    }
}
