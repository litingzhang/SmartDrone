package com.example.ZControl;

import android.app.Activity;
import android.os.Bundle;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.util.Locale;

public class MainActivity extends Activity {

    private static final int CMD_PING     = 0x01;
    private static final int CMD_ARM      = 0x10;
    private static final int CMD_DISARM   = 0x11;
    private static final int CMD_OFFBOARD = 0x12;
    private static final int CMD_HOLD     = 0x13;
    private static final int CMD_LAND     = 0x14;

    private TextView tv;

    private static float parseF(EditText et, float defVal) {
        try {
            String s = et.getText().toString().trim();
            if (s.isEmpty()) return defVal;
            return Float.parseFloat(s);
        } catch (Throwable t) {
            return defVal;
        }
    }

    private static String hex(byte[] b) {
        if (b == null) return "null";
        StringBuilder sb = new StringBuilder(b.length * 2);
        for (byte v : b) sb.append(String.format(Locale.US, "%02X", v));
        return sb.toString();
    }

    private void sendSimpleCmd(String name, int cmd) {
        try {
            int seq = NativeUdp.sendCmd(cmd);
            tv.setText(name + " sent seq=" + seq + " cmd=0x" + Integer.toHexString(cmd));
        } catch (Throwable t) {
            tv.setText(name + " error: " + t.getMessage());
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        tv = findViewById(R.id.tvStatus);

        final String cm5Ip = "192.168.0.103";
        final int cm5Port = 14550;

        boolean ok;
        try {
            ok = NativeUdp.init(cm5Ip, cm5Port);
        } catch (Throwable t) {
            tv.setText("Native init error: " + t.getMessage());
            ok = false;
        }
        tv.setText(ok ? ("UDP ready -> " + cm5Ip + ":" + cm5Port) : "UDP init failed");

        Button btnArm = findViewById(R.id.btnArm);
        Button btnDisarm = findViewById(R.id.btnDisarm);
        Button btnOffboard = findViewById(R.id.btnOffboard);
        Button btnHold = findViewById(R.id.btnHold);
        Button btnLand = findViewById(R.id.btnLand);
        Button btnPing = findViewById(R.id.btnPing);

        Button btnMove = findViewById(R.id.btnMove);
        Button btnPoll = findViewById(R.id.btnPoll);

        EditText etX = findViewById(R.id.etX);
        EditText etY = findViewById(R.id.etY);
        EditText etZ = findViewById(R.id.etZ);
        EditText etYaw = findViewById(R.id.etYaw);
        EditText etMaxV = findViewById(R.id.etMaxV);

        btnArm.setOnClickListener(v -> sendSimpleCmd("ARM", CMD_ARM));
        btnDisarm.setOnClickListener(v -> sendSimpleCmd("DISARM", CMD_DISARM));
        btnOffboard.setOnClickListener(v -> sendSimpleCmd("OFFBOARD", CMD_OFFBOARD));
        btnHold.setOnClickListener(v -> sendSimpleCmd("HOLD", CMD_HOLD));
        btnLand.setOnClickListener(v -> sendSimpleCmd("LAND", CMD_LAND));
        btnPing.setOnClickListener(v -> sendSimpleCmd("PING", CMD_PING));

        btnMove.setOnClickListener(v -> {
            float x = parseF(etX, 0);
            float y = parseF(etY, 0);
            float z = parseF(etZ, 1.2f);
            float yaw = parseF(etYaw, 0);
            float maxV = parseF(etMaxV, 0.6f);

            try {
                int frameType = 2; // FRAME_NED=2 (按你 CM5 端解释)
                int seq = NativeUdp.sendMove(frameType, x, y, z, yaw, maxV);
                tv.setText(String.format(Locale.US,
                        "MOVE sent seq=%d frame=%d x=%.2f y=%.2f z=%.2f yaw=%.2f v=%.2f",
                        seq, frameType, x, y, z, yaw, maxV));
            } catch (Throwable t) {
                tv.setText("MOVE error: " + t.getMessage());
            }
        });

        btnPoll.setOnClickListener(v -> {
            try {
                byte[] rx = NativeUdp.pollRecv();
                tv.setText("RX: " + hex(rx));
            } catch (Throwable t) {
                tv.setText("poll error: " + t.getMessage());
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try { NativeUdp.close(); } catch (Throwable ignored) {}
    }
}
