package com.example.smartdrone;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Map3DView extends View {

    private static final int MAX_TRACK_POINTS = 240;
    private static final int MAX_CLOUD_POINTS = 2400;

    private static final class Point3 {
        final float x;
        final float y;
        final float z;

        Point3(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    private final Paint m_panelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_axisXPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_axisYPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_axisZPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_trackPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_bodyPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_dotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_cloudPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint m_poseHaloPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF mPanelRect = new RectF();
    private final List<Point3> mTrack = new ArrayList<>();
    private final List<Point3> mCloud = new ArrayList<>();

    private float mPosX;
    private float mPosY;
    private float mPosZ;
    private float mRollDeg;
    private float mPitchDeg;
    private float mYawDeg;
    private boolean mHasPose;
    private boolean mZoomedIn;
    private float mViewCenterX;
    private float mViewCenterY;
    private float mViewCenterZ;
    private boolean mHasViewCenter;

    public Map3DView(Context context)
    {
        super(context);
        init();
    }

    public Map3DView(Context context, AttributeSet attrs)
    {
        super(context, attrs);
        init();
    }

    public Map3DView(Context context, AttributeSet attrs, int defStyleAttr)
    {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init()
    {
        m_panelPaint.setColor(Color.argb(170, 8, 15, 24));
        m_panelPaint.setStyle(Paint.Style.FILL);

        m_gridPaint.setColor(Color.argb(120, 170, 210, 230));
        m_gridPaint.setStyle(Paint.Style.STROKE);
        m_gridPaint.setStrokeWidth(1.6f);

        m_axisXPaint.setColor(Color.rgb(255, 99, 71));
        m_axisXPaint.setStrokeWidth(4f);
        m_axisYPaint.setColor(Color.rgb(80, 220, 140));
        m_axisYPaint.setStrokeWidth(4f);
        m_axisZPaint.setColor(Color.rgb(80, 170, 255));
        m_axisZPaint.setStrokeWidth(4f);

        m_trackPaint.setColor(Color.rgb(255, 210, 90));
        m_trackPaint.setStyle(Paint.Style.STROKE);
        m_trackPaint.setStrokeWidth(3f);

        m_bodyPaint.setColor(Color.WHITE);
        m_bodyPaint.setStyle(Paint.Style.STROKE);
        m_bodyPaint.setStrokeWidth(3f);

        m_textPaint.setColor(Color.WHITE);
        m_textPaint.setTextSize(26f);

        m_dotPaint.setColor(Color.WHITE);
        m_dotPaint.setStyle(Paint.Style.FILL);

        m_cloudPaint.setColor(Color.argb(220, 130, 230, 255));
        m_cloudPaint.setStyle(Paint.Style.FILL);

        m_poseHaloPaint.setColor(Color.argb(210, 255, 160, 40));
        m_poseHaloPaint.setStyle(Paint.Style.STROKE);
        m_poseHaloPaint.setStrokeWidth(3f);
    }

    public void setPose(float x, float y, float z, float rollDeg, float pitchDeg, float yawDeg, boolean valid)
    {
        mHasPose = valid;
        if (!valid) {
            invalidate();
            return;
        }
        mPosX = x;
        mPosY = y;
        mPosZ = z;
        mRollDeg = rollDeg;
        mPitchDeg = pitchDeg;
        mYawDeg = yawDeg;

        if (!mHasViewCenter) {
            mViewCenterX = x;
            mViewCenterY = y;
            mViewCenterZ = z;
            mHasViewCenter = true;
        } else {
            final float follow = 0.18f;
            mViewCenterX += (x - mViewCenterX) * follow;
            mViewCenterY += (y - mViewCenterY) * follow;
            mViewCenterZ += (z - mViewCenterZ) * 0.10f;
        }

        if (mTrack.isEmpty()) {
            mTrack.add(new Point3(x, y, z));
        } else {
            Point3 last = mTrack.get(mTrack.size() - 1);
            float dx = x - last.x;
            float dy = y - last.y;
            float dz = z - last.z;
            if ((dx * dx + dy * dy + dz * dz) > 0.0025f) {
                mTrack.add(new Point3(x, y, z));
                if (mTrack.size() > MAX_TRACK_POINTS) {
                    mTrack.remove(0);
                }
            }
        }
        invalidate();
    }

    public void setPointCloud(float[] xyz, int pointCount)
    {
        if (xyz != null) {
            int count = Math.min(pointCount, xyz.length / 3);
            for (int i = 0; i < count; ++i) {
                int base = i * 3;
                float x = xyz[base];
                float y = xyz[base + 1];
                float z = xyz[base + 2];
                if (!containsNearbyPoint(x, y, z, 0.08f)) {
                    mCloud.add(new Point3(x, y, z));
                    if (mCloud.size() > MAX_CLOUD_POINTS) {
                        mCloud.remove(0);
                    }
                }
            }
        }
        invalidate();
    }

    public void clearPointCloud()
    {
        mCloud.clear();
        mTrack.clear();
        if (mHasPose) {
            mViewCenterX = mPosX;
            mViewCenterY = mPosY;
            mViewCenterZ = mPosZ;
            mHasViewCenter = true;
        } else {
            mHasViewCenter = false;
        }
        invalidate();
    }

    public void toggleZoom()
    {
        mZoomedIn = !mZoomedIn;
        invalidate();
    }

    public boolean isZoomedIn() { return mZoomedIn; }

    private boolean containsNearbyPoint(float x, float y, float z, float threshold)
    {
        float threshold2 = threshold * threshold;
        for (int i = Math.max(0, mCloud.size() - 160); i < mCloud.size(); ++i) {
            Point3 point = mCloud.get(i);
            float dx = point.x - x;
            float dy = point.y - y;
            float dz = point.z - z;
            if (dx * dx + dy * dy + dz * dz <= threshold2) {
                return true;
            }
        }
        return false;
    }

    @Override protected void onDraw(Canvas canvas)
    {
        super.onDraw(canvas);

        float w = getWidth();
        float h = getHeight();
        if (w <= 0f || h <= 0f) {
            return;
        }

        mPanelRect.set(0f, 0f, w, h);
        canvas.drawRoundRect(mPanelRect, 24f, 24f, m_panelPaint);

        float cx = w * 0.50f;
        float cy = h * 0.70f;
        float scale = Math.min(w, h) * (mZoomedIn ? 0.24f : 0.14f);

        drawGrid(canvas, cx, cy, scale);
        drawWorldAxes(canvas, cx, cy, scale);
        drawPointCloud(canvas, cx, cy, scale);

        if (mHasPose) {
            drawTrack(canvas, cx, cy, scale);
            drawVehicle(canvas, cx, cy, scale);
            canvas.drawText(String.format(Locale.US, "P %.2f %.2f %.2f", mPosX, mPosY, mPosZ), 18f, 34f, m_textPaint);
            canvas.drawText(String.format(Locale.US, "R %.0f  P %.0f  Y %.0f", mRollDeg, mPitchDeg, mYawDeg), 18f, 64f,
                            m_textPaint);
            canvas.drawText(String.format(Locale.US, "Cloud %d", mCloud.size()), 18f, 94f, m_textPaint);
        } else {
            canvas.drawText("3D Map waiting pose", 18f, 34f, m_textPaint);
        }
    }

    private void drawPointCloud(Canvas canvas, float cx, float cy, float scale)
    {
        for (int i = 0; i < mCloud.size(); ++i) {
            Point3 point = mCloud.get(i);
            float[] p = project(cx, cy, scale, point.x, point.y, point.z);
            canvas.drawCircle(p[0], p[1], 2.4f, m_cloudPaint);
        }
    }

    private void drawGrid(Canvas canvas, float cx, float cy, float scale)
    {
        for (int i = -4; i <= 4; ++i) {
            drawLine3(canvas, cx, cy, scale, -4f, i, 0f, 4f, i, 0f, m_gridPaint);
            drawLine3(canvas, cx, cy, scale, i, -4f, 0f, i, 4f, 0f, m_gridPaint);
        }
    }

    private void drawWorldAxes(Canvas canvas, float cx, float cy, float scale)
    {
        drawLine3(canvas, cx, cy, scale, 0f, 0f, 0f, 1.5f, 0f, 0f, m_axisXPaint);
        drawLine3(canvas, cx, cy, scale, 0f, 0f, 0f, 0f, 1.5f, 0f, m_axisYPaint);
        drawLine3(canvas, cx, cy, scale, 0f, 0f, 0f, 0f, 0f, 1.15f, m_axisZPaint);
        float[] px = project(cx, cy, scale, 1.65f, 0f, 0f);
        float[] py = project(cx, cy, scale, 0f, 1.65f, 0f);
        float[] pz = project(cx, cy, scale, 0f, 0f, 1.28f);
        canvas.drawText("N", px[0] + 4f, px[1], m_textPaint);
        canvas.drawText("E", py[0] + 4f, py[1], m_textPaint);
        canvas.drawText("D", pz[0] + 4f, pz[1], m_textPaint);
    }

    private void drawTrack(Canvas canvas, float cx, float cy, float scale)
    {
        if (mTrack.size() < 2) {
            float[] p = project(cx, cy, scale, mPosX, mPosY, mPosZ);
            canvas.drawCircle(p[0], p[1], 5f, m_dotPaint);
            return;
        }
        Path path = new Path();
        for (int i = 0; i < mTrack.size(); ++i) {
            Point3 point = mTrack.get(i);
            float[] p = project(cx, cy, scale, point.x, point.y, point.z);
            if (i == 0) {
                path.moveTo(p[0], p[1]);
            } else {
                path.lineTo(p[0], p[1]);
            }
        }
        canvas.drawPath(path, m_trackPaint);
        float[] p = project(cx, cy, scale, mPosX, mPosY, mPosZ);
        canvas.drawCircle(p[0], p[1], 6f, m_dotPaint);
    }

    private void drawVehicle(Canvas canvas, float cx, float cy, float scale)
    {
        float[] center = project(cx, cy, scale, mPosX, mPosY, mPosZ);
        canvas.drawCircle(center[0], center[1], 11f, m_poseHaloPaint);
        canvas.drawCircle(center[0], center[1], 5.5f, m_dotPaint);

        float size = 0.34f;
        float[][] bodyPoints = new float[][] {{size, 0f, 0f},
                                              {-size * 0.7f, size * 0.42f, 0f},
                                              {-size * 0.7f, -size * 0.42f, 0f},
                                              {-size * 0.15f, 0f, size * 0.20f}};

        float[][] worldPoints = new float[bodyPoints.length][3];
        for (int i = 0; i < bodyPoints.length; ++i) {
            float[] rotated = rotateBody(bodyPoints[i][0], bodyPoints[i][1], bodyPoints[i][2]);
            worldPoints[i][0] = rotated[0] + mPosX;
            worldPoints[i][1] = rotated[1] + mPosY;
            worldPoints[i][2] = rotated[2] + mPosZ;
        }

        drawEdge(canvas, cx, cy, scale, worldPoints[0], worldPoints[1]);
        drawEdge(canvas, cx, cy, scale, worldPoints[1], worldPoints[2]);
        drawEdge(canvas, cx, cy, scale, worldPoints[2], worldPoints[0]);
        drawEdge(canvas, cx, cy, scale, worldPoints[0], worldPoints[3]);
        drawEdge(canvas, cx, cy, scale, worldPoints[1], worldPoints[3]);
        drawEdge(canvas, cx, cy, scale, worldPoints[2], worldPoints[3]);

        float[] axisX = rotateBody(size * 1.25f, 0f, 0f);
        float[] axisY = rotateBody(0f, size * 1.05f, 0f);
        float[] axisZ = rotateBody(0f, 0f, size * 1.05f);
        drawLine3(canvas, cx, cy, scale, mPosX, mPosY, mPosZ, mPosX + axisX[0], mPosY + axisX[1], mPosZ + axisX[2],
                  m_axisXPaint);
        drawLine3(canvas, cx, cy, scale, mPosX, mPosY, mPosZ, mPosX + axisY[0], mPosY + axisY[1], mPosZ + axisY[2],
                  m_axisYPaint);
        drawLine3(canvas, cx, cy, scale, mPosX, mPosY, mPosZ, mPosX + axisZ[0], mPosY + axisZ[1], mPosZ + axisZ[2],
                  m_axisZPaint);

        float[] nose = project(cx, cy, scale, mPosX + axisX[0], mPosY + axisX[1], mPosZ + axisX[2]);
        drawHeadingArrow(canvas, center[0], center[1], nose[0], nose[1]);
    }

    private void drawHeadingArrow(Canvas canvas, float x0, float y0, float x1, float y1)
    {
        canvas.drawLine(x0, y0, x1, y1, m_axisXPaint);
        float dx = x1 - x0;
        float dy = y1 - y0;
        float len = (float)Math.hypot(dx, dy);
        if (len < 1f) {
            return;
        }
        float ux = dx / len;
        float uy = dy / len;
        float arrow = 12f;
        float wing = 6f;
        float lx = x1 - ux * arrow - uy * wing;
        float ly = y1 - uy * arrow + ux * wing;
        float rx = x1 - ux * arrow + uy * wing;
        float ry = y1 - uy * arrow - ux * wing;
        canvas.drawLine(x1, y1, lx, ly, m_axisXPaint);
        canvas.drawLine(x1, y1, rx, ry, m_axisXPaint);
    }

    private void drawEdge(Canvas canvas, float cx, float cy, float scale, float[] a, float[] b)
    {
        drawLine3(canvas, cx, cy, scale, a[0], a[1], a[2], b[0], b[1], b[2], m_bodyPaint);
    }

    private void drawLine3(Canvas canvas, float cx, float cy, float scale, float x0, float y0, float z0, float x1,
                           float y1, float z1, Paint paint)
    {
        float[] p0 = project(cx, cy, scale, x0, y0, z0);
        float[] p1 = project(cx, cy, scale, x1, y1, z1);
        canvas.drawLine(p0[0], p0[1], p1[0], p1[1], paint);
    }

    private float[] rotateBody(float x, float y, float z)
    {
        double roll = Math.toRadians(mRollDeg);
        double pitch = Math.toRadians(mPitchDeg);
        double yaw = Math.toRadians(mYawDeg);

        double cr = Math.cos(roll);
        double sr = Math.sin(roll);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);

        double x1 = x;
        double y1 = cr * y - sr * z;
        double z1 = sr * y + cr * z;

        double x2 = cp * x1 + sp * z1;
        double y2 = y1;
        double z2 = -sp * x1 + cp * z1;

        double x3 = cy * x2 - sy * y2;
        double y3 = sy * x2 + cy * y2;
        double z3 = z2;

        return new float[] {(float)x3, (float)y3, (float)z3};
    }

    private float[] project(float cx, float cy, float scale, float x, float y, float z)
    {
        float px = x;
        float py = y;
        float pz = z;
        if (mHasViewCenter) {
            px -= mViewCenterX;
            py -= mViewCenterY;
            pz -= mViewCenterZ * 0.35f;
        }
        // Display E/D reversed so the whole map, vehicle pose, and axes stay aligned.
        py = -py;
        pz = -pz;
        float sx = cx + (px - py) * scale;
        float sy = cy - (px + py) * scale * 0.45f - pz * scale * 1.35f;
        return new float[] {sx, sy};
    }
}
