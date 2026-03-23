package com.example.smartdrone;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class JoystickView extends View {
    private static final int AXIS_LOCK_NONE = 0;
    private static final int AXIS_LOCK_HORIZONTAL = 1;
    private static final int AXIS_LOCK_VERTICAL = 2;
    private static final float AXIS_LOCK_THRESHOLD = 0.12f;

    public interface OnStickChangedListener {
        void onStickChanged(float xNorm, float yNorm, boolean active);
    }

    private final Paint mBasePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mKnobPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mCrossPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private float mCenterX;
    private float mCenterY;
    private float mBaseRadius;
    private float mKnobRadius;

    private float mStickXNorm;
    private float mStickYNorm;
    private boolean mActive;
    private boolean mCardinalAxisLockEnabled;
    private int mAxisLock = AXIS_LOCK_NONE;
    private OnStickChangedListener mListener;

    public JoystickView(Context context) {
        super(context);
        Init();
    }

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        Init();
    }

    public JoystickView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        Init();
    }

    private void Init() {
        mBasePaint.setStyle(Paint.Style.FILL);
        mBasePaint.setColor(Color.parseColor("#203A4A"));

        mKnobPaint.setStyle(Paint.Style.FILL);
        mKnobPaint.setColor(Color.parseColor("#4FC3F7"));

        mCrossPaint.setStyle(Paint.Style.STROKE);
        mCrossPaint.setStrokeWidth(4f);
        mCrossPaint.setColor(Color.parseColor("#90A4AE"));

        setClickable(true);
    }

    public void SetOnStickChangedListener(OnStickChangedListener listener) {
        mListener = listener;
    }

    public float GetXNorm() {
        return mStickXNorm;
    }

    public float GetYNorm() {
        return mStickYNorm;
    }

    public boolean IsActive() {
        return mActive;
    }

    public void SetCardinalAxisLockEnabled(boolean enabled) {
        mCardinalAxisLockEnabled = enabled;
        if (!enabled) {
            mAxisLock = AXIS_LOCK_NONE;
        }
    }

    public void Reset() {
        mAxisLock = AXIS_LOCK_NONE;
        UpdateStick(0f, 0f, false, true);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldW, int oldH) {
        super.onSizeChanged(w, h, oldW, oldH);
        mCenterX = w * 0.5f;
        mCenterY = h * 0.5f;
        mBaseRadius = Math.min(w, h) * 0.42f;
        mKnobRadius = mBaseRadius * 0.28f;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        canvas.drawCircle(mCenterX, mCenterY, mBaseRadius, mBasePaint);
        canvas.drawLine(mCenterX - mBaseRadius, mCenterY, mCenterX + mBaseRadius, mCenterY, mCrossPaint);
        canvas.drawLine(mCenterX, mCenterY - mBaseRadius, mCenterX, mCenterY + mBaseRadius, mCrossPaint);

        float knobX = mCenterX + mStickXNorm * mBaseRadius;
        float knobY = mCenterY - mStickYNorm * mBaseRadius; // yNorm > 0 means up
        canvas.drawCircle(knobX, knobY, mKnobRadius, mKnobPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                mAxisLock = AXIS_LOCK_NONE;
                HandleTouch(event.getX(), event.getY(), true);
                return true;
            case MotionEvent.ACTION_MOVE:
                HandleTouch(event.getX(), event.getY(), true);
                return true;
            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                Reset();
                return true;
            default:
                return super.onTouchEvent(event);
        }
    }

    private void HandleTouch(float x, float y, boolean active) {
        float dx = x - mCenterX;
        float dy = y - mCenterY;
        float dist = (float) Math.sqrt(dx * dx + dy * dy);
        if (dist > mBaseRadius && dist > 0f) {
            float scale = mBaseRadius / dist;
            dx *= scale;
            dy *= scale;
        }

        float xNorm = (mBaseRadius > 0f) ? (dx / mBaseRadius) : 0f;
        float yNorm = (mBaseRadius > 0f) ? (-dy / mBaseRadius) : 0f; // up positive

        if (mCardinalAxisLockEnabled) {
            final float absX = Math.abs(xNorm);
            final float absY = Math.abs(yNorm);
            final int preferredLock = absY >= absX ? AXIS_LOCK_VERTICAL : AXIS_LOCK_HORIZONTAL;

            if (mAxisLock == AXIS_LOCK_NONE && (absX >= AXIS_LOCK_THRESHOLD || absY >= AXIS_LOCK_THRESHOLD)) {
                mAxisLock = preferredLock;
            }

            final int effectiveLock = (mAxisLock != AXIS_LOCK_NONE) ? mAxisLock : preferredLock;
            if (effectiveLock == AXIS_LOCK_VERTICAL) {
                xNorm = 0f;
            } else if (effectiveLock == AXIS_LOCK_HORIZONTAL) {
                yNorm = 0f;
            }
        }

        UpdateStick(xNorm, yNorm, active, true);
    }

    private void UpdateStick(float xNorm, float yNorm, boolean active, boolean notify) {
        mStickXNorm = Clamp(xNorm);
        mStickYNorm = Clamp(yNorm);
        mActive = active;
        invalidate();
        if (notify && mListener != null) {
            mListener.onStickChanged(mStickXNorm, mStickYNorm, mActive);
        }
    }

    private static float Clamp(float v) {
        return Math.max(-1f, Math.min(1f, v));
    }
}
