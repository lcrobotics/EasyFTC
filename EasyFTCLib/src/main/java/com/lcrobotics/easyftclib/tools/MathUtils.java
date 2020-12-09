package com.lcrobotics.easyftclib.tools;

public final class MathUtils {

    public static double angleWrap(double currentAngle) {
        if(currentAngle < -180) {
            return 360 + currentAngle;
        } else if (currentAngle > 180) {
            return -360 + currentAngle;
        } else {
            return currentAngle;
        }
    }

    public static int clamp(int val, int low, int high) {
        return Math.max(low, Math.min(val, high));
    }
    public static double clamp(double val, double low, double high) {
        return Math.max(low, Math.min(val, high));
    }
    public static float clamp(float val, float low, float high) {
        return Math.max(low, Math.min(val, high));
    }
}
