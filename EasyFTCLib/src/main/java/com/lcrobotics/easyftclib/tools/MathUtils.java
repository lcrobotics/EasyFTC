package com.lcrobotics.easyftclib.tools;

public class MathUtils {

    public static double angleWrap(double currentAngle) {
        if(currentAngle < -180) {
            return 360 + currentAngle;
        } else if (currentAngle > 180) {
            return -360 + currentAngle;
        } else {
            return currentAngle;
        }
    }
}
