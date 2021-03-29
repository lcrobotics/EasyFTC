package com.lcrobotics.easyftclib.encoderOdometry;

import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.tools.geometry.Pose2d;

public class EncoderOdometry {

    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1;

    public static double LATERAL_DISTANCE = 10;
    public static double FORWARD_OFFSET = 4;

    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    public Motor.Encoder frontEncoder;

    public Pose2d position;

    public EncoderOdometry(Motor.Encoder leftEncoder, Motor.Encoder rightEncoder, Motor.Encoder frontEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.frontEncoder = frontEncoder;

        this.position = new Pose2d();
    }


}
