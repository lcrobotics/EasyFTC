package com.lcrobotics.easyftclib.encoderOdometry;

import com.lcrobotics.easyftclib.tools.geometry.Pose2d;
import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.lcrobotics.easyftclib.tools.geometry.Twist2d;

import java.util.function.DoubleSupplier;

public class HolonomicOdometry extends Odometry {

    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    private final double centerWheelOffset;

    DoubleSupplier leftEncoder, rightEncoder, horizontalEncoder;

    public HolonomicOdometry(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                             DoubleSupplier horizontalEncoder, double trackWidth, double centerWheelOffset) {
        this(trackWidth, centerWheelOffset);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.horizontalEncoder = horizontalEncoder;
    }

    public HolonomicOdometry(Pose2d initialPose, double trackwidth, double centerWheelOffset) {
        super(initialPose, trackwidth);
        previousAngle = initialPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
    }

    public HolonomicOdometry(double trackWidth, double centerWheelOffset) {
        this(new Pose2d(), trackWidth, centerWheelOffset);
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(leftEncoder.getAsDouble(), rightEncoder.getAsDouble(), horizontalEncoder.getAsDouble());
    }

    @Override
    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftEncoder - deltaRightEncoder) / trackWidth
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
}
