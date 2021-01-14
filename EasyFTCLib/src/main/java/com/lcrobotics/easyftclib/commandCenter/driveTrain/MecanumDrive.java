package com.lcrobotics.easyftclib.commandCenter.driveTrain;

import com.lcrobotics.easyftclib.commandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.commandCenter.old.WheelPosition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumDrive extends SmartTrainEx {

    private double rightMultiplier;

    private final double radius;
    private final Motor[] motors;

    /**
     * Base constructor for the mecanum drive. Reverses right side by default.
     *
     * @param radius     the wheel radius
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MecanumDrive(double radius, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this(true, radius, frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Constructor for the mecanum drive.
     *
     * @param autoInvert whether to automatically reverse the right motors
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MecanumDrive(boolean autoInvert, double radius, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
        this.radius = radius;

    }

    /**
     * @return True if the right side is inverted
     */
    public boolean isRightSideInverted() {
        return rightMultiplier == -1.0;
    }

    /**
     * @param inverted If true, sets the right multiplier to -1, 1 otherwise
     */
    public void setRightSideInverted(boolean inverted) {
        rightMultiplier = inverted? -1.0 : 1.0;
    }

    /**
     * Stops all motors
     */
    @Override
    public void stop() {
        for (Motor m : motors) {
            m.stopMotor();
        }
    }

    public void drive(double strafe, double forward, double turn, boolean square) {
        strafe = square? square(strafe) : strafe;
        forward = square? square(forward) : forward;
        turn = square? square(turn) : turn;

        strafe = clipRange(strafe);
        forward = clipRange(forward);
        turn = clipRange(turn);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[WheelPosition.FRONT_LEFT.value] = forward - strafe + turn;
        wheelSpeeds[WheelPosition.FRONT_RIGHT.value] = forward + strafe - turn;
        wheelSpeeds[WheelPosition.BACK_LEFT.value] = forward + strafe + turn;
        wheelSpeeds[WheelPosition.BACK_RIGHT.value] = forward - strafe - turn;

        normalize(wheelSpeeds);

        wheelSpeeds[WheelPosition.FRONT_LEFT.value] *= maxSpeed;
        wheelSpeeds[WheelPosition.FRONT_RIGHT.value] *= maxSpeed * rightMultiplier;
        wheelSpeeds[WheelPosition.BACK_LEFT.value] *= maxSpeed;
        wheelSpeeds[WheelPosition.BACK_RIGHT.value] *= maxSpeed * rightMultiplier;

        setPower(wheelSpeeds);
    }

    public void setPower(double[] powers) {
        if (powers.length != motors.length) {
            throw new RuntimeException("Array length mismatch when setting motor powers");
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(powers[i]);
        }
    }
}
