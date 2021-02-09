package com.lcrobotics.easyftclib.CommandCenter.driveTrain;


import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.tools.geometry.Vector2d;

/**
 * Class to drive Mecanum robots.
 */
public class MecanumDrive extends DriveBase {

    private final Motor[] motors;
    private double rightMultiplier;

    /**
     * Base constructor for the mecanum drive. Reverses right side by default.
     *
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this(true, frontLeft, frontRight, backLeft, backRight);
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
    public MecanumDrive(boolean autoInvert, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
        setRightSideInverted(autoInvert);
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
        rightMultiplier = inverted ? -1.0 : 1.0;
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

    /**
     * Drives the mecanum drivebase with given powers. For TeleOp, call this repeatedly in
     * {@code void loop()} in an OpMode or {@code while (!isStopRequested() && opModeIsActive())}
     * in a LinearOpMode.
     *
     * @param strafe  strafing power
     * @param forward driving power
     * @param turn    turning power
     * @param square  whether to square the inputs
     */
    public void driveRobotCentric(double strafe, double forward, double turn, boolean square) {
        // square inputs if needed
        strafe = square ? clipRange(square(strafe)) : clipRange(strafe);
        forward = square ? clipRange(square(forward)) : clipRange(forward);
        turn = square ? clipRange(square(turn)) : clipRange(turn);

        driveRobotCentric(strafe, forward, turn);
    }

    /**
     * Drive method that defaults to not squaring the inputs.
     * See {@link #driveRobotCentric(double, double, double, boolean)}
     *
     * @param strafe  strafing power
     * @param forward driving power
     * @param turn    turning power
     */
    public void driveRobotCentric(double strafe, double forward, double turn) {
        driveFieldCentric(strafe, forward, turn, 0.0);
    }

    /**
     * Drives the robot from the perspective of the driver rather than that of the robot.
     * This means that pushing forward on the drive stick will always make the robot move away
     * from the driver, regardless of its orientation.
     *
     * @param strafe    strafing power
     * @param forward   driving power
     * @param turn      turning power
     * @param gyroAngle angle reported by the gyroscope
     */
    public void driveFieldCentric(double strafe, double forward, double turn, double gyroAngle) {

        // compensate for gyro angle
        Vector2d input = new Vector2d(strafe, forward);
        input.rotateBy(-gyroAngle);

        forward = input.getX();
        strafe = input.getY();

        double[] wheelSpeeds = new double[4];
        // calculate base speeds
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

    /**
     * Drives the robot from the perspective of the driver rather than that of the robot.
     * This means that pushing forward on the drive stick will always make the robot move away
     * from the driver, regardless of its orientation.
     *
     * @param strafe    strafing power
     * @param forward   driving power
     * @param turn      turning power
     * @param gyroAngle angle reported by the gyroscope
     * @param square    whether to square the inputs, allowing for finer control
     */
    public void driveFieldCentric(double strafe, double forward, double turn, double gyroAngle, boolean square) {
        strafe = square ? clipRange(square(strafe)) : clipRange(strafe);
        forward = square ? clipRange(square(forward)) : clipRange(forward);
        turn = square ? clipRange(square(turn)) : clipRange(turn);

        driveFieldCentric(strafe, forward, turn, gyroAngle);
    }

    /**
     * Convenience method to set motor powers using array
     *
     * @param powers array of motor powers
     */
    public void setPower(double... powers) {
        if (powers.length != motors.length) {
            throw new RuntimeException("Array length mismatch when setting motor powers");
        }
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(powers[i]);
        }
    }
}
