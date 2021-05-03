package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.MecanumDrive;
import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.encoderOdometry.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ThreeWheelOdometryTest extends OpMode {

    public static final double WHEEL_DIAMETER = 2;
    private static final double TRACK_WIDTH = 5.5;
    private static final double CENTER_WHEEL_OFFSET = -2.97;
    public final double TICKS_PER_REV = 8192;
    public final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    // declare drive motors
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;

    // declare encoders
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    public Motor.Encoder horizontalEncoder;

    public HolonomicOdometry odometry;
    public MecanumDrive drive;
    final int cpr = 448;
    final int rpm = 64;

    @Override
    public void init() {
        // initialize drive motors
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive", cpr, rpm);

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // multipliers on frontRightDrive and backLeftDrive are because of the weight imbalance on our robot
        // was .6
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive", cpr, rpm);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setInverted(true);
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive", cpr, rpm);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive = new Motor(hardwareMap, "BackRightDrive", cpr, rpm);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        frontRightDrive.setInverted(true);

        leftEncoder = backRightDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder = frontRightDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        horizontalEncoder = backLeftDrive.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(leftEncoder::getDistance, rightEncoder::getDistance,
                horizontalEncoder::getDistance, TRACK_WIDTH, CENTER_WHEEL_OFFSET);
        telemetry.addLine("Encoder Positions")
                .addData("Left", leftEncoder::getPosition)
                .addData("Right", rightEncoder::getPosition)
                .addData("Middle", horizontalEncoder::getPosition);

        drive = new MecanumDrive(true, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        horizontalEncoder.reset();
        leftEncoder.reset();
        rightEncoder.reset();
    }

    @Override
    public void loop() {
        odometry.updatePose();
        telemetry.addData("Position", odometry.robotPose);
        drive();
    }

    public void drive() {
        double strafePower = Math.abs(gamepad1.left_stick_y) < 0.1 ? 0 : gamepad1.left_stick_y;
        double forwardPower = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
        double turnPower = Math.abs(gamepad1.right_stick_x) < 0.1 ? 0 : gamepad1.right_stick_x;

        drive.driveRobotCentric(
                strafePower * 0.8,
                forwardPower * 0.8,
                turnPower * 0.7,
                true
        );
    }
}
