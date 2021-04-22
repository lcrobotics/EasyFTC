package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.encoderOdometry.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class ThreeWheelOdometryTest extends OpMode {

    // constants (not accurate yet)
    private static final double TRACK_WIDTH = 5.5;
    private static final double CENTER_WHEEL_OFFSET = -2.97;

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
    // if needed, one can add a gearing term here
    public static final double WHEEL_DIAMETER = 2;
    public final double TICKS_PER_REV = 8192;
    public final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    @Override
    public void init() {
        // Get motors from hardware map
        backRightDrive = new Motor(hardwareMap, "BackRightDrive");
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive");
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive");
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive");
        // set distance per pulse (how far each encoder tick is in inches)
        leftEncoder = backRightDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder = frontRightDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        horizontalEncoder = backLeftDrive.setDistancePerPulse(DISTANCE_PER_PULSE);
        // reset encoders
        horizontalEncoder.reset();
        leftEncoder.reset();
        rightEncoder.reset();

        telemetry.addLine("Encoder Positions")
                .addData("Left", leftEncoder::getPosition)
                .addData("Right", rightEncoder::getPosition)
                .addData("Middle", horizontalEncoder::getPosition);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                horizontalEncoder::getDistance,
                TRACK_WIDTH,
                CENTER_WHEEL_OFFSET);
    }

    @Override
    public void loop() {
        telemetry.addData("Position", odometry.robotPose);
        odometry.updatePose();
    }
}
