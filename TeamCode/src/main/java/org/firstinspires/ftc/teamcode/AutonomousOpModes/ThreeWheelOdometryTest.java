package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.encoderOdometry.EncoderOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class ThreeWheelOdometryTest extends OpMode {

    // constants (not accurate yet)
    private static final double TRACKWIDTH = 14.25;
    private static final double CENTERWHEELOFFSET = 2;

    // declare drive motors
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;

    // declare encoders
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    public Motor.Encoder horizontalEncoder;

    public EncoderOdometry odometry;

    @Override
    public void init() {
        backRightDrive = new Motor(hardwareMap, "BackRightDrive");
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive");
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive");
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive");

        leftEncoder = backRightDrive.encoder;
        rightEncoder = frontRightDrive.encoder;
        horizontalEncoder = backLeftDrive.encoder;

        odometry = new EncoderOdometry(leftEncoder::getPosition, rightEncoder::getPosition, horizontalEncoder::getPosition, TRACKWIDTH, CENTERWHEELOFFSET);

    }

    @Override
    public void loop() {
        odometry.updatePose();
        telemetry.addData("Position", odometry.robotPose);
    }
}
