package org.firstinspires.ftc.teamcode.AutonomousOpModes;

import com.lcrobotics.easyftclib.CommandCenter.hardware.Motor;
import com.lcrobotics.easyftclib.encoderOdometry.EncoderOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class ThreeWheelOdometryTest extends OpMode {

    // declare drive motors
    public Motor frontLeftDrive;
    public Motor backLeftDrive;
    public Motor frontRightDrive;
    public Motor backRightDrive;

    // declare encoders
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    public Motor.Encoder frontEncoder;

    public EncoderOdometry odometry;

    @Override
    public void init() {
        backRightDrive = new Motor(hardwareMap, "BackRightDrive");
        backLeftDrive = new Motor(hardwareMap, "BackLeftDrive");
        frontRightDrive = new Motor(hardwareMap, "FrontRightDrive");
        frontLeftDrive = new Motor(hardwareMap, "FrontLeftDrive");

        leftEncoder = backRightDrive.encoder;
        rightEncoder = frontRightDrive.encoder;
        frontEncoder = backLeftDrive.encoder;

        odometry = new EncoderOdometry(leftEncoder, rightEncoder, frontEncoder);

    }

    @Override
    public void loop() {

    }
}
