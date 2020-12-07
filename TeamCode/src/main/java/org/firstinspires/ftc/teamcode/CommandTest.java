package org.firstinspires.ftc.teamcode;

import com.lcrobotics.easyftclib.CommandCenter.driveTrain.CommandData;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands.Command;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.Commands.Drive;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.SmartTrainGyro;
import com.lcrobotics.easyftclib.CommandCenter.driveTrain.WheelType;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Arrays;

@Autonomous
public class CommandTest extends OpMode {
    // motors
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;

    public SmartTrainGyro wheels;
    public Command drive;
    public BNO055IMU gyro;

    @Override
    public void init() {
        // init gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 100);
        // init motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");
        // reverse motors on right side
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        // set gyroscope and construct smart train
        wheels = new SmartTrainGyro(WheelType.MECANUM, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, 16, 5);
        wheels.setGyro(gyro);
        // create command
        drive = new Drive(100, 0, 0.5);
        Command.Parameters params = new Command.Parameters();
        drive.initialize(params);

        wheels.pushCommand(drive);
    }

    @Override
    public void loop() {
        wheels.update();
        telemetry.addData("Front Left Power", frontLeftDrive.getPower());
        telemetry.addData("Front Right Power", frontRightDrive.getPower());
        telemetry.addData("Back Left Power", backLeftDrive.getPower());
        telemetry.addData("Back Right Power", backRightDrive.getPower());
        telemetry.addData("Target Positions", Arrays.toString(CommandData.targetPositions));
        telemetry.addData("Current Positions", Arrays.toString(CommandData.currentPositions));
        telemetry.addData("Motor Powers", Arrays.toString(CommandData.motorPowers));
    }
}
