package com.lcrobotics.easyftclib.commandCenter.hardware;

import com.lcrobotics.easyftclib.tools.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RevIMU extends GyroBase {
    BNO055IMU revIMU;
    /**
     * Heading relative to starting position.
     */
    double globalHeading;
    /**
     * Heading relative to last reset.
     */
    double relativeHeading;
    /**
     * Offset between global and relative headings.
     */
    double offset;
    private int multiplier;

    /**
     * Constructs a new RevIMU with the given configuration name.
     *
     * @param hw Hardware map from OpMode
     * @param name Name of IMU in configuration
     */
    public RevIMU(HardwareMap hw, String name) {
        revIMU = hw.get(BNO055IMU.class, name);
        multiplier = 1;
    }

    /**
     * Constructs a new RevIMU with default name of "imu".
     *
     * @param hw Hardware map from OpMode.
     */
    public RevIMU(HardwareMap hw) {
        this(hw, "imu");
    }

    /**
     * Initializes gyro with default parameters.
     */
    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        init(parameters);
    }

    /**
     * Initializes gyro with custom parameters.
     *
     * @param parameters Parameters to use for the gyro.
     */
    public void init(BNO055IMU.Parameters parameters) {
        revIMU.initialize(parameters);

        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
    }

    /**
     * Inverts the output of the gyro.
     */
    public void invertGyro() {
        multiplier *= -1;
    }

    /**
     * @return The relative heading of the robot
     */
    @Override
    public double getHeading() {
        globalHeading = revIMU.getAngularOrientation().firstAngle;
        relativeHeading = globalHeading + offset;

        return relativeHeading * multiplier;
    }

    /**
     * @return The absolute heading of the robot.
     */
    @Override
    public double getAbsoluteHeading() {
        return revIMU.getAngularOrientation().firstAngle * multiplier;
    }

    /**
     * @return The x, y, and z angles of the gyro.
     */
    @Override
    public double[] getAngles() {
        Orientation angles = revIMU.getAngularOrientation();

        return new double[]{angles.firstAngle, angles.secondAngle, angles.thirdAngle};
    }

    /**
     * @return The gyro heading as a Rotation2d
     */
    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void reset() {
        offset = -getHeading();
    }

    @Override
    public void disable() {
        revIMU.close();
    }

    @Override
    public String getDeviceType() {
        return "Rev Expansion Hub IMU";
    }

    public BNO055IMU getRevIMU() {
        return revIMU;
    }
}
